from concurrent.futures import ThreadPoolExecutor
import coloredlogs, logging
import time
from typing import Iterator
import os
import sys
import json
import asyncio

import carla

from opencda.scenario_testing.utils.yaml_utils import load_yaml

import grpc
from google.protobuf.json_format import MessageToJson
from google.protobuf.timestamp_pb2 import Timestamp

import ecloud_pb2 as ecloud
import ecloud_pb2_grpc as ecloud_rpc

logger = logging.getLogger(__name__)
coloredlogs.install(level='DEBUG', logger=logger)
logger.setLevel(logging.DEBUG)

cloud_config = load_yaml("cloud_config.yaml")
if cloud_config["log_level"] == "error":
    logger.setLevel(logging.ERROR)
elif cloud_config["log_level"] == "warning":
    logger.setLevel(logging.WARNING)
elif cloud_config["log_level"] == "info":
    logger.setLevel(logging.INFO)

ECLOUD_PUSH_BASE_PORT = 50101 # TODO: config

class EcloudClient:

    '''
    Wrapper Class around gRPC Vehicle Client Calls
    '''

    retry_opts = json.dumps({
                    "methodConfig": [
                    {
                        "name": [{"service": "ecloud.Ecloud"}],
                        "retryPolicy": {
                            "maxAttempts": 5,
                            "initialBackoff": "0.05s",
                            "maxBackoff": "0.5s",
                            "backoffMultiplier": 2,
                            "retryableStatusCodes": ["UNAVAILABLE"],
                        },
                    }]})

    def __init__(self, channel: grpc.Channel) -> None:
        self.channel = channel
        self.stub = ecloud_rpc.EcloudStub(self.channel)

    async def run(self) -> ecloud.Tick:
        count = 0
        pong = None
        async for ecloud_update in self.stub.SimulationStateStream(ecloud.Tick( tick_id = self.tick_id )):
            logger.debug("T%s:C%s", ecloud_update.tick_id, ecloud_update.command)
            assert(self.tick_id != ecloud_update.tick_id)
            self.tick_id = ecloud_update.tick_id
            count += 1
            pong = ecloud_update

        assert(pong != None)
        assert(count == 1)
        return pong

    async def register_vehicle(self, update: ecloud.VehicleUpdate) -> ecloud.SimulationInfo:
        sim_info = await self.stub.Client_RegisterVehicle(update)

        return sim_info

    async def send_vehicle_update(self, update: ecloud.VehicleUpdate) -> ecloud.Empty:
        empty = await self.stub.Client_SendUpdate(update)

        return empty

    async def get_waypoints(self, request: ecloud.WaypointRequest) -> ecloud.WaypointBuffer:
        buffer = await self.stub.Client_GetWaypoints(request)
        
        return buffer

    async def get_objects(self, request: ecloud.ObjectRequest) -> ecloud.ObjectBuffer:
        buffer = await self.stub.Client_GetObjects(request)

        return buffer

class EcloudPushServer(ecloud_rpc.EcloudServicer):

    '''
    Lightweight gRPC Server Class for Receiving Push Messages from Ochestrator
    '''

    def __init__(self,
                 que: asyncio.Queue):

        logger.info("eCloud push server initialized")
        self.que = que
        self.last_tick_id = -1
        self.last_tick_command = None
        self.port_no = 0

    def PushTick(self,
                 request: ecloud.Tick,
                 context: grpc.aio.ServicerContext) -> ecloud.Empty:

        tick = request # readability - gRPC prefers overrides preserve variable names
        logger.info("new tick - %s", tick)

        assert self.que.empty()
        if self.last_tick_id != tick.tick_id \
            or self.last_tick_command != tick.command:
            self.que.put_nowait(tick)
            self.last_tick_id = tick.tick_id
            self.last_tick_command = tick.command
        
        else:
            logger.error('received a duplicate tick - %s', tick)

        return ecloud.Empty()

async def ecloud_run_push_server(port,
                                 que: asyncio.Queue) -> None:
    '''
    runs a simple listen server that accepts event-based message from the central ecloud gRPC server
    '''

    logger.info("spinning up eCloud push server")
    server = grpc.aio.server()
    ecloud_rpc.add_EcloudServicer_to_server(EcloudPushServer(que), server)
    
    server_started = False
    while not server_started:
        try:
            listen_addr = f"0.0.0.0:{port}"
            server.add_insecure_port(listen_addr)
            server_started = True
        except Exception as port_exception: # pylint: disable=broad-exception-caught
            logger.warning("failed to start push server on port %s - incrementing port & retrying", 
                           port)
            port += 1
            continue

    logger.critical("started eCloud push server on port %s", port)

    if port >= ECLOUD_PUSH_BASE_PORT: # only push for vehicle clients
        que.put_nowait(port)

    await server.start()
    await server.wait_for_termination()
