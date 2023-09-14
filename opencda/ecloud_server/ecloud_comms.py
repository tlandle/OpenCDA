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

class EcloudClient:

    '''
    Wrapper Class around gRPC Vehicle Client Calls
    
    // CLIENT
    rpc Client_SendUpdate (VehicleUpdate) returns (SimulationState);
    rpc Client_RegisterVehicle (VehicleUpdate) returns (SimulationState);
    rpc Client_Ping (Ping) returns (Ping);
    rpc Client_GetWaypoints(WaypointRequest) returns (WaypointBuffer);
    '''

    def __init__(self, channel: grpc.Channel) -> None:
        self.channel = channel
        self.stub = ecloud_rpc.EcloudStub(self.channel)     

    async def run(self) -> ecloud.Ping:
        count = 0
        pong = None
        async for ecloud_update in self.stub.SimulationStateStream(ecloud.Ping( tick_id = self.tick_id )):
            logger.debug(f"T{ecloud_update.tick_id}:C{ecloud_update.command}")
            assert(self.tick_id != ecloud_update.tick_id)
            self.tick_id = ecloud_update.tick_id
            count += 1
            pong = ecloud_update

        assert(pong != None)
        assert(count == 1)
        return pong
        
    async def register_vehicle(self, update: ecloud.VehicleUpdate) -> ecloud.SimulationState:
        sim_state = await self.stub.Client_RegisterVehicle(update)

        return sim_state

    async def send_vehicle_update(self, update: ecloud.VehicleUpdate) -> ecloud.SimulationState:
        sim_state = await self.stub.Client_SendUpdate(update)

        return sim_state

    async def get_waypoints(self, request: ecloud.WaypointRequest) -> ecloud.WaypointBuffer:
        buffer = await self.stub.Client_GetWaypoints(request)

        return buffer

class EcloudPushServer(ecloud_rpc.EcloudServicer):

    '''
    Lightweight gRPC Server Class for Receiving Push Messages from Ochestrator
    '''

    def __init__(self, 
                 q: asyncio.Queue):
        
        logger.info("eCloud push server initialized")
        self.q = q

    async def PushTick(self, 
                       ping: ecloud.Ping, 
                       context: grpc.aio.ServicerContext) -> ecloud.Empty:

        logger.debug(f"PushTick(): ping - {ping.SerializeToString()}")
        assert(self.q.empty())
        self.q.put_nowait(ping)

        return ecloud.Empty()     

async def ecloud_run_push_server(port, 
                       q: asyncio.Queue) -> None:
    
    logger.info("spinning up eCloud push server")
    server = grpc.aio.server()
    ecloud_rpc.add_EcloudServicer_to_server(EcloudPushServer(q), server)
    listen_addr = f"[::]:{port}"
    server.add_insecure_port(listen_addr)
    print(f"starting eCloud push server on {listen_addr}")
    
    await server.start()
    await server.wait_for_termination()