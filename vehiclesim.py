# -*- coding: utf-8 -*-
"""
Script to run a simulated vehicle
"""

# Authors: Aaron Drysdale <adrysdale3@gatech.edu>
#        : Jordan Rapp <jrapp7@gatech.edu>


import argparse
import sys
import json
import asyncio
import os
import logging
import threading
import time
import queue

import carla
import numpy as np
import coloredlogs
import pickle

from opencda.version import __version__
from opencda.core.common.cav_world import CavWorld
from opencda.core.common.vehicle_manager import VehicleManager
from opencda.scenario_testing.utils.yaml_utils import load_yaml
from opencda.core.application.edge.networking import NetworkEmulator
from opencda.core.common.ecloud_config import EcloudConfig, eDoneBehavior
from opencda.ecloud_server.ecloud_comms import EcloudClient, ecloud_run_push_server

import grpc
from google.protobuf.json_format import MessageToJson
from google.protobuf.timestamp_pb2 import Timestamp

import ecloud_pb2 as ecloud
import ecloud_pb2_grpc as ecloud_rpc

logger = logging.getLogger(__name__)
coloredlogs.install(level='DEBUG', logger=logger)
logger.setLevel(logging.DEBUG)

# TODO: move to eCloudConfig
cloud_config = load_yaml("cloud_config.yaml")
CARLA_IP = cloud_config["carla_server_public_ip"]
ECLOUD_IP = cloud_config["ecloud_server_public_ip"]
VEHICLE_IP = cloud_config["vehicle_client_public_ip"]
ECLOUD_PUSH_BASE_PORT = 50101 # TODO: config

if cloud_config["log_level"] == "error":
    logger.setLevel(logging.ERROR)
elif cloud_config["log_level"] == "warning":
    logger.setLevel(logging.WARNING)
elif cloud_config["log_level"] == "info":
    logger.setLevel(logging.INFO)

#TODO: move to eCloudClient
def serialize_debug_info(vehicle_update, vehicle_manager) -> None:
    planer_debug_helper = vehicle_manager.agent.debug_helper
    planer_debug_helper_msg = ecloud.PlanerDebugHelper()
    planer_debug_helper.serialize_debug_info(planer_debug_helper_msg)
    vehicle_update.planer_debug_helper.CopyFrom( planer_debug_helper_msg )

    loc_debug_helper = vehicle_manager.localizer.debug_helper
    loc_debug_helper_msg = ecloud.LocDebugHelper()
    loc_debug_helper.serialize_debug_info(loc_debug_helper_msg)
    vehicle_update.loc_debug_helper.CopyFrom( loc_debug_helper_msg )

    client_debug_helper = vehicle_manager.debug_helper
    #logger.debug(vehicle_manager.debug_helper.perception_time_list)
    client_debug_helper_msg = ecloud.ClientDebugHelper()
    client_debug_helper.serialize_debug_info(client_debug_helper_msg)
    vehicle_update.client_debug_helper.CopyFrom(client_debug_helper_msg)

#TODO: move to eCloudClient
async def send_registration_to_ecloud_server(stub_, push_port) -> ecloud.SimulationInfo:
    request = ecloud.RegistrationInfo()
    request.vehicle_state = ecloud.VehicleState.REGISTERING
    try:
        request.container_name = os.environ["HOSTNAME"]
    except Exception as e:
        request.container_name = f"vehiclesim.py"

    request.vehicle_ip = VEHICLE_IP
    request.vehicle_port = push_port

    sim_info = await stub_.Client_RegisterVehicle(request)

    logger.info("vehicle ID %s received...", sim_info.vehicle_index)

    return sim_info

#TODO: move to eCloudClient
async def send_carla_data_to_opencda(stub_, vehicle_index, actor_id, vid) -> ecloud.SimulationInfo:
    message = {"vehicle_index": vehicle_index, "actor_id": actor_id, "vid": vid}
    logger.info("Vehicle: Sending Carla rpc %s", message)

    # send actor ID and vid to API
    update = ecloud.RegistrationInfo()
    update.vehicle_state = ecloud.VehicleState.CARLA_UPDATE
    update.vehicle_index = vehicle_index
    update.vid = vid
    update.actor_id = actor_id

    sim_info = await stub_.Client_RegisterVehicle(update)

    logger.info("send_carla_data_to_opencda: response received")

    return sim_info

#TODO: move to eCloudClient
async def send_vehicle_update(stub_, vehicle_update_):
    logger.debug("send_vehicle_update: sending")
    empty = await stub_.Client_SendUpdate(vehicle_update_)
    logger.debug("send_vehicle_update: send complete")
    return empty

def arg_parse():
    parser = argparse.ArgumentParser(description="OpenCDA Vehicle Simulation.")
    parser.add_argument("--apply_ml",
                        action='store_true',
                        help='whether ml/dl framework such as sklearn/pytorch is needed in the testing. '
                             'Set it to true only when you have installed the pytorch/sklearn package.')
    parser.add_argument('-i', "--ipaddress", type=str, default=CARLA_IP,
                        help="Specifies the ip address of the server to connect to. [Default: localhost]")
    parser.add_argument('-p', "--port", type=int, default=50051,
                        help="Specifies the port to connect to. [Default: 50051]")
    parser.add_argument('-v', "--verbose", action="store_true",
                            help="Make more noise")
    parser.add_argument('-q', "--quiet", action="store_true",
                            help="Make no noise")
    parser.add_argument('-c',"--container_id", type=int, default=0,
                        help="container ID #. Used as the counter from the base port for the eCloud push service")

    opt = parser.parse_args()
    return opt

async def main():
    #TODO: move to eCloudConfig
    # default params which can be over-written from the simulation controller
    SPECTATOR_INDEX = 0

    application = ["single"]
    version = "0.9.12"
    tick_id = 0
    reported_done = False
    push_q = asyncio.Queue()

    opt = arg_parse()
    if opt.verbose:
        logger.setLevel(logging.DEBUG)
    elif opt.quiet:
        logger.setLevel(logging.WARNING)
    logger.info("OpenCDA Version: %s", version)

    logging.basicConfig()

    # spawn push server
    push_port = ECLOUD_PUSH_BASE_PORT + opt.container_id
    push_server = asyncio.create_task(ecloud_run_push_server(push_port, push_q))

    await asyncio.sleep(1)

    push_port = await push_q.get() # make sure we get the actual port - try logic may have altered it.
    push_q.task_done()

    logger.info("push server spun up on port %s", push_port)

    # TODO: move to eCloudClient
    channel = grpc.aio.insecure_channel(
        target=f"{ECLOUD_IP}:{opt.port}",
        options=[
            ("grpc.lb_policy_name", "pick_first"),
            ("grpc.enable_retries", 1),
            ("grpc.keepalive_timeout_ms", 10000),
            ("grpc.service_config", EcloudClient.retry_opts),],
        )

    ecloud_server = ecloud_rpc.EcloudStub(channel)
    ecloud_update = await send_registration_to_ecloud_server(ecloud_server, push_port)
    vehicle_index = ecloud_update.vehicle_index
    assert( vehicle_index != None )

    test_scenario = ecloud_update.test_scenario
    application = ecloud_update.application
    version = ecloud_update.version

    logger.debug("main - application: %s", application)
    logger.debug("main - version: %s", version)

    # create CAV world
    cav_world = CavWorld(opt.apply_ml)

    logger.info("eCloud debug: creating VehicleManager vehicle_index: %s", vehicle_index)

    scenario_yaml = json.loads(test_scenario) #load_yaml(test_scenario)
    if 'debug_scenario' in scenario_yaml:
        logger.debug("main - test_scenario: %s", test_scenario)

    ecloud_config = EcloudConfig(scenario_yaml, logger)
    SPAWN_SLEEP_TIME = ecloud_config.get_client_spawn_ping_time_s()
    TICK_SLEEP_TIME = ecloud_config.get_client_tick_ping_time_s()
    WORLD_TIME_SLEEP_FACTOR = ecloud_config.get_client_world_tick_factor()
    NUM_SERVERS = ecloud_config.get_num_servers()
    NUM_PORTS = ecloud_config.get_num_ports()

    location_type = ecloud_config.get_location_type()
    done_behavior = ecloud_config.get_done_behavior()

    target_speed = None
    edge_sets_destination = False
    is_edge = False # TODO: added this to the actual protobuf message
    network_emulator = None
    edge_sets_destination = False
    verbose_updates = ecloud_config.do_verbose_update()
    if 'edge_list' in scenario_yaml['scenario']:
        is_edge = True
        # TODO: support multiple edges...
        #target_speed = scenario_yaml['scenario']['edge_list'][0]['target_speed']
        target_speed = scenario_yaml['scenario']['edge_list'][0]['members'][int(f'{vehicle_index}')]['behavior']['max_speed']
        print(target_speed)
        time.sleep(5)
        edge_sets_destination = scenario_yaml['scenario']['edge_list'][0]['edge_sets_destination'] \
            if 'edge_sets_destination' in scenario_yaml['scenario']['edge_list'][0] else False

    if opt.apply_ml:
        await asyncio.sleep(vehicle_index + 1)

    vehicle_manager = VehicleManager(vehicle_index=vehicle_index, config_yaml=scenario_yaml, application=application, cav_world=cav_world, \
                                     carla_version=version, location_type=location_type, run_distributed=True, is_edge=is_edge, perception_active=opt.apply_ml)

    if is_edge:
        network_emulator = NetworkEmulator(edge_sets_destination=edge_sets_destination,
                                            vehicle_manager=vehicle_manager)

    actor_id = vehicle_manager.vehicle.id
    vid = vehicle_manager.vid

    await send_carla_data_to_opencda(ecloud_server, vehicle_index, actor_id, vid)

    assert push_q.empty(), logger.exception("push_q had %s in it when it should have been empty", push_q.get_nowait())
    pong = await push_q.get()
    push_q.task_done()

    vehicle_manager.update_info()
    vehicle_manager.set_destination(
                vehicle_manager.vehicle.get_location(),
                vehicle_manager.destination_location,
                clean=True)

    logger.info("vehicle %s beginning scenario tick flow", vehicle_index)
    while pong.command != ecloud.Command.END:

        vehicle_update = ecloud.VehicleUpdate()
        if pong.command != ecloud.Command.TICK: # don't print tick message since there are too many
            logger.info("Vehicle: received cmd %s", pong.command)

        # HANDLE DEBUG DATA REQUEST
        if pong.command == ecloud.Command.REQUEST_DEBUG_INFO:
            vehicle_update.vehicle_state = ecloud.VehicleState.DEBUG_INFO_UPDATE
            serialize_debug_info(vehicle_update, vehicle_manager)

        # HANDLE TICK
        elif pong.command == ecloud.Command.TICK:
            client_start_timestamp = Timestamp()
            client_start_timestamp.GetCurrentTime()
            # update info runs BEFORE waypoint injection
            update_info_start_time = time.time()
            vehicle_manager.update_info()
            update_info_end_time = time.time()
            vehicle_manager.debug_helper.update_update_info_time((update_info_end_time-update_info_start_time)*1000)
            logger.debug("update_info complete")

            if is_edge:
                network_emulator.update_waypoints()

            if reported_done:
                target_speed = 0
            control = vehicle_manager.run_step(target_speed=target_speed)
            logger.debug("run_step complete")

            vehicle_update.tick_id = tick_id

            if control is None or vehicle_manager.is_close_to_scenario_destination():
                vehicle_update.vehicle_state = ecloud.VehicleState.TICK_DONE
                if not reported_done:
                    serialize_debug_info(vehicle_update, vehicle_manager)

                if control is not None and done_behavior == eDoneBehavior.CONTROL:
                    vehicle_manager.apply_control(control)

            else:
                vehicle_manager.apply_control(control)
                logger.debug("apply_control complete")

                step_timestamps = ecloud.Timestamps()
                step_timestamps.tick_id = tick_id
                step_timestamps.client_end_tstamp.GetCurrentTime()
                step_timestamps.client_start_tstamp.CopyFrom(client_start_timestamp)
                vehicle_manager.debug_helper.update_timestamp(step_timestamps)

                vehicle_update.vehicle_state = ecloud.VehicleState.TICK_OK
                vehicle_update.duration_ns = step_timestamps.client_end_tstamp.ToNanoseconds() - step_timestamps.client_start_tstamp.ToNanoseconds()

            if is_edge or vehicle_index == SPECTATOR_INDEX or verbose_updates:
                velocity = vehicle_manager.vehicle.get_velocity()
                pv = ecloud.Velocity()
                pv.x = velocity.x
                pv.y = velocity.y
                pv.z = velocity.z
                vehicle_update.velocity.CopyFrom(pv)

                transform = vehicle_manager.vehicle.get_transform()
                pt = ecloud.Transform()
                pt.location.x = transform.location.x
                pt.location.y = transform.location.y
                pt.location.z = transform.location.z
                pt.rotation.roll = transform.rotation.roll
                pt.rotation.yaw = transform.rotation.yaw
                pt.rotation.pitch = transform.rotation.pitch
                vehicle_update.transform.CopyFrom(pt)

            # vehicle_update.vehicle_state = ecloud.VehicleState.ERROR # TODO: handle error status
            # logger.error("ecloud_client error")

            #cur_location = vehicle_manager.vehicle.get_location()
            #logger.debug("send OK and location for vehicle_%s - is - x: %s, y: %s", vehicle_index, cur_location.x, cur_location.y)

        # block waiting for a response
        if not reported_done or done_behavior == eDoneBehavior.CONTROL:
            if not reported_done:
                vehicle_update.tick_id = tick_id
                vehicle_update.vehicle_index = vehicle_index
                logger.debug('vehicle_update: \n vehicle_index: %s \n tick_id: %s \n %s', vehicle_index, tick_id, vehicle_update)
                ecloud_update = await send_vehicle_update(ecloud_server, vehicle_update)

            if vehicle_update.vehicle_state == ecloud.VehicleState.TICK_DONE or vehicle_update.vehicle_state == ecloud.VehicleState.DEBUG_INFO_UPDATE:
                if vehicle_update.vehicle_state == ecloud.VehicleState.DEBUG_INFO_UPDATE and pong.command == ecloud.Command.REQUEST_DEBUG_INFO:
                    # we were asked for debug data and provided it, so NOW we exit
                    # TODO: this is better handled by done
                    logger.info("pushed DEBUG_INFO_UPDATE")

                reported_done = True
                logger.info("reported_done")

            assert push_q.empty(), logger.exception("push_q had %s in it when it should have been empty", push_q.get_nowait())
            pong = await push_q.get()
            push_q.task_done()
            assert( pong.tick_id != tick_id )
            tick_id = pong.tick_id

            if pong.command == ecloud.Command.PULL_WAYPOINTS_AND_TICK:
                wp_request = ecloud.WaypointRequest()
                wp_request.vehicle_index = vehicle_index
                waypoint_proto = await ecloud_server.Client_GetWaypoints(wp_request)
                network_emulator.enqueue_wp(waypoint_proto)
                pong.command = ecloud.Command.TICK

            elif pong.command == ecloud.Command.PULL_OBJECTS_AND_TICK:
              obj_request = ecloud.ObjectRequest()
              obj_request.vehicle_index = vehicle_index
              object_proto = awayt ecloud_server.Client_GetObjects(obj_request)
              network_emulator.enqueue_obj(obj_request)
              pong.command = ecloud.Command.TICK

            # HANDLE END
            elif pong.command == ecloud.Command.END:
                logger.critical("END received")
                break

        else: # done
            logger.info("EXIT destroy-on-done vehicle actor")
            break

    # end while
    vehicle_manager.destroy()
    push_server.cancel()
    logger.info("scenario complete. exiting.")
    sys.exit(0)

if __name__ == '__main__':
    try:
        asyncio.get_event_loop().run_until_complete(main())
    except KeyboardInterrupt:
        logger.info('exited by user.')
