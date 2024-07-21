# -*- coding: utf-8 -*-
"""
Utilize scenario manager to manage CARLA simulation construction. This script
is used for carla simulation only, and if you want to manage the Co-simulation,
please use cosim_api.py.
"""
# Author: Runsheng Xu <rxx3386@ucla.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib

import math
from queue import Queue
import random
from sqlite3 import connect
import sys
import json
from random import shuffle
import socket
import time
import json
import random
import copy
import hashlib
import os
import asyncio
import subprocess
import signal

from concurrent.futures import ThreadPoolExecutor, thread
import coloredlogs, logging
import threading
import time
from typing import Iterable
from queue import Queue
import heapq
from google.protobuf.timestamp_pb2 import Timestamp

from google.protobuf.json_format import MessageToJson
import grpc

import ecloud_pb2 as ecloud
import ecloud_pb2_grpc as ecloud_rpc
from omegaconf import OmegaConf
from omegaconf.listconfig import ListConfig

import carla
import numpy as np
import pandas as pd
import pickle

import matplotlib.pyplot as plt
#import k_means_constrained

from opencda.core.common.vehicle_manager_proxy import VehicleManagerProxy
from opencda.core.common.vehicle_manager import VehicleManager
from opencda.core.application.platooning.platooning_manager import \
    PlatooningManager
from opencda.core.common.rsu_manager import RSUManager
from opencda.core.common.cav_world import CavWorld
from opencda.scenario_testing.utils.customized_map_api import \
    load_customized_world, bcolors
from opencda.core.application.edge.edge_manager import \
     EdgeManager
from opencda.sim_debug_helper import SimDebugHelper
from opencda.client_debug_helper import ClientDebugHelper
from opencda.scenario_testing.utils.yaml_utils import load_yaml
import opencda.core.plan.drive_profile_plotting as open_plt

# TODO: make base ecloud folder
from opencda.core.common.ecloud_config import EcloudConfig
from opencda.ecloud_server.ecloud_comms import EcloudClient, EcloudPushServer, ecloud_run_push_server

logger = logging.getLogger(__name__)
coloredlogs.install(level='DEBUG', logger=logger)

cloud_config = load_yaml("cloud_config.yaml")
CARLA_IP = cloud_config["carla_server_public_ip"]
ECLOUD_IP = cloud_config["ecloud_server_public_ip"]
VEHICLE_IP = cloud_config["vehicle_client_public_ip"]
ECLOUD_PUSH_API_PORT = 50061 # TODO: config

if cloud_config["log_level"] == "error":
    logger.setLevel(logging.ERROR)
elif cloud_config["log_level"] == "warning":
    logger.setLevel(logging.WARNING)
elif cloud_config["log_level"] == "info":
    logger.setLevel(logging.INFO)

TIMEOUT_S = 10
TIMEOUT_MS = TIMEOUT_S * 1000
NSEC_TO_MSEC = 1/1000000
ECLOUD_PUSH_API_PORT = 50061 # TODO: config

def car_blueprint_filter(blueprint_library, carla_version='0.9.15'):
    """
    Exclude the uncommon vehicles from the default CARLA blueprint library
    (i.e., isetta, carlacola, cybertruck, t2).

    Parameters
    ----------
    blueprint_library : carla.blueprint_library
        The blueprint library that contains all models.

    carla_version : str
        CARLA simulator version, currently support 0.9.11 and 0.9.12. We need
        this as since CARLA 0.9.12 the blueprint name has been changed a lot.

    Returns
    -------
    blueprints : list
        The list of suitable blueprints for vehicles.
    """

    if carla_version == '0.9.15':
      blueprints = [
            blueprint_library.find('vehicle.audi.a2'),
            blueprint_library.find('vehicle.audi.tt'),
            blueprint_library.find('vehicle.ford.ambulance'),
            blueprint_library.find('vehicle.ford.crown'),
            blueprint_library.find('vehicle.mini.cooper_s_2021'),
            blueprint_library.find('vehicle.nissan.micra'),
            blueprint_library.find('vehicle.nissan.patrol'),
            blueprint_library.find('vehicle.nissan.patrol_2021'),
            blueprint_library.find('vehicle.tesla.cybertruck'),
            blueprint_library.find('vehicle.volkswagen.t2'),
            blueprint_library.find('vehicle.volkswagen.t2_2021'),
            blueprint_library.find('vehicle.micro.microlino'),
            blueprint_library.find('vehicle.dodge.charger_police'),
            blueprint_library.find('vehicle.dodge.charger_police_2020'),
            blueprint_library.find('vehicle.dodge.charger_2020'),
            blueprint_library.find('vehicle.jeep.wrangler_rubicon'),
            blueprint_library.find('vehicle.chevrolet.impala'),
            blueprint_library.find('vehicle.mini.cooper_s'),
            blueprint_library.find('vehicle.audi.etron'),
            blueprint_library.find('vehicle.mercedes.coupe'),
            blueprint_library.find('vehicle.mercedes.coupe_2020'),
            blueprint_library.find('vehicle.bmw.grandtourer'),
            blueprint_library.find('vehicle.toyota.prius'),
            blueprint_library.find('vehicle.citroen.c3'),
            blueprint_library.find('vehicle.ford.mustang'),
            blueprint_library.find('vehicle.tesla.model3'),
            blueprint_library.find('vehicle.lincoln.mkz_2017'),
            blueprint_library.find('vehicle.lincoln.mkz_2020'),
            blueprint_library.find('vehicle.seat.leon'),
            blueprint_library.find('vehicle.nissan.patrol'),
            blueprint_library.find('vehicle.nissan.micra')
        ]
    elif carla_version == '0.9.12':
        blueprints = [
            blueprint_library.find('vehicle.audi.a2'),
            blueprint_library.find('vehicle.audi.tt'),
            blueprint_library.find('vehicle.dodge.charger_police'),
            blueprint_library.find('vehicle.dodge.charger_police_2020'),
            blueprint_library.find('vehicle.dodge.charger_2020'),
            blueprint_library.find('vehicle.jeep.wrangler_rubicon'),
            blueprint_library.find('vehicle.chevrolet.impala'),
            blueprint_library.find('vehicle.mini.cooper_s'),
            blueprint_library.find('vehicle.audi.etron'),
            blueprint_library.find('vehicle.mercedes.coupe'),
            blueprint_library.find('vehicle.mercedes.coupe_2020'),
            blueprint_library.find('vehicle.bmw.grandtourer'),
            blueprint_library.find('vehicle.toyota.prius'),
            blueprint_library.find('vehicle.citroen.c3'),
            blueprint_library.find('vehicle.ford.mustang'),
            blueprint_library.find('vehicle.tesla.model3'),
            blueprint_library.find('vehicle.lincoln.mkz_2017'),
            blueprint_library.find('vehicle.lincoln.mkz_2020'),
            blueprint_library.find('vehicle.seat.leon'),
            blueprint_library.find('vehicle.nissan.patrol'),
            blueprint_library.find('vehicle.nissan.micra'),
        ]
    else:
        sys.exit("Since v0.1.0, we do not support version earlier than "
                 "CARLA v0.9.15")
            
    return blueprints


def multi_class_vehicle_blueprint_filter(label, blueprint_library, bp_meta):
    """
    Get a list of blueprints that have the class equals the specified label.

    Parameters
    ----------
    label : str
        Specified blueprint.

    blueprint_library : carla.blueprint_library
        The blueprint library that contains all models.

    bp_meta : dict
        Dictionary of {blueprint name: blueprint class}.

    Returns
    -------
    blueprints : list
        List of blueprints that have the class equals the specified label.

    """
    blueprints = [
        blueprint_library.find(k)
        for k, v in bp_meta.items() if v["class"] == label
    ]
    return blueprints


class ScenarioManager:
    """
    The manager that controls simulation construction, backgound traffic
    generation and CAVs spawning.

    Parameters
    ----------
    scenario_params : dict
        The dictionary contains all simulation configurations.

    carla_version : str
        CARLA simulator version, it currently supports 0.9.11 and 0.9.12

    xodr_path : str
        The xodr file to the customized map, default: None.

    town : str
        Town name if not using customized map, eg. 'Town06'.

    apply_ml : bool
        Whether need to load dl/ml model(pytorch required) in this simulation.

    Attributes
    ----------
    client : carla.client
        The client that connects to carla server.

    world : carla.world
        Carla simulation server.

    origin_settings : dict
        The origin setting of the simulation server.

    cav_world : opencda object
        CAV World that contains the information of all CAVs.

    carla_map : carla.map
        Carla HD Map.

    """

    tick_id = 0 # current tick counter

    vehicle_managers = {}
    vehicles = {} # vehicle_index -> tuple (actor_id, vid)
    vehicle_count = 0

    rsu_managers = {}

    carla_version = None
    application = ['single']
    scenario = None
    ecloud_server = None
    is_edge = False
    vehicle_state = ecloud.VehicleState.REGISTERING

    debug_helper = SimDebugHelper(0)
    sm_start_tstamp = Timestamp()
    SPECTATOR_INDEX = 0

    async def server_unpack_debug_data(self, stub_):
        logger.info("fetching vehicle updates")
        vehicle_updates_list = []
        while True:
            ecloud_update = await stub_.Server_GetVehicleUpdates(ecloud.Empty())
            if len(ecloud_update.vehicle_update) == 0:
                break
            for v in ecloud_update.vehicle_update:
                u = ecloud.VehicleUpdate()
                u.CopyFrom(v)
                vehicle_updates_list.append(u)
            await asyncio.sleep(0.1)
        #logger.debug("%s", ecloud_update)
        for vehicle_update in vehicle_updates_list:
            vehicle_manager_proxy = self.vehicle_managers[ vehicle_update.vehicle_index ]
            vehicle_manager_proxy.localizer.debug_helper.deserialize_debug_info( vehicle_update.loc_debug_helper )
            vehicle_manager_proxy.agent.debug_helper.deserialize_debug_info( vehicle_update.planer_debug_helper )
            vehicle_manager_proxy.debug_helper.deserialize_debug_info(vehicle_update.client_debug_helper)

            latencies_by_tick = self.debug_helper.network_time_dict
            overall_steps_by_tick = self.debug_helper.client_tick_time_dict
            for timestamps in vehicle_manager_proxy.debug_helper.timestamps_list:
                if timestamps.tick_id in overall_steps_by_tick:
                    assert timestamps.tick_id in latencies_by_tick, logger.exception('%s not in latencies_by_tick: %s', timestamps.tick_id, latencies_by_tick)
                    client_process_time_ms = (timestamps.client_end_tstamp.ToNanoseconds() - timestamps.client_start_tstamp.ToNanoseconds()) * NSEC_TO_MSEC # doing work
                    idle_time_ms = overall_steps_by_tick[timestamps.tick_id] - latencies_by_tick[timestamps.tick_id] - client_process_time_ms # inferred rather than actual "idle" time
                    #if idle_time_ms < 0:
                    #    logger.warning("got a NEGATIVE inferred idle_time value of %sms for vehicle %s", round(idle_time_ms, 2), v.vehicle_index)
                    #idle_time_ms = idle_time_ms if idle_time_ms > 0 else 0 # TODO: confirm if we wantt to do this?
                    logger.debug("timestamps: client_end - %s client_start - %s", timestamps.client_end_tstamp.ToDatetime().time(), timestamps.client_start_tstamp.ToDatetime().time())
                    logger.info('client process time: %sms', round(client_process_time_ms, 2))
                    logger.info('idle time: %sms', round(idle_time_ms, 2))
                    self.debug_helper.update_idle_time_timestamp(vehicle_manager_proxy.vehicle_index, idle_time_ms) # this inferred
                    self.debug_helper.update_client_process_time_timestamp(vehicle_manager_proxy.vehicle_index, client_process_time_ms) # how long client actually was active

                    # dupe the data since it makes evaluation simpler
                    self.debug_helper.update_network_time_per_client_timestamp(vehicle_manager_proxy.vehicle_index, latencies_by_tick[timestamps.tick_id])
                    self.debug_helper.update_overall_step_time_per_client_timestamp(vehicle_manager_proxy.vehicle_index, overall_steps_by_tick[timestamps.tick_id])

                    logger.debug("updated time stamp data for vehicle %s", vehicle_manager_proxy.vehicle_index)

    async def server_unpack_vehicle_updates(self, stub_):
        logger.debug("getting vehicle updates")
        ecloud_update = await stub_.Server_GetVehicleUpdates(ecloud.Empty())
        logger.debug("unpacking vehicle updates")
        try:
            for vehicle_update in ecloud_update.vehicle_update:
                if not vehicle_update.HasField('transform') or not vehicle_update.HasField('velocity'):
                    continue

                if not ( self.is_edge or self.verbose_updates ) and vehicle_update.vehicle_index != ScenarioManager.SPECTATOR_INDEX:
                    continue

                vehicle_manager_proxy = self.vehicle_managers[ vehicle_update.vehicle_index ]
                if hasattr( vehicle_manager_proxy.vehicle, 'is_proxy' ) or self.verbose_updates :
                    #logger.debug("updating transform & velocity - %s", vehicle_update)
                    t = carla.Transform(
                    carla.Location(
                        x=vehicle_update.transform.location.x,
                        y=vehicle_update.transform.location.y,
                        z=vehicle_update.transform.location.z),
                    carla.Rotation(
                        yaw=vehicle_update.transform.rotation.yaw,
                        roll=vehicle_update.transform.rotation.roll,
                        pitch=vehicle_update.transform.rotation.pitch))
                    v = carla.Vector3D(
                        x=vehicle_update.velocity.x,
                        y=vehicle_update.velocity.y,
                        z=vehicle_update.velocity.z)
                    if hasattr( vehicle_manager_proxy.vehicle, 'is_proxy' ):
                        #logger.debug("updating transform & velocity - %s", vehicle_update)
                        vehicle_manager_proxy.vehicle.set_velocity(v)
                        vehicle_manager_proxy.vehicle.set_transform(t)  
                    
                    self.debug_helper.update_velocity_per_client_timestamp(tick_id=self.tick_id,
                                                                           velocity=v)
        except:
            logger.exception('%s', vehicle_update)

        logger.debug("vehicle updates unpacked")

    async def server_push_waypoints(self, stub_, wps_):
        empty = await stub_.Server_PushEdgeWaypoints(wps_)

        return empty

    async def server_do_tick(self, stub_, update_):
        empty = await stub_.Server_DoTick(update_)

        assert self.push_q.empty(), logger.exception("push_q should have been empty, but had %s", self.push_q.get_nowait())
        tick = await self.push_q.get()
        snapshot_t = time.time_ns()
        self.push_q.task_done()

        # the first tick time is dramatically slower due to startup, so we don't want it to skew runtime data
        if self.tick_id == 1:
            self.debug_helper.startup_time_ms = ( snapshot_t - self.sm_start_tstamp.ToNanoseconds() ) * NSEC_TO_MSEC
            return empty

        overall_step_time_ms = ( snapshot_t - self.sm_start_tstamp.ToNanoseconds() ) * NSEC_TO_MSEC # barrier sync means this is the same for ALL vehicles per tick
        step_latency_ms = overall_step_time_ms - ( tick.last_client_duration_ns * NSEC_TO_MSEC ) # we care about the worst case per tick - how much did we affect the final vehicle to report. This captures both delay in getting that vehicle started and in it reporting its completion
        logger.info("timestamps: overall_step_time_ms - %sms | step_latency_ms - %sms", round(overall_step_time_ms, 2), round(step_latency_ms, 2))
        self.debug_helper.update_network_time_timestamp(tick.tick_id, step_latency_ms) # same for all vehicles *per tick*
        self.debug_helper.update_overall_step_time_timestamp(tick.tick_id, overall_step_time_ms)

        if update_.command == ecloud.Command.REQUEST_DEBUG_INFO:
            await self.server_unpack_debug_data(stub_)

        else:
            await self.server_unpack_vehicle_updates(stub_)

        return empty

    async def server_start_scenario(self, stub_, update_):
        await stub_.Server_StartScenario(update_)

        print(f"pushed scenario start")
        print(f"start {self.vehicle_count} vehicle containers")

        assert self.push_q.empty(), logger.exception("push_q had %s in it when it should have been empty", self.push_q.get_nowait())
        await self.push_q.get()
        self.push_q.task_done()

        logger.info("vehicle registration complete")

        response = await stub_.Server_GetVehicleUpdates(ecloud.Empty())

        logger.info("vehicle registration data received")


        return response

    async def server_end_scenario(self, stub_):
        empty = await stub_.Server_EndScenario(ecloud.Empty())

        return empty

    debug_helper = SimDebugHelper(0)

    def __init__(self, scenario_params,
                 apply_ml,
                 carla_version,
                 xodr_path=None,
                 town=None,
                 cav_world=None,
                 #config_file=None,
                 distributed=False):

        #self.config_file = config_file
        self.ecloud_config = EcloudConfig(scenario_params, logger)
        self.sm_start_tstamp.GetCurrentTime()
        self.scenario_params = scenario_params
        self.carla_version = carla_version
        self.perception = scenario_params['perception_active'] if 'perception_active' in scenario_params else False

        simulation_config = scenario_params['world']

        self.run_distributed = distributed
        if distributed and ( ECLOUD_IP == 'localhost' or ECLOUD_IP == CARLA_IP ):
            server_log_level = 0 if logger.getEffectiveLevel() == logging.DEBUG else \
                                1 if logger.getEffectiveLevel() == logging.WARNING else 2 # 1: WARNING | 2: ERROR
            try:
                ecloud_pid = subprocess.check_output(['pgrep','ecloud_server'])
            except subprocess.CalledProcessError as e:
                if e.returncode > 1:
                    raise
                ecloud_pid = None
            if ecloud_pid is not None:
                logger.info('killing existing ecloud gRPC server process')
                subprocess.run(['pkill','-9','ecloud_server'])

            self.ecloud_server_process = subprocess.Popen(['./opencda/ecloud_server/ecloud_server',f'--minloglevel={server_log_level}'], stderr=sys.stdout.buffer)

        cav_world.update_scenario_manager(self)

        random.seed(time.time())
        # set random seed if stated
        if 'seed' in simulation_config:
            np.random.seed(simulation_config['seed'])
            random.seed(simulation_config['seed'])

        self.client = \
            carla.Client(CARLA_IP, simulation_config['client_port'])
        self.client.set_timeout(10.0)

        if xodr_path:
            self.world = load_customized_world(xodr_path, self.client)
        elif town:
            try:
                print(self.client.get_available_maps())
                self.world = self.client.load_world(town)
            except RuntimeError:
                logger.error(
                    f"{bcolors.FAIL} %s is not found in your CARLA repo! "
                    f"Please download all town maps to your CARLA "
                    f"repo!{bcolors.ENDC}" % town)
        else:
            self.world = self.client.get_world()

        if not self.world:
            sys.exit('World loading failed')

        self.origin_settings = self.world.get_settings()
        new_settings = self.world.get_settings()

        if simulation_config['sync_mode']:
            new_settings.synchronous_mode = True
            new_settings.fixed_delta_seconds = \
                simulation_config['fixed_delta_seconds']
        else:
            sys.exit(
                'ERROR: Current version only supports sync simulation mode')

        self.world.apply_settings(new_settings)

        # set weather
        weather = self.set_weather(simulation_config['weather'])
        self.world.set_weather(weather)

        # Define probabilities for each type of blueprint
        self.use_multi_class_bp = scenario_params["blueprint"][
            'use_multi_class_bp'] if 'blueprint' in scenario_params else False
        if self.use_multi_class_bp:
            # bbx/blueprint meta
            with open(scenario_params['blueprint']['bp_meta_path']) as f:
                self.bp_meta = json.load(f)
            self.bp_class_sample_prob = scenario_params['blueprint'][
                'bp_class_sample_prob']

            # normalize probability
            self.bp_class_sample_prob = {
                k: v / sum(self.bp_class_sample_prob.values()) for k, v in
                self.bp_class_sample_prob.items()}

        self.cav_world = cav_world
        self.carla_map = self.world.get_map()
        self.apply_ml = apply_ml

        # eCLOUD BEGIN

        self.verbose_updates = self.ecloud_config.do_verbose_update()

        if 'ecloud' in scenario_params['scenario'] and 'num_cars' in scenario_params['scenario']['ecloud']:
            assert 'edge_list' not in scenario_params['scenario'], logger.exception("edge requires explicit")
            self.vehicle_count = scenario_params['scenario']['ecloud']['num_cars']
            logger.debug("'ecloud' in YAML specified %s cars", self.vehicle_count)

        elif 'edge_list' in scenario_params['scenario']:
            # TODO: support multiple edges...
            self.is_edge = True
            self.vehicle_count = len(scenario_params['scenario']['edge_list'][0]['vehicles'])
            self.rsu_count = len(scenario_params['scenario']['edge_list'][0]['rsus'])

        elif 'single_cav_list' in scenario_params['scenario']:
            self.vehicle_count = len(scenario_params['scenario']['single_cav_list'])

        else:
            assert False, logger.exception("no known vehicle indexing format found")

        if self.run_distributed:
            self.apply_ml = False
            if apply_ml == True:
                assert False, logger.exception("ML should only be run on the distributed clients")

            channel = grpc.aio.insecure_channel(
            target=f"{ECLOUD_IP}:50051",
            options=[
                ("grpc.lb_policy_name", "pick_first"),
                ("grpc.enable_retries", 1),
                ("grpc.keepalive_timeout_ms", TIMEOUT_MS),
                ("grpc.service_config", EcloudClient.retry_opts)],
            )
            self.ecloud_server = ecloud_rpc.EcloudStub(channel)

            self.debug_helper.update_sim_start_timestamp(time.time())

            logger.info(type(scenario_params))

            self.scenario = json.dumps(OmegaConf.to_container(scenario_params))
            self.carla_version = self.carla_version

        # eCLOUD END

        else: # sequential
            self.debug_helper.update_sim_start_timestamp(time.time())

    async def run_comms(self):
        self.push_q = asyncio.Queue()
        self.push_server = asyncio.create_task(ecloud_run_push_server(ECLOUD_PUSH_API_PORT, self.push_q))
        #self.push_server = threading.Thread(target=ecloud_run_push_server, args=(ECLOUD_PUSH_API_PORT, self.push_q,))
        #self.push_server.start()

        try:
          await asyncio.sleep(1) # this yields CPU to allow the PushServer to start
          logger.info("Push Server Started")
          server_request = ecloud.SimulationInfo() 
          server_request.test_scenario = self.scenario
          server_request.application = self.application[0]
          server_request.version = self.carla_version
          server_request.vehicle_index = self.vehicle_count # bit of a hack to use vindex as count here
          server_request.is_edge = self.is_edge or self.verbose_updates

          logger.info("Waiting for scenario start")
          await self.server_start_scenario(self.ecloud_server, server_request)
          logger.info("Start scenario started")

          self.world.tick()
        except Exception as e:
          logger.exception('unhandled exception')

        logger.debug("eCloud debug: pushed START")

    @staticmethod
    def set_weather(weather_settings):
        """
        Set CARLA weather params.

        Parameters
        ----------
        weather_settings : dict
            The dictionary that contains all parameters of weather.

        Returns
        -------
        The CARLA weather setting.
        """
        weather = carla.WeatherParameters(
            sun_altitude_angle=weather_settings['sun_altitude_angle'],
            cloudiness=weather_settings['cloudiness'],
            precipitation=weather_settings['precipitation'],
            precipitation_deposits=weather_settings['precipitation_deposits'],
            wind_intensity=weather_settings['wind_intensity'],
            fog_density=weather_settings['fog_density'],
            fog_distance=weather_settings['fog_distance'],
            fog_falloff=weather_settings['fog_falloff'],
            wetness=weather_settings['wetness']
        )
        return weather

    # BEGIN Core OpenCDA
    def create_vehicle_manager(self, application,
                               map_helper=None,
                               data_dump=False):
        """
        Create a list of single CAVs.
        Parameters
        ----------
        application : list
            The application purpose, a list, eg. ['single'], ['platoon'].
        map_helper : function
            A function to help spawn vehicle on a specific position in
            a specific map.
        data_dump : bool
            Whether to dump sensor data.
        Returns
        -------
        single_cav_list : list
            A list contains all single CAVs' vehicle manager.
        """
        logger.info('Creating single CAVs non dist.')
        single_cav_list = []
        #for vehicle_index in range(self.vehicle_count):
  
        default_model = 'vehicle.lincoln.mkz_2017'

        
        cav_vehicle_bp = \
            self.world.get_blueprint_library().find(default_model)
        for vehicle_index in range(self.vehicle_count):
        #for i, cav_config in enumerate(
                #self.scenario_params['scenario']['single_cav_list']):
            # in case the cav wants to join a platoon later
            # it will be empty dictionary for single cav application
            #platoon_base = OmegaConf.create({'platoon': self.scenario_params.get('platoon_base',{})})
            #cav_config = OmegaConf.merge(self.scenario_params['vehicle_base'],
            #                             platoon_base,
            #                             cav_config)
            # if the spawn position is a single scalar, we need to use map
            # helper to transfer to spawn transform
            #if 'spawn_special' not in cav_config:
            #    spawn_transform = carla.Transform(
            #        carla.Location(
            #            x=cav_config['spawn_position'][0],
            #            y=cav_config['spawn_position'][1],
            #            z=cav_config['spawn_position'][2]),
            #        carla.Rotation(
            #            pitch=cav_config['spawn_position'][5],
            #            yaw=cav_config['spawn_position'][4],
            #            roll=cav_config['spawn_position'][3]))
            #else:
            #    spawn_transform = map_helper(self.carla_version,
            #                                *cav_config['spawn_special'])

            cav_vehicle_bp.set_attribute('color', '0, 0, 255')
            cav_vehicle_bp.set_attribute('role_name', 'hero')
            #vehicle = self.world.spawn_actor(cav_vehicle_bp, spawn_transform)

            # create vehicle manager for each cav
            vehicle_manager = VehicleManager(
                vehicle_index=vehicle_index, carla_world=self.world,
                config_yaml=self.scenario_params, application=application,
                carla_map=self.carla_map, cav_world=self.cav_world,
                current_time=self.scenario_params['current_time'],
                data_dumping=data_dump, map_helper=map_helper,
                location_type=self.ecloud_config.get_location_type(),
                perception_active=self.apply_ml)

            self.world.tick()

            vehicle_manager.v2x_manager.set_platoon(None)
            self.vehicle_managers[vehicle_index] = vehicle_manager

            vehicle_manager.update_info()
            vehicle_manager.set_destination(
                vehicle_manager.vehicle.get_location(),
                vehicle_manager.destination_location,
                clean=True)

            single_cav_list.append(vehicle_manager)

        return single_cav_list

    def create_platoon_manager(self, map_helper=None, data_dump=False):
        """
        Create a list of platoons.

        Parameters
        ----------
        map_helper : function
            A function to help spawn vehicle on a specific position in a
            specific map.

        data_dump : bool
            Whether to dump sensor data.

        Returns
        -------
        single_cav_list : list
            A list contains all single CAVs' vehicle manager.
        """
        logger.info('Creating platoons/')
        platoon_list = []
        self.cav_world = CavWorld(self.apply_ml)

        # we use lincoln as default choice since our UCLA mobility lab use the
        # same car
        default_model = 'vehicle.lincoln.mkz_2017'

        cav_vehicle_bp = \
            self.world.get_blueprint_library().find(default_model)

        # create platoons
        for i, platoon in enumerate(
                self.scenario_params['scenario']['platoon_list']):
            platoon = OmegaConf.merge(self.scenario_params['platoon_base'],
                                      platoon)
            platoon_manager = PlatooningManager(platoon, self.cav_world)
            for j, cav in enumerate(platoon['members']):
                platton_base = OmegaConf.create({'platoon': platoon})
                cav = OmegaConf.merge(self.scenario_params['vehicle_base'],
                                      platton_base,
                                      cav
                                      )
                if 'spawn_special' not in cav:
                    spawn_transform = carla.Transform(
                        carla.Location(
                            x=cav['spawn_position'][0],
                            y=cav['spawn_position'][1],
                            z=cav['spawn_position'][2]),
                        carla.Rotation(
                            pitch=cav['spawn_position'][5],
                            yaw=cav['spawn_position'][4],
                            roll=cav['spawn_position'][3]))
                else:
                    spawn_transform = map_helper(self.carla_version,
                                                 *cav['spawn_special'])

                cav_vehicle_bp.set_attribute('color', '0, 0, 255')
                vehicle = self.world.spawn_actor(cav_vehicle_bp,
                                                 spawn_transform)

                # create vehicle manager for each cav
                vehicle_manager = VehicleManager(
                    vehicle, cav, ['platooning'],
                    self.carla_map, self.cav_world,
                    current_time=self.scenario_params['current_time'],
                    data_dumping=data_dump,
                    perception_active=self.apply_ml)

                # add the vehicle manager to platoon
                if j == 0:
                    platoon_manager.set_lead(vehicle_manager)
                else:
                    platoon_manager.add_member(vehicle_manager, leader=False)

            self.world.tick()
            destination = carla.Location(x=platoon['destination'][0],
                                         y=platoon['destination'][1],
                                         z=platoon['destination'][2])

            platoon_manager.set_destination(destination)
            platoon_manager.update_member_order()
            platoon_list.append(platoon_manager)

        return platoon_list

    def create_rsu_manager(self, data_dump):
        """
        Create a list of RSU.

        Parameters
        ----------
        data_dump : bool
            Whether to dump sensor data.

        Returns
        -------
        rsu_list : list
            A list contains all rsu managers..
        """
        print('Creating RSU.')
        rsu_list = []
        for i, rsu_config in enumerate(
                self.scenario_params['scenario']['rsu_list']):
            rsu_config = OmegaConf.merge(self.scenario_params['rsu_base'],
                                         rsu_config)
            rsu_manager = RSUManager(self.world, rsu_config,
                                     self.carla_map,
                                     self.cav_world,
                                     self.scenario_params['current_time'],
                                     data_dump)

            rsu_list.append(rsu_manager)

        return rsu_list

    def spawn_vehicles_by_list(self, tm, traffic_config, bg_list):
        """
        Spawn the traffic vehicles by the given list.

        Parameters
        ----------
        tm : carla.TrafficManager
            Traffic manager.

        traffic_config : dict
            Background traffic configuration.

        bg_list : list
            The list contains all background traffic.

        Returns
        -------
        bg_list : list
            Update traffic list.
        """

        blueprint_library = self.world.get_blueprint_library()
        if not self.use_multi_class_bp:
            ego_vehicle_random_list = car_blueprint_filter(blueprint_library,
                                                           self.carla_version)
        else:
            label_list = list(self.bp_class_sample_prob.keys())
            prob = [self.bp_class_sample_prob[itm] for itm in label_list]

        # if not random select, we always choose lincoln.mkz with green color
        color = '0, 255, 0'
        default_model = 'vehicle.lincoln.mkz_2017'
        ego_vehicle_bp = blueprint_library.find(default_model)

        for i, vehicle_config in enumerate(traffic_config['vehicle_list']):
            spawn_transform = carla.Transform(
                carla.Location(
                    x=vehicle_config['spawn_position'][0],
                    y=vehicle_config['spawn_position'][1],
                    z=vehicle_config['spawn_position'][2]),
                carla.Rotation(
                    pitch=vehicle_config['spawn_position'][5],
                    yaw=vehicle_config['spawn_position'][4],
                    roll=vehicle_config['spawn_position'][3]))

            if not traffic_config['random']:
                ego_vehicle_bp.set_attribute('color', color)

            else:
                # sample a bp from various classes
                if self.use_multi_class_bp:
                    label = np.random.choice(label_list, p=prob)
                    # Given the label (class), find all associated blueprints in CARLA
                    ego_vehicle_random_list = multi_class_vehicle_blueprint_filter(
                        label, blueprint_library, self.bp_meta)
                ego_vehicle_bp = random.choice(ego_vehicle_random_list)

                if ego_vehicle_bp.has_attribute("color"):
                    color = random.choice(
                        ego_vehicle_bp.get_attribute(
                            'color').recommended_values)
                    ego_vehicle_bp.set_attribute('color', color)

            vehicle = self.world.spawn_actor(ego_vehicle_bp, spawn_transform)
            vehicle.set_autopilot(True, 8000)

            if 'vehicle_speed_perc' in vehicle_config:
                tm.vehicle_percentage_speed_difference(
                    vehicle, vehicle_config['vehicle_speed_perc'])
            tm.auto_lane_change(vehicle, traffic_config['auto_lane_change'])

            bg_list.append(vehicle)

        return bg_list

    def spawn_vehicle_by_range(self, tm, traffic_config, bg_list):
        """
        Spawn the traffic vehicles by the given range.

        Parameters
        ----------
        tm : carla.TrafficManager
            Traffic manager.

        traffic_config : dict
            Background traffic configuration.

        bg_list : list
            The list contains all background traffic.

        Returns
        -------
        bg_list : list
            Update traffic list.
        """
        blueprint_library = self.world.get_blueprint_library()
        if not self.use_multi_class_bp:
            ego_vehicle_random_list = car_blueprint_filter(blueprint_library,
                                                           self.carla_version)
        else:
            label_list = list(self.bp_class_sample_prob.keys())
            prob = [self.bp_class_sample_prob[itm] for itm in label_list]

        # if not random select, we always choose lincoln.mkz with green color
        color = '0, 255, 0'
        default_model = 'vehicle.lincoln.mkz_2017'
        ego_vehicle_bp = blueprint_library.find(default_model)

        spawn_ranges = traffic_config['range']
        spawn_set = set()
        spawn_num = 0

        for spawn_range in spawn_ranges:
            spawn_num += spawn_range[6]
            x_min, x_max, y_min, y_max = \
                math.floor(spawn_range[0]), math.ceil(spawn_range[1]), \
                math.floor(spawn_range[2]), math.ceil(spawn_range[3])

            for x in range(x_min, x_max, int(spawn_range[4])):
                for y in range(y_min, y_max, int(spawn_range[5])):
                    location = carla.Location(x=x, y=y, z=0.3)
                    way_point = self.carla_map.get_waypoint(location).transform

                    spawn_set.add((way_point.location.x,
                                   way_point.location.y,
                                   way_point.location.z,
                                   way_point.rotation.roll,
                                   way_point.rotation.yaw,
                                   way_point.rotation.pitch))
        count = 0
        spawn_list = list(spawn_set)
        shuffle(spawn_list)

        while count < spawn_num:
            if len(spawn_list) == 0:
                break

            coordinates = spawn_list[0]
            spawn_list.pop(0)

            spawn_transform = carla.Transform(carla.Location(x=coordinates[0],
                                                             y=coordinates[1],
                                                             z=coordinates[
                                                                   2] + 0.3),
                                              carla.Rotation(
                                                  roll=coordinates[3],
                                                  yaw=coordinates[4],
                                                  pitch=coordinates[5]))
            if not traffic_config['random']:
                ego_vehicle_bp.set_attribute('color', color)

            else:
                # sample a bp from various classes
                if self.use_multi_class_bp:
                    label = np.random.choice(label_list, p=prob)
                    # Given the label (class), find all associated blueprints in CARLA
                    ego_vehicle_random_list = multi_class_vehicle_blueprint_filter(
                        label, blueprint_library, self.bp_meta)
                ego_vehicle_bp = random.choice(ego_vehicle_random_list)
                if ego_vehicle_bp.has_attribute("color"):
                    color = random.choice(
                        ego_vehicle_bp.get_attribute(
                            'color').recommended_values)
                    ego_vehicle_bp.set_attribute('color', color)

            vehicle = \
                self.world.try_spawn_actor(ego_vehicle_bp, spawn_transform)

            if not vehicle:
                continue

            vehicle.set_autopilot(True, 8000)
            tm.auto_lane_change(vehicle, traffic_config['auto_lane_change'])

						# hazard behavior for traffic
            tm.ignore_lights_percentage(vehicle, traffic_config[
                                                'ignore_lights_percentage'])
            #tm.ignore_signs_percentage(vehicle, traffic_config[
            #                                    'ignore_signs_percentage'])
            #tm.ignore_vehicles_percentage(vehicle, traffic_config[
            #                                    'ignore_vehicles_percentage'])
            #tm.ignore_walkers_percentage(vehicle, traffic_config[
            #                                    'ignore_walkers_percentage'])

            # each vehicle have slight different speed
            tm.vehicle_percentage_speed_difference(
                vehicle,
                traffic_config['global_speed_perc'] + random.randint(-30, 30))

            bg_list.append(vehicle)
            count += 1

        return bg_list

    def create_traffic_carla(self):
        """
        Create traffic flow.

        Returns
        -------
        tm : carla.traffic_manager
            Carla traffic manager.

        bg_list : list
            The list that contains all the background traffic vehicles.
        """
        logger.info('Spawning CARLA traffic flow.')
        traffic_config = self.scenario_params['carla_traffic_manager']
        tm = self.client.get_trafficmanager()

        tm.set_global_distance_to_leading_vehicle(
            traffic_config['global_distance'])
        tm.set_synchronous_mode(traffic_config['sync_mode'])
        tm.set_osm_mode(traffic_config['set_osm_mode'])
        tm.global_percentage_speed_difference(
            traffic_config['global_speed_perc'])

        bg_list = []

        if isinstance(traffic_config['vehicle_list'], list) or \
                isinstance(traffic_config['vehicle_list'], ListConfig):
            bg_list = self.spawn_vehicles_by_list(tm,
                                                  traffic_config,
                                                  bg_list)

        else:
            bg_list = self.spawn_vehicle_by_range(tm, traffic_config, bg_list)

        logger.info('CARLA traffic flow generated.')
        return tm, bg_list

    def close(self, spectator=None):
        """
        Simulation close.
        """
        # restore to origin setting
        if self.run_distributed:
            if spectator != None:
                logger.info("destroying specator CAV")
                try:
                    spectator.destroy()
                except:
                    logger.error("failed to destroy single CAV")

            subprocess.Popen(['pkill','-9','CarlaUE4'])
            sys.exit(0)

        self.world.apply_settings(self.origin_settings)
        logger.debug("world state restored...")

    # END Core OpenCDA

    # -------------------------------------------------------

    # BEGIN eCloud
    def create_distributed_vehicle_manager(self, application,
                               map_helper=None,
                               data_dump=False):
        """
        Create a list of single CAVs.

        Parameters
        ----------
        application : list
            The application purpose, a list, eg. ['single'], ['platoon'].

        map_helper : function
            A function to help spawn vehicle on a specific position in
            a specific map.

        data_dump : bool
            Whether to dump sensor data.

        Returns
        -------
        single_cav_list : list
            A list contains all single CAVs' vehicle manager.
        """
        logger.info('Creating single CAVs dist.')
        single_cav_list = []

        config_yaml = self.scenario_params
        logger.info(json.dumps(OmegaConf.to_container(config_yaml, resolve=True)))
        logger.info(self.vehicle_count)
        logger.info(application)
        for vehicle_index in range(self.vehicle_count):
            try:
              logger.debug("Creating VehiceManagerProxy for vehicle %s", vehicle_index)

              # create vehicle manager for each cav
              vehicle_manager_proxy = VehicleManagerProxy(
                  vehicle_index, config_yaml, application,
                  self.carla_map, self.cav_world,
                  current_time=self.scenario_params['current_time'],
                  data_dumping=data_dump, carla_version=self.carla_version, location_type=self.ecloud_config.get_location_type())
              logger.debug("finished creating VehiceManagerProxy")

              # self.tick_world()

              # send gRPC with START info
              self.application = application

              vehicle_manager_proxy.start_vehicle()

              vehicle_manager_proxy.v2x_manager.set_platoon(None)
              logger.debug("set platoon on vehicle manager")

              single_cav_list.append(vehicle_manager_proxy)
              self.vehicle_managers[vehicle_index] = vehicle_manager_proxy
            except Exception as e:
              logger.exception("Failed to create vehicle manager proxy")

        self.tick_world()
        logger.info("Finished creating vehicle managers and returning cav list")
        return single_cav_list

    def create_edge_manager(self, application,
                            map_helper=None,
                            data_dump=False,
                            world_dt=0.03,
                            edge_dt=0.20,
                            search_dt=2.00):
        """
        Create a list of edges.

        Parameters
        ----------
        map_helper : function
            A function to help spawn vehicle on a specific position in a
            specific map.
        Returns
        -------
        single_cav_list : list
            A list contains all single CAVs' vehicle manager.
        """

        # TODO: needs to support multiple edges.
        # Probably a more significant refactor,
        # since I think each edge wants its own gRPC server

        logger.info('Creating edge CAVs.')
        edge_list = []

        config_yaml = self.scenario_params
        logger.info(json.dumps(OmegaConf.to_container(config_yaml, resolve=True)))
        # create edges
        for e, edge in enumerate(
                self.scenario_params['scenario']['edge_list']):
            edge_manager = EdgeManager(edge, self.cav_world, carla_client=self.client, world_dt=world_dt, edge_dt=edge_dt, search_dt=search_dt, mode=config_yaml['edge_base']['mode'])
            for index, cav in enumerate(edge['rsus']):
                rsu_manager = RSUManager(self.world, cav,
                                   self.carla_map,
                                   self.cav_world,
                                   self.scenario_params['current_time'],
                                   data_dump)
                edge_manager.add_rsu(rsu_manager)
                self.rsu_managers[index] = rsu_manager
            for index, cav in enumerate(edge['vehicles']): 
                logger.debug("Creating VehiceManagerProxy for vehicle %s", index)
                # create vehicle manager for each cav
                #vehicle_manager = VehicleManagerProxy(
                #      vehicle_index=index, config_yaml=config_yaml, application=application,
                #      carla_world=self.world,
                #      carla_map=self.carla_map, cav_world=self.cav_world,
                #      current_time=self.scenario_params['current_time'],
                #      data_dumping=data_dump, carla_version=self.carla_version)
                vehicle_manager = VehicleManager(
                      vehicle_index=index, config_yaml=config_yaml, application=application,
                      carla_world=self.world,
                      carla_map=self.carla_map, cav_world=self.cav_world,
                      current_time=self.scenario_params['current_time'],
                      data_dumping=data_dump, is_edge=True, map_helper=map_helper,
                      location_type = self.ecloud_config.get_location_type(),
                      perception_active=self.apply_ml)

                logger.debug("finished creating VehiceManagerProxy")

                self.world.tick()

                # send gRPC with START info
                self.application = application

                #vehicle_manager.start_vehicle()
                vehicle_manager.v2x_manager.set_platoon(None)

                # add the vehicle manager to platoon
                edge_manager.add_member(vehicle_manager)
                self.vehicle_managers[index] = vehicle_manager

                vehicle_manager.update_info()
                vehicle_manager.set_destination(
                  vehicle_manager.vehicle.get_location(),
                  vehicle_manager.destination_location,
                  clean=True)


            try:
              self.tick_world()
              logger.debug("World ticked")
              destination = carla.Location(x=edge['destination'][0],
                                           y=edge['destination'][1],
                                           z=edge['destination'][2])

              edge_manager.set_destination(destination)
              logger.debug("Set Destination")
              edge_manager.start_edge()
              logger.debug("Started edge")
              edge_list.append(edge_manager)
            except Exception as e:
              logger.debug("Can't create edge manager: ", e)

        return edge_list

    def tick_world(self):
        """
        Tick the server; just a pass-through to broadcast_tick to preserve backwards compatibility for now...
        """
        pre_world_tick_time = time.time()
        self.world.tick()
        post_world_tick_time = time.time()
        logger.info("World tick completion time: %s", (post_world_tick_time - pre_world_tick_time))
        self.debug_helper.update_world_tick((post_world_tick_time - pre_world_tick_time)*1000)

    def tick(self):
        """
        Tick the server; just a pass-through to broadcast_tick to preserve backwards compatibility for now...
        """
        self.tick_world()

    # just use tick logic here; need something smarter if we want per-vehicle data
    # could also just switch to a "broadcast message "
    def broadcast_message(self, command = ecloud.Command.TICK):
        """
        Request all clients send debug data - broadcasts a message to all vehicles

        just using the tick_id; as noted, we should change to message ID to make this more generic

        returns bool
        """
        pre_client_tick_time = time.time()
        self.tick_id = self.tick_id + 1

        if command == ecloud.Command.REQUEST_DEBUG_INFO:
            self.vehicle_state = ecloud.VehicleState.DEBUG_INFO_UPDATE

        tick = ecloud.Tick()
        tick.tick_id = self.tick_id
        tick.command = command

        logger.debug("Getting timestamp")
        self.sm_start_tstamp.GetCurrentTime()
        logger.debug("Added Timestamp")

        asyncio.get_event_loop().run_until_complete(self.server_do_tick(self.ecloud_server, tick))

        post_client_tick_time = time.time()
        logger.info("Client tick completion time: %s", (post_client_tick_time - pre_client_tick_time))
        if self.tick_id > 1: # discard the first tick as startup is a major outlier
            self.debug_helper.update_client_tick((post_client_tick_time - pre_client_tick_time)*1000)

        return True

    def broadcast_tick(self):
        """
        Tick the server - broadcasts a message to all vehicles

        returns bool
        """
        self.vehicle_state = ecloud.VehicleState.TICK_OK
        return self.broadcast_message(ecloud.Command.TICK)


    def push_waypoint_buffer(self, waypoint_buffer): #, vehicle_index=None, vid=None, actor_id=None):
        """
        adds a waypoint buffer for a specific vehicle to the current tick message

        currently assumes the scenario has constructed a WaypointBuffer protobuf with explicit vehicle_index UID

        returns bool
        """
        edge_wp = ecloud.EdgeWaypoints()
        for wpb_proto in waypoint_buffer:
            #logger.debug(waypoint_buffer_proto.SerializeToString())
            edge_wp.all_waypoint_buffers.extend([wpb_proto])

        asyncio.get_event_loop().run_until_complete(self.server_push_waypoints(self.ecloud_server, edge_wp))

        return True

    def end(self):
        """
        broadcast end to all vehicles
        """
        start_time = time.time()
        self.vehicle_state = ecloud.VehicleState.TICK_DONE
        asyncio.get_event_loop().run_until_complete(self.server_end_scenario(self.ecloud_server))

        logger.info("pushed END")

        if self.run_distributed and ( ECLOUD_IP == 'localhost' or ECLOUD_IP == CARLA_IP ):
            os.kill(self.ecloud_server_process.pid, signal.SIGTERM)

        self.debug_helper.shutdown_time_ms = time.time() - start_time

    def do_pickling(self, column_key, flat_list, file_path):
        logger.info("run stats for %s:\nmean %s: %s \nmedian %s: %s \n95th percentile %s %s",
                    column_key, column_key, np.mean(flat_list),
                    column_key, np.median(flat_list),
                    column_key, np.percentile(flat_list, 95))

        data_df = pd.DataFrame(flat_list, columns = [f'{column_key}_ms'])
        data_df['num_cars'] = self.vehicle_count
        data_df['run_timestamp'] = pd.Timestamp.today().strftime('%Y-%m-%d %X')
        data_df = data_df[['num_cars', f'{column_key}_ms', 'run_timestamp']]

        data_df_path = f'./{file_path}/df_{column_key}'
        try:
            picklefile = open(data_df_path, 'rb+')
            current_data_df = pickle.load(picklefile)  #unpickle the dataframe
        except:
            picklefile = open(data_df_path, 'wb+')
            current_data_df = pd.DataFrame(columns=['num_cars', f'{column_key}_ms', 'run_timestamp'])

        picklefile = open(data_df_path, 'wb+')
        data_df = pd.concat([current_data_df, data_df], axis=0, ignore_index=True)

        # pickle the dataFrame
        pickle.dump(data_df, picklefile)
        logger.debug(data_df)
        #close file
        picklefile.close()

    def evaluate_agent_data(self, cumulative_stats_folder_path):
        if self.run_distributed is False:
            return

        PLANER_AGENT_STEPS = 12
        all_agent_data_lists = [[] for _ in range(PLANER_AGENT_STEPS)]
        for _, vehicle_manager_proxy in self.vehicle_managers.items():
            agent_data_list = vehicle_manager_proxy.agent.debug_helper.get_agent_step_list()
            for idx, sub_list in enumerate(agent_data_list):
                all_agent_data_lists[idx].append(sub_list)

        #logger.debug(all_agent_data_lists)

        for idx, all_agent_sub_list in enumerate(all_agent_data_lists):
            all_client_data_list_flat = np.array(all_agent_sub_list)
            if all_client_data_list_flat.any():
                all_client_data_list_flat = np.hstack(all_client_data_list_flat)
            else:
                all_client_data_list_flat = all_client_data_list_flat.flatten()
            data_key = f"agent_step_list_{idx}"
            self.do_pickling(data_key, all_client_data_list_flat, cumulative_stats_folder_path)

    def evaluate_network_data(self, cumulative_stats_folder_path):
        if self.run_distributed is False:
            return

        all_network_data_list = sum(self.debug_helper.network_time_dict_per_client.values(), [])

        all_network_data_list_flat = np.array(all_network_data_list)
        if all_network_data_list_flat.any():
            all_network_data_list_flat = np.hstack(all_network_data_list_flat)
        else:
            all_network_data_list_flat = all_network_data_list_flat.flatten()

        data_key = f"network_latency"
        self.do_pickling(data_key, all_network_data_list_flat, cumulative_stats_folder_path)

    def evaluate_idle_data(self, cumulative_stats_folder_path):
        if self.run_distributed is False:
            return
        
        all_idle_data_lists = sum(self.debug_helper.idle_time_dict.values(), [])

        all_idle_data_lists_flat = np.array(all_idle_data_lists)
        if all_idle_data_lists_flat.any():
            all_idle_data_lists_flat = np.hstack(all_idle_data_lists_flat)
        else:
            all_idle_data_lists_flat = all_idle_data_lists_flat.flatten()
        data_key = f"idle"
        self.do_pickling(data_key, all_idle_data_lists_flat, cumulative_stats_folder_path)

    def evaluate_client_process_data(self, cumulative_stats_folder_path):
        all_client_process_data_lists = sum(self.debug_helper.client_process_time_dict.values(), [])

        all_client_process_data_list_flat = np.array(all_client_process_data_lists)
        if all_client_process_data_list_flat.any():
            all_client_process_data_list_flat = np.hstack(all_client_process_data_list_flat)
        else:
            all_client_process_data_list_flat = all_client_process_data_list_flat.flatten()
        data_key = f"client_process"
        self.do_pickling(data_key, all_client_process_data_list_flat, cumulative_stats_folder_path)

        data_key = f"client_individual_process_times_dict"

        data_df = pd.DataFrame.from_dict(ScenarioManager.debug_helper.client_process_time_dict)
        data_df['num_cars'] = self.vehicle_count
        data_df['run_timestamp'] = pd.Timestamp.today().strftime('%Y-%m-%d %X')

        data_df_path = f'./{cumulative_stats_folder_path}/df_{data_key}'
        picklefile = open(data_df_path, 'wb')

        # pickle the dataFrame
        pickle.dump(data_df, picklefile)
        logger.debug(data_df)
        #close file
        picklefile.close()

    def evaluate_individual_client_data(self, cumulative_stats_folder_path):
        all_client_data_lists = sum(self.debug_helper.client_tick_time_dict_per_client.values(), [])

        all_client_data_list_flat = np.array(all_client_data_lists)
        if all_client_data_list_flat.any():
            all_client_data_list_flat = np.hstack(all_client_data_list_flat)
        else:
            all_client_data_list_flat = all_client_data_list_flat.flatten()

        data_key = f"client_individual_step_time"
        self.do_pickling(data_key, all_client_data_list_flat, cumulative_stats_folder_path)

    def evaluate_client_data(self, client_data_key, cumulative_stats_folder_path):
        all_client_data_list = []
        for _, vehicle_manager_proxy in self.vehicle_managers.items():
            client_data_list = vehicle_manager_proxy.debug_helper.get_debug_data()[client_data_key]
            all_client_data_list.append(client_data_list)

        logger.debug(all_client_data_list)

        
        #logger.debug(all_client_data_list)

        all_client_data_list_flat = np.array(all_client_data_list)
        if all_client_data_list_flat.any():
            all_client_data_list_flat = np.hstack(all_client_data_list_flat)
        else:
            all_client_data_list_flat = all_client_data_list_flat.flatten()

        if(len(all_client_data_list_flat) == 0):
          return
        self.do_pickling(client_data_key, all_client_data_list_flat, cumulative_stats_folder_path)

    def evaluate_collision_data(self, cumulative_stats_folder_path):
        all_client_data_list = []
        for _, vehicle_manager_proxy in self.vehicle_managers.items():
            client_data_list = vehicle_manager_proxy.debug_helper.get_debug_data()["client_collisons_list"]
            for collision_event in client_data_list:
              all_client_data_list.append()

        logger.debug(all_client_data_list)

        
        #logger.debug(all_client_data_list)

        all_client_data_list_flat = np.array(all_client_data_list)
        if all_client_data_list_flat.any():
            all_client_data_list_flat = np.hstack(all_client_data_list_flat)
        else:
            all_client_data_list_flat = all_client_data_list_flat.flatten()

        if(len(all_client_data_list_flat) == 0):
          return
        self.do_pickling(client_data_key, all_client_data_list_flat, cumulative_stats_folder_path)



    def evaluate(self, excludes_list = ["client_collisions_list", "client_lane_invasions_list"]):
        """
        Used to save all members' statistics.

        Returns
        -------
        figure : matplotlib.figure
            The figure drawing performance curve passed back to save to
            the disk.

        perform_txt : str
            The string that contains all evaluation results to print out.
        """

        perform_txt = ''

        num_clients = len(VEHICLE_IP.split(","))
        if(self.run_distributed):
            cumulative_stats_folder_path = f'./evaluation_outputs/cumulative_stats_dist_{num_clients}_no_perception'
            if self.perception:
                cumulative_stats_folder_path = f'./evaluation_outputs/cumulative_stats_dist_{num_clients}_with_perception'
        else:
            cumulative_stats_folder_path = f'./evaluation_outputs/cumulative_stats_seq_no_perception'
            if self.perception:
                cumulative_stats_folder_path = f'./evaluation_outputs/cumulative_stats_seq_with_perception'

        if not os.path.exists(cumulative_stats_folder_path):
            os.makedirs(cumulative_stats_folder_path)

        if(self.run_distributed):
            #self.evaluate_velocity_data(cumulative_stats_folder_path)
            self.evaluate_agent_data(cumulative_stats_folder_path)
            self.evaluate_network_data(cumulative_stats_folder_path)
            self.evaluate_idle_data(cumulative_stats_folder_path)
            self.evaluate_client_process_data(cumulative_stats_folder_path)
            self.evaluate_individual_client_data(cumulative_stats_folder_path)

        client_helper = ClientDebugHelper(0)
        debug_data_lists = client_helper.get_debug_data().keys()
         
        for list_name in debug_data_lists:
                if excludes_list is not None and list_name in excludes_list:
                    continue
                
                self.evaluate_client_data(list_name, cumulative_stats_folder_path)
                logger.debug(list_name)

        #self.evaluate_collision_data(cumulative_stats_folder_path)

        # ___________Client Step time__________________________________
        client_tick_time_list = self.debug_helper.client_tick_time_list
        client_tick_time_list_flat = np.concatenate(client_tick_time_list)
        if client_tick_time_list_flat.any():
            client_tick_time_list_flat = np.hstack(client_tick_time_list_flat)
        else:
            client_tick_time_list_flat = client_tick_time_list_flat.flatten()
        client_step_time_key = 'client_step_time'
        self.do_pickling(client_step_time_key, client_tick_time_list_flat, cumulative_stats_folder_path)

        # ___________World Step time_________________________________
        world_tick_time_list = self.debug_helper.world_tick_time_list
        world_tick_time_list_flat = np.concatenate(world_tick_time_list)
        if world_tick_time_list_flat.any():
            world_tick_time_list_flat = np.hstack(world_tick_time_list_flat)
        else:
            world_tick_time_list_flat = world_tick_time_list_flat.flatten()
        world_step_time_key = 'world_step_time'
        self.do_pickling(world_step_time_key, world_tick_time_list_flat, cumulative_stats_folder_path)

        # ___________Total simulation time ___________________
        sim_start_time = self.debug_helper.sim_start_timestamp
        sim_end_time = time.time()
        total_sim_time = (sim_end_time - sim_start_time) # total time in seconds
        perform_txt += f"Total Simulation Time: {total_sim_time} \n\t Registration Time: {self.debug_helper.startup_time_ms}ms \n\t Shutdown Time: {self.debug_helper.shutdown_time_ms}ms"

        sim_time_df_path = f'./{cumulative_stats_folder_path}/df_total_sim_time'
        try:
            picklefile = open(sim_time_df_path, 'rb+')
            sim_time_df = pickle.load(picklefile)  #unpickle the dataframe
        except:
            picklefile = open(sim_time_df_path, 'wb+')
            sim_time_df = pd.DataFrame(columns=['num_cars', 'time_s', 'startup_time_ms', 'shutdown_time_ms', 'run_timestamp'])

        picklefile = open(sim_time_df_path, 'wb+')
        sim_time_df = pd.concat([sim_time_df, pd.DataFrame.from_records \
            ([{"num_cars": self.vehicle_count, \
                "time_s": total_sim_time, \
                "startup_time_ms": self.debug_helper.startup_time_ms, \
                "shutdown_time_ms": self.debug_helper.shutdown_time_ms, \
                "run_timestamp": pd.Timestamp.today().strftime('%Y-%m-%d %X') }])], \
                ignore_index=True)

        # pickle the dataFrame
        pickle.dump(sim_time_df, picklefile)
        logger.debug(sim_time_df)
        #close file
        picklefile.close()

        # plotting
        figure = plt.figure()

        plt.subplot(411)
        open_plt.draw_world_tick_time_profile_single_plot(world_tick_time_list)

        

        # plt.subplot(412)
        # open_plt.draw_algorithm_time_profile_single_plot(algorithm_time_list)

        return figure, perform_txt

    def evaluate_velocity_error(self):
        with open('velocity_list', 'w+') as f:
            f.write(f'{self.debug_helper.client_velocity_dict}')

        plt.subplot(413)
        open_plt.draw_deviation_from_target_velocity(self.debug_helper.client_velocity_dict)
        all_vels = sum(self.debug_helper.client_velocity_dict.values(), [])
        all_vels_flat = np.array(all_vels)
        vel_mean = np.mean(all_vels_flat.flatten())
        logger.info("Mean Velocity: %s", vel_mean)

    # END eCloud
