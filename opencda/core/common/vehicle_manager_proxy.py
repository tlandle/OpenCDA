# -*- coding: utf-8 -*-
"""
Basic class of CAV
"""
# Author: Tyler Landle <tlandle3@gatech.edu>, Jordan Rapp <jrapp7@gatech.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib

import uuid
import subprocess
import random
import socket
import json
import psutil

import carla
import numpy as np

from opencda.core.actuation.control_manager \
    import ControlManager
from opencda.core.application.platooning.platoon_behavior_agent\
    import PlatooningBehaviorAgent
from opencda.core.common.v2x_manager \
    import V2XManager
from opencda.core.sensing.localization.localization_manager \
    import LocalizationManager
from opencda.core.sensing.perception.perception_manager \
    import PerceptionManager
from opencda.core.plan.behavior_agent \
    import BehaviorAgent
from opencda.core.common.data_dumper import DataDumper
from opencda.scenario_testing.utils.yaml_utils import load_yaml
from opencda.client_debug_helper import ClientDebugHelper
from opencda.core.common.ecloud_config import eLocationType

RESULT_SUCCESS = 0 # Step ran ok
RESULT_ERROR = 1 # Step resulted in an error
RESULT_END = 2 # Step resulted in the vehicle simulation ending

cloud_config = load_yaml("cloud_config.yaml")
CARLA_IP = cloud_config["carla_server_public_ip"]

# once we have all methods, we no longer need any reference to the actual actor in ecloud
class ActorProxy(object):
    def __init__(self,
                 id = 0):
        self.id = id
        self.transform = carla.Transform(
                carla.Location(
                    x=0,
                    y=0,
                    z=0),
                carla.Rotation(
                    yaw=0,
                    roll=0,
                    pitch=0))
        self.velocity = carla.Vector3D(x=0, y=0, z=0)

    def is_proxy(self):
        return True
    
    def get_transform(self):
        return self.transform
    
    def set_transform(self, transform):
        self.transform = transform

    def get_location(self):
        # need to return Carla.location
        return self.transform.location
    
    def set_location(self, location):
        self.transform.location = location

    def get_velocity(self):
        # need to return carla.Vector3D
        return self.velocity

    def set_velocity(self, velocity):
        self.velocity = velocity

class VehicleManagerProxy(object):
    """
    TODO: update
    """

    def __init__(
            self,
            vehicle_index,
            carla_world,
            config_yaml,
            application,
            carla_map,
            cav_world,
            carla_version,
            current_time='',
            data_dumping=False,
            location_type=eLocationType.EXPLICIT):

        self.is_edge = False
        if 'edge_list' in config_yaml['scenario']:
            # TODO: support multiple edges...
            self.is_edge = True
            self.cav_config = config_yaml['scenario']['edge_list'][0]['vehicles'][vehicle_index]
        
        elif 'single_cav_list' in config_yaml['scenario']:
                self.cav_config = config_yaml['scenario']['single_cav_list'][vehicle_index] if location_type == eLocationType.EXPLICIT \
                                else config_yaml['scenario']['single_cav_list'][0]
        
        else:
            assert(False, "no known vehicle indexing format found")
        
        self.carla_world = carla_world
        self.cav_world = cav_world
        self.data_dumping = data_dumping
        self.application = application
        self.current_time = current_time
        self.vehicle_index = vehicle_index
        self.vid = vehicle_index # needed for compatibility

        self.carla_map = carla_map

        # Use sockets for interprocess communication between OpenCDA and each vehicle
        #self._socket = conn
        self.debug_helper = ClientDebugHelper(0)

    def start_vehicle(self):
        #print("eCloud debug | actor_id: " + str(actor_id))
        self.vehicle = ActorProxy(self.vehicle_index)

        # retrieve the configure for different modules
        sensing_config = self.cav_config['sensing']
        behavior_config = self.cav_config['behavior']
        control_config = self.cav_config['controller']
        v2x_config = self.cav_config['v2x']
        # v2x module
        self.v2x_manager = V2XManager(self.cav_world, v2x_config, self.vehicle_index)
        print("V2X Manager Created")
        # localization module
        self.localizer = LocalizationManager(
            self.vehicle, sensing_config['localization'], self.carla_map)
        # perception module - proxy should never use perception
        sensing_config['perception']['activate'] = False
        sensing_config['perception']['camera_visualize'] = False
        sensing_config['perception']['lidar_visualize'] = False
        self.perception_manager = PerceptionManager(
            self.vehicle, sensing_config['perception'], carla_world=self.carla_world, cav_world=self.cav_world,
            data_dump=self.data_dumping)
        print("Perception Manager created")

        # behavior agent
        self.agent = None
        if 'platooning' in self.application:
            platoon_config = self.cav_config['platoon']
            self.agent = PlatooningBehaviorAgent(
                self.vehicle, # TODO: fix
                self,
                self.v2x_manager,
                behavior_config,
                platoon_config,
                self.carla_map)
        else:
            self.agent = BehaviorAgent(self.vehicle, self.carla_map, behavior_config)
            print("Behavior Agent Created")

        # Control module
        self.controller = ControlManager(control_config)

        if self.data_dumping:
            self.data_dumper = DataDumper(self.perception_manager,
                                          self.vehicle.id,
                                          save_time=self.current_time)
        else:
            self.data_dumper = None

        print("Created Proxy")

        self.cav_world.update_vehicle_manager(self)
