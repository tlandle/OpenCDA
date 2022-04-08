# -*- coding: utf-8 -*-
"""
Basic class of CAV
"""
# Author: Runsheng Xu <rxx3386@ucla.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib

import random
import uuid

import carla
import numpy as np

from opencda.core.common.cav_world import CavWorld
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
from opencda.core.map.map_manager import MapManager
from opencda.core.common.data_dumper import DataDumper
from opencda.scenario_testing.utils.yaml_utils import load_yaml



class VehicleManager(object):
    """
    A class manager to embed different modules with vehicle together.

    Parameters
    ----------
    vehicle : carla.Vehicle
        The carla.Vehicle. We need this class to spawn our gnss and imu sensor.

    config_yaml : dict
        The configuration dictionary of this CAV.

    application : list
        The application category, currently support:['single','platoon'].

    carla_map : carla.Map
        The CARLA simulation map.

    cav_world : opencda object
        CAV World. This is used for V2X communication simulation.

    current_time : str
        Timestamp of the simulation beginning, used for data dumping.

    data_dumping : bool
        Indicates whether to dump sensor data during simulation.

    Attributes
    ----------
    v2x_manager : opencda object
        The current V2X manager.

    localizer : opencda object
        The current localization manager.

    perception_manager : opencda object
        The current V2X perception manager.

    agent : opencda object
        The current carla agent that handles the basic behavior
         planning of ego vehicle.

    controller : opencda object
        The current control manager.

    data_dumper : opencda object
        Used for dumping sensor data.
    """

    def __init__(
            self,
            vehicle_index,
            config_file,
            application,
            cav_world,
            carla_version,
            current_time='',
            data_dumping=False):

        # an unique uuid for this vehicle
        self.vid = str(uuid.uuid1())

        self.scenario_params = load_yaml(config_file)

        self.initialize_process()
        self.carla_version = carla_version

        # By default, we use lincoln as our cav model.
        default_model = 'vehicle.lincoln.mkz2017' \
            if self.carla_version == '0.9.11' else 'vehicle.lincoln.mkz_2017'

        cav_vehicle_bp = \
            self.world.get_blueprint_library().find(default_model)

        # if the spawn position is a single scalar, we need to use map
        # helper to transfer to spawn transform
        cav_config = self.scenario_params['scenario']['single_cav_list'][vehicle_index]
        if 'spawn_special' not in cav_config:
            spawn_transform = carla.Transform(
                carla.Location(
                    x=cav_config['spawn_position'][0],
                    y=cav_config['spawn_position'][1],
                    z=cav_config['spawn_position'][2]),
                carla.Rotation(
                    pitch=cav_config['spawn_position'][5],
                    yaw=cav_config['spawn_position'][4],
                    roll=cav_config['spawn_position'][3]))
# TODO eCloud Need to put this back in. Is not being used for simple scenario I'm working with currently
#        else:
#            spawn_transform = map_helper(self.carla_version,
#                                         *cav_config['spawn_special'])

        cav_vehicle_bp.set_attribute('color', '0, 0, 255')
        self.vehicle = self.world.spawn_actor(cav_vehicle_bp, spawn_transform)

        # retrieve the configure for different modules
        sensing_config = config_yaml['sensing']
        map_config = config_yaml['map_manager']
        behavior_config = config_yaml['behavior']
        control_config = config_yaml['controller']
        v2x_config = config_yaml['v2x']

        # v2x module
        self.v2x_manager = V2XManager(cav_world, v2x_config, self.vid)
        # localization module
        self.localizer = LocalizationManager(
            self.vehicle, sensing_config['localization'], self.carla_map)
        # perception module
        self.perception_manager = PerceptionManager(
            self.vehicle, sensing_config['perception'], cav_world,
            data_dumping)
        # map manager
        self.map_manager = MapManager(vehicle,
                                      carla_map,
                                      map_config)

        # behavior agent
        self.agent = None
        if 'platooning' in application:
            platoon_config = cav_config['platoon']
            self.agent = PlatooningBehaviorAgent(
                self.vehicle,
                self,
                self.v2x_manager,
                behavior_config,
                platoon_config,
                self.carla_map)
        else:
            self.agent = BehaviorAgent(self.vehicle, self.carla_map, behavior_config)

        # Control module
        self.controller = ControlManager(control_config)

        if data_dumping:
            self.data_dumper = DataDumper(self.perception_manager,
                                          self.vehicle.id,
                                          save_time=current_time)
        else:
            self.data_dumper = None

        cav_world.update_vehicle_manager(self)

    def initialize_process(self):
        simulation_config = self.scenario_params['world']

        # set random seed if stated
        if 'seed' in simulation_config:
            np.random.seed(simulation_config['seed'])
            random.seed(simulation_config['seed'])

        self.client = \
            carla.Client('localhost', simulation_config['client_port'])
        self.client.set_timeout(10.0)
        self.world = self.client.get_world()
        self.carla_map = self.world.get_map()

    def set_destination(
            self,
            start_location,
            end_location,
            clean=False,
            end_reset=True):
        """
        Set global route.

        Parameters
        ----------
        start_location : carla.location
            The CAV start location.

        end_location : carla.location
            The CAV destination.

        clean : bool
             Indicator of whether clean waypoint queue.

        end_reset : bool
            Indicator of whether reset the end location.

        Returns
        -------
        """

        self.agent.set_destination(
            start_location, end_location, clean, end_reset)

    def update_info(self):
        """
        Call perception and localization module to
        retrieve surrounding info an ego position.
        """
        # localization
        self.localizer.localize()

        ego_pos = self.localizer.get_ego_pos()
        ego_spd = self.localizer.get_ego_spd()

        # object detection
        objects = self.perception_manager.detect(ego_pos)

        # update the ego pose for map manager
        self.map_manager.update_information(ego_pos)

        # update ego position and speed to v2x manager,
        # and then v2x manager will search the nearby cavs
        self.v2x_manager.update_info(ego_pos, ego_spd)

        self.agent.update_information(ego_pos, ego_spd, objects)
        # pass position and speed info to controller
        self.controller.update_info(ego_pos, ego_spd)

    def run_step(self, target_speed=None):
        """
        Execute one step of navigation.
        """
        # visualize the bev map if needed
        self.map_manager.run_step()
        target_speed, target_pos = self.agent.run_step(target_speed)
        if target_speed == -1:
            print("run_step: simulation is over")
            return None # -1 indicates the simulation is over. TODO Need a const here.
        control = self.controller.run_step(target_speed, target_pos)

        # dump data
        if self.data_dumper:
            self.data_dumper.run_step(self.perception_manager,
                                      self.localizer,
                                      self.agent)

        return control

    def apply_control(self, control):
        """
        Apply the controls to the vehicle
        """
        self.vehicle.apply_control(control)

    def destroy(self):
        """
        Destroy the actor vehicle
        """
        self.perception_manager.destroy()
        self.localizer.destroy()
        self.vehicle.destroy()
        self.map_manager.destroy()
