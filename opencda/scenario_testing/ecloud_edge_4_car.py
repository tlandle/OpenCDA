# -*- coding: utf-8 -*-
"""
Scenario testing: merging vehicle joining a platoon in the customized 2-lane
freeway sorely with carla Warning: You have to load the 2lanefreecomplete
map into your ue4 editor before running this
"""
# Author: Tyler Landle <tlandle3@gatech.edu>
#       : Jordan Rapp <jrapp7@gatech.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib
# Core
import asyncio

# 3rd Party
import carla

# OpenCDA Utils
import opencda.scenario_testing.utils.sim_api as sim_api
from opencda.scenario_testing.utils.yaml_utils import load_yaml
from opencda.core.common.cav_world import CavWorld
from opencda.scenario_testing.evaluations.evaluate_manager import \
    EvaluationManager
from opencda.scenario_testing.utils.yaml_utils import add_current_time
# ONLY *required* for 2 Lane highway scenarios
# import opencda.scenario_testing.utils.customized_map_api as map_api

import ecloud_pb2 as ecloud

# Consts
LOG_NAME = "ecloud_4lane_edge.log" # data drive from file name?
SCENARIO_NAME = "ecloud_4lane_edge_scenario" # data drive from file name?
TOWN = 'Town06'
STEP_COUNT = 600

def run_scenario(opt, scenario_params):
    step = 0
    try:
        scenario_params = add_current_time(scenario_params)

        cav_world = CavWorld(opt.apply_ml)
        # create scenario manager
        scenario_manager = sim_api.ScenarioManager(scenario_params,
                                                   opt.apply_ml,
                                                   opt.version,
                                                   town=TOWN,
                                                   cav_world=cav_world,
                                                   distributed=True)

        if opt.record:
            scenario_manager.client. \
                start_recorder(LOG_NAME, True)

        world_dt = scenario_params['world']['fixed_delta_seconds']
        edge_dt = scenario_params['edge_base']['edge_dt']
        assert( edge_dt % world_dt == 0 ) # we need edge time to be an exact multiple of world time because we send waypoints every Nth tick

        # create single cavs
        asyncio.get_event_loop().run_until_complete(scenario_manager.run_comms())
        edge_list = \
            scenario_manager.create_edge_manager(application=['edge'], edge_dt=edge_dt, world_dt=world_dt)

        # create background traffic in carla
        #traffic_manager, bg_veh_list = \
            #scenario_manager.create_traffic_carla()

        eval_manager = \
            EvaluationManager(scenario_manager.cav_world,
                              script_name='ecloud_edge_scenario',
                              current_time=scenario_params['current_time'])

        spectator = scenario_manager.world.get_spectator()
        spectator_vehicle = edge_list[0].vehicle_manager_list[1].vehicle

        # run steps
        step = 0
        flag = True
        waypoint_buffer = []
        world_time = 0
        while flag:
            print("Step: %d" %step)
            scenario_manager.tick_world()

            world_time += world_dt

            if world_time % edge_dt == 0:
                world_time = 0

                waypoint_buffer.clear()
                for edge in edge_list:
                    edge.update_information()
                    waypoint_buffer = edge.run_step()
                    #print(waypoint_buffer)

                scenario_manager.push_waypoint_buffer(waypoint_buffer)
                flag = scenario_manager.broadcast_message(ecloud.Command.PULL_WAYPOINTS_AND_TICK)
            else:
               flag = scenario_manager.broadcast_tick()
            
            step = step + 1
            if step > STEP_COUNT:
                flag = scenario_manager.broadcast_message(ecloud.Command.REQUEST_DEBUG_INFO)
                break             

            transform = spectator_vehicle.get_transform()
            spectator.set_transform(
                carla.Transform(
                    transform.location +
                    carla.Location(
                        z=120),
                    carla.Rotation(
                        pitch=-
                        90)))

    finally:
        scenario_manager.end() # only dist requires explicit scenario end call

        if step >= STEP_COUNT:
            eval_manager.evaluate()

        if opt.record:
            scenario_manager.client.stop_recorder()       

        scenario_manager.close()
        for edge in edge_list:
            edge.destroy()
