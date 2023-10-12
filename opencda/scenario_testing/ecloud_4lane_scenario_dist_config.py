# -*- coding: utf-8 -*-
"""
eCloudSim
---------
Scenario testing: *TEMPLATE* 
Use for OpenCDA vs eCloudSim DIST-ONLY comparisons

*NOT for Edge*

Town 06 Scenarios *ONLY*

DO NOT USE for 2-Lane Free
"""
# Author: Jordan Rapp, Dean Blank, Tyler Landle <Georgia Tech>
# License: TDG-Attribution-NonCommercial-NoDistrib

# Core
import os
import time
import asyncio

# 3rd Party
import carla

# OpenCDA Utils
import opencda.scenario_testing.utils.sim_api as sim_api
from opencda.scenario_testing.utils.yaml_utils import load_yaml
from opencda.core.common.cav_world import CavWorld
from opencda.scenario_testing.evaluations.evaluate_manager import \
    EvaluationManager
# ONLY *required* for 2 Lane highway scenarios
# import opencda.scenario_testing.utils.customized_map_api as map_api

import ecloud_pb2 as ecloud

# Consts
LOG_NAME = "ecloud_4lane.log" # data drive from file name?
SCENARIO_NAME = "ecloud_4lane_scenario" # data drive from file name?
TOWN = 'Town06'
STEP_COUNT = 300

def run_scenario(opt, config_yaml):
    step = 0
    try:
        scenario_params = load_yaml(config_yaml)

        # sanity checks...
        assert('edge_list' not in scenario_params['scenario']) # do NOT use this template for edge scenarios
        assert('sync_mode' in scenario_params['world'] and scenario_params['world']['sync_mode'] == True)
        assert(scenario_params['world']['fixed_delta_seconds'] == 0.03 or scenario_params['world']['fixed_delta_seconds'] == 0.05)
        
        # spectator configs
        world_x = scenario_params['world']['x_pos'] if 'x_pos' in scenario_params['world'] else 0 
        world_y = scenario_params['world']['y_pos'] if 'y_pos' in scenario_params['world'] else 0
        world_z = scenario_params['world']['z_pos'] if 'z_pos' in scenario_params['world'] else 256
        world_roll = scenario_params['world']['roll'] if 'roll' in scenario_params['world'] else 0
        world_pitch = scenario_params['world']['pitch'] if 'pitch' in scenario_params['world'] else -90
        world_yaw = scenario_params['world']['yaw'] if 'yaw' in scenario_params['world'] else 0

        run_distributed = scenario_params['distributed'] if 'distributed' in scenario_params else \
                          True if 'ecloud' in scenario_params else \
                          False

        cav_world = CavWorld(opt.apply_ml)
        # create scenario manager
        scenario_manager = sim_api.ScenarioManager(scenario_params,
                                                   opt.apply_ml,
                                                   opt.version,
                                                   town=TOWN,
                                                   cav_world=cav_world,
                                                   config_file=config_yaml,
                                                   distributed=run_distributed)

        if opt.record:
            scenario_manager.client. \
                start_recorder(LOG_NAME, True)

        
        # create single cavs        
        if run_distributed:
            asyncio.get_event_loop().run_until_complete(scenario_manager.run_comms())
            single_cav_list = \
                scenario_manager.create_distributed_vehicle_manager(application=['single']) 
        else:    
            single_cav_list = \
                scenario_manager.create_vehicle_manager(application=['single'])

        # create background traffic in carla
        traffic_manager, bg_veh_list = \
            scenario_manager.create_traffic_carla()

        # create evaluation manager
        eval_manager = \
            EvaluationManager(scenario_manager.cav_world,
                              script_name=SCENARIO_NAME,
                              current_time=scenario_params['current_time'])

        spectator = scenario_manager.world.get_spectator()
 
        flag = True
        while flag:
            print("Step: %d" %step)
            scenario_manager.tick_world()
            if run_distributed:
                flag = scenario_manager.broadcast_tick()
            
            else:    
                # non-dist will break automatically; don't need to set flag
                pre_client_tick_time = time.time()
                for i, single_cav in enumerate(single_cav_list):
                    single_cav.update_info()
                    control = single_cav.run_step()
                    single_cav.vehicle.apply_control(control)
                post_client_tick_time = time.time()
                print("Client tick completion time: %s" %(post_client_tick_time - pre_client_tick_time))
                if step > 0: # discard the first tick as startup is a major outlier
                    scenario_manager.debug_helper.update_client_tick((post_client_tick_time - pre_client_tick_time)*1000)

            # same for dist / non-dist - only required for specate
            transform = single_cav_list[0].vehicle.get_transform()
            spectator.set_transform(carla.Transform(
                transform.location +
                carla.Location(
                    x=world_x,
                    y=world_y,
                    z=world_z),
                carla.Rotation(
                    yaw=world_yaw,
                    roll=world_roll,
                    pitch=world_pitch)))   

            step = step + 1
            if step > STEP_COUNT:
                if run_distributed:
                    flag = scenario_manager.broadcast_message(ecloud.Command.REQUEST_DEBUG_INFO)
                break             

    finally:
        if run_distributed:
            scenario_manager.end() # only dist requires explicit scenario end call

        if step >= STEP_COUNT:
            eval_manager.evaluate()

        if opt.record:
            scenario_manager.client.stop_recorder()

        scenario_manager.close()
  
        if not run_distributed:
            for v in single_cav_list:
                v.destroy()

        for v in bg_veh_list:
            print("destroying background vehicle")
            try:
                v.destroy()
            except:
                print("failed to destroy background vehicle")  
