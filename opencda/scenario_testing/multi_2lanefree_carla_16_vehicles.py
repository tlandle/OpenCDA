# -*- coding: utf-8 -*-
"""
Scenario testing: Single vehicle dring in the customized 2 lane highway map.
"""
# Author: Runsheng Xu <rxx3386@ucla.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib

import os

import carla
from opencda.core.common.vehicle_manager_proxy import VehicleManagerProxy

import opencda.scenario_testing.utils.sim_api as sim_api
import opencda.scenario_testing.utils.customized_map_api as map_api

from opencda.core.common.cav_world import CavWorld
from opencda.scenario_testing.evaluations.evaluate_manager import \
    EvaluationManager
from opencda.scenario_testing.utils.yaml_utils import load_yaml

from timeit import default_timer as timer
import statistics

def run_scenario(opt, config_yaml):
    
    runtimes = []

    try:
        scenario_params = load_yaml(config_yaml)

        current_path = os.path.dirname(os.path.realpath(__file__))
        xodr_path = os.path.join(
            current_path,
            '../assets/2lane_freeway_simplified/2lane_freeway_simplified.xodr')

        # create CAV world
        cav_world = CavWorld(opt.apply_ml)
        # create scenario manager
        scenario_manager = sim_api.ScenarioManager(scenario_params,
                                                   opt.apply_ml,
                                                   opt.version,
                                                   xodr_path=xodr_path,
                                                   cav_world=cav_world,
                                                   config_file=config_yaml)

        if opt.record:
            scenario_manager.client. \
                start_recorder("single_2lanefree_carla.log", True)

        single_cav_list = \
            scenario_manager.create_vehicle_manager(application=['single'],
                                                    map_helper=map_api.
                                                    spawn_helper_2lanefree)

        # create background traffic in carla
        traffic_manager, bg_veh_list = \
            scenario_manager.create_traffic_carla()

        # create evaluation manager
        eval_manager = \
            EvaluationManager(scenario_manager.cav_world,
                              script_name='single_2lanefree_carla',
                              current_time=scenario_params['current_time'])

        spectator = scenario_manager.world.get_spectator()
        # run steps
       
        flag = True
        while flag:
            start = timer()
            scenario_manager.tick()
            # TODO eCloud - figure out another way to have the vehicle follow a CAV. Perhaps still access the bp since it's read only?
            transform = single_cav_list[0].vehicle.get_transform()
            spectator.set_transform(carla.Transform(
                transform.location +
                carla.Location(
                    z=120),
                carla.Rotation(
                    pitch=-
                    90)))

            for _, single_cav in enumerate(single_cav_list):
                result = single_cav.do_tick()
                if result == 1: # Need to figure out how to use a const
                    print("Unexpected termination: Sending END to all vehicles and ending.")
                    flag = False
                    break
                elif result == 2:
                    print("Simulation ended: Sending END to all vehicles and ending.")
                    flag = False
                    break
            end = timer()
            runtimes.append(end-start)
            print(end - start)
            #with open ("log-%s.txt"%(scenario_params['current_time']), "a") as f:
              #f.write("%s" %(end-start))


        for _, single_cav in enumerate(single_cav_list):
            single_cav.end_step()

    finally:
        print("Evaluating simulation results...")
        #eval_manager.evaluate()
        print(runtimes)
        print(scenario_params['current_time'])
        with open ("log-%s.txt"%(scenario_params['current_time']), "w") as f:
              f.write("%s"%runtimes)
        with open ("stats-%s.txt"%(scenario_params['current_time']), "w") as f:
              f.write("%s"%statistics.mean(runtimes))
              f.write("%s"%statistics.quantiles(runtimes))


        if opt.record:
            scenario_manager.client.stop_recorder()

        scenario_manager.close()

        for v in single_cav_list:
            v.destroy()
        for v in bg_veh_list:
            v.destroy()
