# -*- coding: utf-8 -*-
"""
Evaluation manager.
"""

# Author: Runsheng Xu <rxx3386@ucla.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib

import subprocess
import os
from datetime import datetime
from opencda.scenario_testing.evaluations.utils import lprint

class EvaluationManager(object):
    """
    Evaluation manager to manage the analysis of the
    results for different modules.

    Parameters
    ----------
    cav_world : opencda object
        The CavWorld object that contains all CAVs' information.

    script_name : str
        The current scenario testing name. E.g, single_town06_carla

    current_time : str
        Current timestamp, used to name the output folder.

    Attributes
    ----------
    eval_save_path : str
        The final output folder name.

    """

    def __init__(self, cav_world, script_name, current_time):
        self.cav_world = cav_world

        current_path = os.path.dirname(os.path.realpath(__file__))

        self.eval_save_path = os.path.join(
            current_path, '../../../evaluation_outputs',
            script_name + '_' + current_time)
        if not os.path.exists(self.eval_save_path):
            os.makedirs(self.eval_save_path)

        #print(f"self.eval_save_path: {self.eval_save_path}") 

    def evaluate(self):
        """
        Evaluate performance of all modules by plotting and writing the
        statistics into the log file.
        """
        log_file = os.path.join(self.eval_save_path, 'log.txt')

        self.localization_eval(log_file)
        print('Localization Evaluation Done.')

        self.kinematics_eval(log_file)
        print('Kinematics Evaluation Done.')

        self.platooning_eval(log_file)
        print('Platooning Evaluation Done.')

        self.edge_eval(log_file)
        print('Edge Evaluation Done.')

        self.simulation_eval(log_file)
        print('Simulation Evaluation Done.')

    def kinematics_eval(self, log_file):
        """
        vehicle kinematics related evaluation.

        Args:
            -log_file (File): The log file to write the data.

        """
        lprint(log_file, "***********Kinematics Module***********")
        for vid, vm in self.cav_world.get_vehicle_managers().items():
            actor_id = vm.vehicle.id
            lprint(log_file, 'Actor ID: %d' % actor_id)

            loc_debug_helper = vm.agent.debug_helper
            figure, perform_txt = loc_debug_helper.evaluate()

            # save plotting
            figure_save_path = os.path.join(
                self.eval_save_path,
                '%d_kinematics_plotting.eps' %
                actor_id)
            figure.savefig(figure_save_path, format='eps', dpi=1200)


            lprint(log_file, perform_txt)

    def localization_eval(self, log_file):
        """
        Localization module evaluation.

        Args:
            -log_file (File): The log file to write the data.
        """
        lprint(log_file, "***********Localization Module***********")
        for vid, vm in self.cav_world.get_vehicle_managers().items():
            actor_id = vm.vehicle.id
            #print("actor_id = vm.vehicle.id")
            lprint(log_file, 'Actor ID: %d' % actor_id)

            loc_debug_helper = vm.localizer.debug_helper
            #print("loc_debug_helper = vm.localizer.debug_helper")
            figure, perform_txt = loc_debug_helper.evaluate()
            #print("figure, perform_txt = loc_debug_helper.evaluate()")

            # save plotting
            figure_save_path = os.path.join(
                self.eval_save_path,
                '%d_localization_plotting.eps' %
                actor_id)
            figure.savefig(figure_save_path, format='eps', dpi=1200)


            # save log txt
            lprint(log_file, perform_txt)

    def platooning_eval(self, log_file):
        """
        Platooning evaluation.

        Args:
            -log_file (File): The log file to write the data.

        """
        lprint(log_file, "***********Platooning Analysis***********")

        for pmid, pm in self.cav_world.get_platoon_dict().items():
            lprint(log_file, 'Platoon ID: %s' % pmid)
            figure, perform_txt = pm.evaluate()

            # save plotting
            figure_save_path = os.path.join(
                self.eval_save_path,
                '%s_platoon_plotting.eps' %
                pmid)
            figure.savefig(figure_save_path, format='eps', dpi=1200)


            # save log txt
            lprint(log_file, perform_txt)

    def edge_eval(self, log_file):
        """
        Edge evaluation.

        Args:
            -log_file (File): The log file to write the data.

        """
        lprint(log_file, "***********Edge Analysis***********")

        for pmid, pm in self.cav_world.get_edge_dict().items():
            lprint(log_file, 'Edge ID: %s' % pmid)
            figure, perform_txt = pm.evaluate()

            # save plotting
            figure_save_path = os.path.join(
                self.eval_save_path,
                '%s_edge_plotting.eps' %
                pmid)
            figure.savefig(figure_save_path, format='eps', dpi=1200)


            # save log txt
            lprint(log_file, perform_txt)


    def simulation_eval(self, log_file):
        """
        Simulation evaluation.

        Args:
            -log_file (File): The log file to write the data.

        """
        lprint(log_file, "***********Simulation Analysis***********")

        scenario_manager =self.cav_world.get_scenario_manager()
        figure, perform_txt = scenario_manager.evaluate()

        # save plotting
        figure_save_path = os.path.join(
            self.eval_save_path,
            'simulation_plotting.eps')
        figure.savefig(figure_save_path, format='eps', dpi=1200)

        # save log txt
        lprint(log_file, perform_txt)
