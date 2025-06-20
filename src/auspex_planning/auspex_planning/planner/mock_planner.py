#!/usr/bin/env python3
from fractions import Fraction
import copy
import os
from auspex_msgs.msg import ActionInstance, Plan, ActionStatus
from upf_msgs.msg import (
    Atom,
    Real
)
import json
from .converter import enum_to_str
from .planner_base import PlannerBase

class Mock_Planner(PlannerBase):
    planner_key = 'mock_planner'
    """
    Ros2 node used to create a mock plan

    """
    def __init__(self, kb_client):
        """
        Constructor method
        """
        self._kb_client = kb_client
        params_dir =  os.getenv('AUSPEX_PARAMS_PATH')
        self._auspex_params_path = os.path.join(params_dir, 'mission/')
        print("Initialized mock planner...")
        pass

    def feedback(self, team_id, feedback_msg):
        pass

    def result(self, team_id, result_msg):
        pass

    def plan_rth(self, team_id):
        rth_template = self.load_mission_from_json(jsonpath=self._auspex_params_path+'return_to_home_and_land.json')[0]
        action_list = []
        if team_id.lower() != "all":
            vhcl_dict = self._kb_client.query('platform', 'platform_id', 'team_id', team_id)
        elif(team_id.lower() == "all"):
            vhcl_dict = self._kb_client.query('platform', 'platform_id')

        for vhcl in vhcl_dict:
            rth_mission_platform = copy.deepcopy(rth_template)
            rth_mission_platform.platform_id = vhcl['platform_id']
            rth_mission_platform.team_id = team_id
            rth_mission_platform.priority = 10
            for action in rth_mission_platform.tasks:
                action.parameters[0].symbol_atom = [vhcl['platform_id']]
            action_list.append(rth_mission_platform)
        return action_list


    def plan_mission(self, team_id):
        """
        Creates the ActionInstance List for the executer to execute
        """
        print(f"[INFO]: Mock Planner Selected. Loading Mission from JSON.")
        return self.load_mission_from_json(jsonpath=self._auspex_params_path+'mock_mission.json')

    def load_mission_from_json(self,jsonpath):
        """
        Reads a JSON file and converts it into the up_msg format (list of Plan objects).
        """
        with open(jsonpath, 'r') as file:
            mission_data = json.load(file)

        platform_plans = []

        for platform in mission_data:
            planned_tasks = []
            for id_a, action in enumerate(platform["actions"]):
                new_action = ActionInstance()
                new_action.action_name = action["action_name"]
                new_action.id = id_a
                new_action.status = enum_to_str(ActionStatus, ActionStatus.INACTIVE)

                new_parameters = []
                atom = Atom()
                atom.symbol_atom = [platform["platform_id"]]
                new_parameters.append(atom)

                for param in action["parameters"]:
                    atom = Atom()
                    if isinstance(param, str):
                        atom.symbol_atom = [param]
                    elif isinstance(param, float):
                        fraction_representation = Fraction.from_float(param)
                        real_msg = Real()
                        real_msg.numerator = fraction_representation.numerator
                        real_msg.denominator = fraction_representation.denominator
                        atom.real_atom = [real_msg]
                    elif isinstance(param, int):
                        fraction_representation = Fraction(param)
                        real_msg = Real()
                        real_msg.numerator = fraction_representation.numerator
                        real_msg.denominator = fraction_representation.denominator
                        atom.real_atom = [real_msg]
                    elif isinstance(param, bool):
                        atom.bool_atom = [param]
                    else:
                        atom.symbol_atom = [str(param)]

                    new_parameters.append(atom)

                new_action.parameters = new_parameters
                planned_tasks.append(new_action)

            plan_msg = Plan()
            plan_msg.tasks = planned_tasks
            plan_msg.priority = 0
            plan_msg.platform_id = platform["platform_id"]
            plan_msg.team_id = platform["team_id"]
            platform_plans.append(plan_msg)

        return platform_plans
