#!/usr/bin/env python3
import ast
from .llm_planner_utils.model_say import SayModel
from .llm_planner_utils.model_can import CanModel
from .llm_planner_utils.model_pay import PayModel
from .llm_planner_utils.utils import *
from .llm_planner_utils.env_discription import EnvDiscription
from .planner_base import PlannerBase
from .converter import AUSPEXConverter
from auspex_msgs.msg import ExecutorState
import torch.distributed as dist
from enum import Enum
import numpy as np
import time
import copy

TAKE_OFF_ACTION = "Take off"
LAND_ACTION = "Land"

class LLM_Planner(PlannerBase):
    planner_key = 'llm_planner'
    """
    A Planner class implementing Say-ReapEx framework
    """

    def __init__(self, kb_client):
        """
        Initialize the planner with environment description, knowledge base,
        and high-level models for decision making.
        """
        self.initialized = False
        # Knowledge Base Client
        self._kb_client = kb_client
        # Environment Description
        self.environment_description = EnvDiscription()

        self._converter = AUSPEXConverter()

        # Drone State
        self.drone_state = None

        # Prompts and User Input
        self.main_prompt = ""
        self.goal_prompt = ""
        self.current_additional_prompt = ""
        self.current_user_input = ""

        # Search Parameters
        self.target_object = ""
        self.target_color = ""
        self.target_area = ""
        self.target_season = ""
        self.goal = ""
        self.additional_information = []

        # Execution State
        self.current_action = ""
        self.manual_action_queue = []
        self.manual_action_queue.append("Take off to 20 metres altitude.")
        self.manual_action_queue.append("Return to Home and Land.")

        self.final_action = "Return to Home and Land."
        self.max_allowed_actions = 15
        self.action_count = 0
        self.action_list_string = ""

        self.total_time_action_selection = 0.0

        # Load Waypoints and Lookup Table
        self._operation_areas = dict()

        areas_db = self._kb_client.query(collection='area', key='', value='')
        if len(areas_db) > 0:
            for area in areas_db:
                if not area:
                    continue
                self._operation_areas[area['name']] = area

        self.waypoints = json.dumps(self._operation_areas)

        # Models
        self.say_model = SayModel(provider="openai", model="gpt-4o-mini")
        self.can_model = CanModel()
        self.pay_model = PayModel(self, self._operation_areas)

        if self.pay_model != None and self.can_model != None:
            self.initialized = True
            # Update Environment Description with Waypoints
            self.environment_description.env_description += f"\n{self.waypoints}"

            # Preprocess Actions
            self.action_list_with_placeholders = splitActionStringToList(self.environment_description.action_string_with_description)
            self.action_list_without_params = self.environment_description.stripActionList(self.action_list_with_placeholders)
            self.action_list_parameterized = generate_actions(
                self.action_list_with_placeholders,
                self.environment_description.heights,
                self.environment_description.wps,
                self.environment_description.durations,
                self.environment_description.objects
            )

            # Setup POMDP in Pay Model
            self.pay_model.setup_pomdp(
                planner=self,
                states=self.environment_description.wps,
                actions=copy.deepcopy(self.action_list_parameterized)
            )

            print(f"Initialized with {len(self.action_list_parameterized)} actions.")

    def extract_goal_information(self, goals):
        """
        Extracts goal-related information from user input or default instruction.
        """
        default_goal = "SEARCH"
        detection_target = ["person"]
        location_target = ["openareas"]
        default_season = ["summer"]
        detection_color = ["NOT_SPECIFIED"]
        additional_info = "Backpack"

        formatted_goals = ""

        for goal in goals:
            goal_status = goal.get('status', '').upper()
            if goal_status != 'UNPLANNED':
                continue

            formatted_goal = ""
            if goal.get('type') == "AND" or goal.get('type') == "OR":
                continue

            self.goal = self.goal + f"{goal.get('type')},"

            if goal.get('type') == "SEARCH":
                if goal.get('parameters_json'):
                    parameters = goal.get('parameters_json')
                    location_target = parameters.get('locations', ["NOT_SPECIFIED"])
                    if location_target[0] == "NOT_SPECIFIED":
                        formatted_goal = (f"SEARCH the areas close to a point of interest. If you found nothing, return home and land. ")
                        continue
                formatted_goal = (f"SEARCH the areas specified by {','.join(location_target)}. If you found nothing, return home and land. ")

            elif goal.get('type') == "FIND":
                if goal.get('parameters_json'):
                    parameters = goal.get('parameters_json')
                    detection_target = parameters.get('objects', detection_target)
                    detection_color = parameters.get('colors', detection_color)
                    additional_info = parameters.get('additional_info', additional_info)

                formatted_goal = (f"FIND and Detect objects: {','.join(detection_target)}. "
                                    f"If you confirm one {','.join(detection_target)}, return home and land. "
                                    f"Additional information: {','.join(additional_info)}")
            elif goal.get('type') == "LAND":
                formatted_goal = (f"LAND after completing the mission. ")

            formatted_goals += f"{formatted_goal}\n"

        # Update search parameters
        self.target_object = ','.join(detection_target)
        self.target_color = ','.join(detection_color)
        self.target_area = ','.join(location_target)
        self.target_season = default_season[0]
        self.additional_information = ','.join(additional_info)
        self.current_user_input = goals[0].get('description', "Search the area for a person.")
        return formatted_goals

    def get_next_action(self,
            mode="Say'n'Fly",
            actions_with_params=[],
            actions_without_params=[],
            actions_with_placeholders=[],
            stitched_prompt=""
            ):

        n_actions_computed = 0

        """
        If predefined goals are given
        """
        start_time = time.time()
        if len(self.manual_action_queue) != 0:
            selected_action_string  = self.manual_action_queue.pop(0)
        else:
            '''
            Get LLM Scores
            '''
            detected_objects = self._kb_client.query(collection='object')
            if mode == "SayCan":
                '''
                Select action and params
                '''
                print(len(actions_with_params))
                platform_status = self.drone_state.get('platform_status', 'UNKNOWN')
                say_scores = self.say_model.compute_llm_score(stitched_prompt, actions_with_params, platform_status, synchronous_call=True)
                can_scores = self.can_model.compute_affordance_score(state, actions_with_params, self)

                combined_scores = say_scores * can_scores

                selected_action_string = actions_with_params[np.argmax(combined_scores)]

                display_scores(say_scores, actions_with_params)
                n_actions_computed = len(actions_with_params)

            elif mode == "SayCanPay":
                '''
                Select action and params
                '''
                platform_status = self.drone_state.get('platform_status', 'UNKNOWN')
                say_scores = self.say_model.compute_llm_score(stitched_prompt, actions_with_params, platform_status, synchronous_call=True)
                can_scores = self.can_model.compute_affordance_score(self.drone_state, detected_objects, actions_with_params, self)
                pay_scores = self.pay_model.compute_pay_score(self.drone_state, actions_with_params)

                combined_scores = say_scores * can_scores * pay_scores

                selected_action_string = actions_with_params[np.argmax(combined_scores)]

                display_scores(say_scores, actions_with_params)
                n_actions_computed = len(actions_with_params)

            elif mode == "Say'n'Fly":
                top_k = 10 #takes the top five for affordance

                '''
                Select action
                '''
                platform_status = self.drone_state.get('platform_status', 'UNKNOWN')
                say_scores = self.say_model.compute_llm_score(stitched_prompt, actions_with_placeholders, state=platform_status, synchronous_call=False)

                can_scores = self.can_model.compute_affordance_score(self.drone_state, detected_objects, actions_with_placeholders, self)

                combined_scores = say_scores * can_scores

                selected_action_string = actions_with_placeholders[np.argmax(combined_scores)]
                print("---------------------------------")

                print("SAY SCORES:")
                display_scores(say_scores, actions_with_placeholders)
                # time.sleep(10)
                print("\nCAN-PRE SCORES:")
                display_scores(can_scores, actions_with_placeholders)
                # time.sleep(10)
                print("\n")

                print(f'Selected action: {selected_action_string}\n' )
                print("Now selecting parameters...\n")
                print("---------------------------------")

                '''
                Select parameter for action
                '''
                parameterized_actions =  generate_actions(
                                                    [selected_action_string],
                                                    self.environment_description.heights,
                                                    self.environment_description.wps,
                                                    self.environment_description.durations,
                                                    self.environment_description.objects)

                can_scores = self.can_model.compute_affordance_score(self.drone_state, detected_objects, parameterized_actions, self)
                print("CAN-COMP SCORES:")
                display_scores(can_scores, parameterized_actions)
                # time.sleep(10)
                print("\n")
                index_max = 0
                if top_k > len(can_scores):
                    action_np = np.array(parameterized_actions)
                    filtered_actions = action_np
                    filtered_can_scores = can_scores
                else:
                    index_max = top_k
                    action_np = np.array(parameterized_actions)

                    paired_list = list(zip(can_scores, action_np))
                    sorted_list = sorted(paired_list, key=lambda x: x[0], reverse=True)
                    can_scores, action_np = zip(*sorted_list)
                    can_scores = np.array(list(can_scores))
                    action_np = np.array(list(action_np))

                    can_scores[index_max:] = [0.0] * (len(can_scores) - index_max) # setting everything to zero except top k

                    filtered_actions = action_np[can_scores > 0.0]
                    filtered_can_scores = can_scores[can_scores > 0.0]

                print(f"Shrinked action space from {len(actions_with_params)} to {len(filtered_actions)} actions.")
                say_scores = self.say_model.compute_llm_score(stitched_prompt, filtered_actions, state=platform_status, synchronous_call=False)
                pay_scores = self.pay_model.compute_pay_score(self.drone_state, filtered_actions)

                combined_scores = say_scores * filtered_can_scores * pay_scores

                selected_action_string = filtered_actions[np.argmax(combined_scores)]

                print("\nSAY SCORES:")
                display_scores(say_scores, filtered_actions)
                # time.sleep(10)
                print("\nPAY SCORES:")
                display_scores(pay_scores, filtered_actions)
                # time.sleep(10)
                print("---------------------------------")

                n_actions_computed = len(actions_without_params) + len(filtered_actions)

        end_time = time.time()
        self.total_time_action_selection = self.total_time_action_selection + (end_time - start_time)
        print(f'\nSelected final parameterized action: {selected_action_string}\n')
        return selected_action_string, n_actions_computed


    def plan_mission(self, team_id):
        """
        Orchestrates the planning and execution of the mission.
        """
        if not self.initialized:
            return []

        print(f"[INFO]: LLM Planner Selected for team : {team_id}")

        self.current_platforms = self._kb_client.query(collection='platform', field='platform_id', key='team_id', value=team_id)

        if self.current_platforms == []:
            print("No platforms found.")
            return []

        self.update_state(self.current_platforms[0]['platform_id'])

        goals = self._kb_client.query(collection='goal', field='', key='team_id', value=team_id)

        if goals == {}:
            print("No goals found.")
            return []
        constraint_ids = []
        for goal in goals:
            for constraint_id in goal.get('constraint_ids', []):
                if constraint_id not in constraint_ids:
                    constraint_ids.append(constraint_id)
        constraints = []
        for constraint_id in constraint_ids:
            constraint = self._kb_client.query(collection='constraint', field='', key='constraint_id', value=constraint_id)
            if constraint:
                constraints.append(constraint[0])

        if constraints:
            constraints_text = "\n".join([f"- {c.get('description', 'No description')}" for c in constraints])
            self.goal_prompt += f"\n**Constraints:**\n{constraints_text}\n"
        else:
            print(f"[INFO]: No constraints found.")

        goal_description = self.extract_goal_information(goals)
        print(f"Goal Description: {goal_description}")

        # Build the initial prompt
        self.main_prompt = self.environment_description.env_description
        self.goal_prompt = f"\n**Goal:** {goal_description}\n"

        # Model selection
        self.mode = "Say'n'Fly"

        # Generate Initial Instruction
        instruction_prompt = self.environment_description.instruction_prompt.format(
            action_params_description=self.environment_description.action_string_with_description,
            actions=self.action_list_string,
            goal_prompt=self.goal_prompt,
            search_object=self.target_object,
            search_area=self.target_area
        )
        instruction_prompt = instruction_prompt + self.current_additional_prompt
        stitched_prompt = self.main_prompt + instruction_prompt

        """
        Get Platform State from KB
        """
        # Select Next Action
        next_action, _ = self.get_next_action(
            mode=self.mode,
            actions_with_params=self.action_list_parameterized,
            actions_without_params=self.action_list_without_params,
            actions_with_placeholders=self.action_list_with_placeholders,
            stitched_prompt=stitched_prompt
        )

        print(f"Planned Next Action: {next_action}")

        if "continue" in next_action.lower():
            print("Continue action. No new action planned.")
            return ["Continue"]

        '''
        Execute next action?
        '''
        self.current_action = next_action
        self.action_count = self.action_count + 1

        action_msg = self.environment_description.NL2action_msg(next_action, self.action_count)

        """
        Query for platform in team.
        """
        planned_tasks = self._converter.convert_plan_llm2auspex(self.current_platforms[0]['platform_id'], team_id, [action_msg])

        plans = []
        plan_msg = Plan()
        plan_msg.tasks = planned_tasks
        plan_msg.platform_id = self.current_platforms[0]['platform_id']
        plan_msg.team_id = team_id
        plan_msg.priority = 0
        plans.append(plan_msg)

        if plans == None:
            plans = []
        return plans

    def update_state(self, platform_id):
        """
        Updates the drone state.
        """
        drone_state_tmp = self._kb_client.query(collection='platform', key='platform_id', value=platform_id)
        if drone_state_tmp == []:
            print("No drone state found.")
            return
        self.drone_state = drone_state_tmp[0]

    def feedback(self, team_id, feedback_message):
        """
        Process feedback from the drone.
        """
        pass

    def result(self, team_id, result_message):
        """
        Process the result of the executed action.
        """
        print("Finished action. Planning new if applicable...")
        '''
        Update the POMDP for the AEMS2 heuristic
        '''
        self.pay_model.update_pomdp(self.current_action, 1) # 1 means not detected and 0 mean detected

        '''
        Add action to prompt
        '''
        self.action_list_string +=  self.current_action + "\n"

        if len(result_message.info.status_flags) > 0:
            self.current_additional_prompt = f"Status Flags: {','.join(result_message.info.status_flags)}\n"
        else:
            self.current_additional_prompt = ""

        if self.current_action != self.final_action and self.action_count <= self.max_allowed_actions:
            return self.plan_mission(team_id)
        else:
            print("Final Action. Shutting down LLM Planner.")



    def on_destroy(self):
        """
        Cleanup operations when the planner is destroyed.
        """
        if dist.is_initialized():
            dist.destroy_process_group()



#      take_off(uav1)
# [planning_main_node-1]     fly(uav1, home, pois)
# [planning_main_node-1]     search_poi(uav1, pois)
# [planning_main_node-1]     fly(uav1, pois, manual_search_area)
# [planning_main_node-1]     search_area(uav1, manual_search_area)
# [planning_main_node-1]     fly(uav1, manual_search_area, home)
# [planning_main_node-1]     land(uav1)