#!/usr/bin/env python3
from .llm_planner_utils.model_say import SayModel
from .llm_planner_utils.model_can import CanModel
from .llm_planner_utils.model_pay import PayModel
from .llm_planner_utils.utils import *
from .llm_planner_utils.env_discription import EnvDiscription
from .planner_base import PlannerBase
import torch.distributed as dist
from enum import Enum
import numpy as np
import time
import copy

TAKE_OFF_ACTION = "Take off"
LAND_ACTION = "Land"

class LLM_Planner(PlannerBase):
    """
    A Planner class implementing Say-ReapEx framework
    """

    def __init__(self, kb_client):
        """
        Initialize the planner with environment description, knowledge base, 
        and high-level models for decision making.
        """
        # Environment Description
        self.environment_description = EnvDiscription()

        # Initialize Available Entities and Skills
        self.available_detections = self.environment_description.objects
        self.available_locations = self.environment_description.getUniqueLocations()
        self.available_high_level_skills = self.environment_description.high_level_skill
        self.available_detection_colors = self.environment_description.detection_color
        self.available_seasons = self.environment_description.seasons

        # Knowledge Base Client
        self._kb_client = kb_client

        # Drone State
        self.drone_state = DroneState()

        # Prompts and User Input
        self.main_prompt = ""
        self.goal_prompt = ""
        self.current_instruction_prompt = ""
        self.current_user_input = ""

        # Search Parameters
        self.target_object = ""
        self.target_color = ""
        self.target_area = ""
        self.target_season = ""
        self.high_level_skill = ""
        self.additional_information = []

        # Execution State
        self.is_executing = False
        self.detection_count = 0
        self.reaction_mode = "llm"
        self.current_action = ""
        self.manual_action_queue = []
        self.final_action = "Return to Home and Land."
        self.max_allowed_actions = 15
        self.action_count = 0
        self.action_list_string = ""

        # Simulation Connection State
        self.simulation_connected = True
        self.total_time_action_selection = 0.0

        # Load Waypoints and Lookup Table
        self.operation_areas = load_json('/home/companion/auspex_params/geographic/areas/location1_areas_shrinked.json')
        self.waypoints = load_file_content('/home/companion/auspex_params/geographic/areas/location1_areas_shrinked.json')

        # Models
        self.say_model = SayModel(use_llm="gpt")
        self.can_model = CanModel()
        self.pay_model = PayModel(self, self.operation_areas)

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

    def extract_goal_information(self, human_input=False):
        """
        Extracts goal-related information from user input or default instruction.
        """
        high_level_skill = ["Search"]
        detection_target = ["person"]
        location_target = ["openareas"]
        season = ["summer"]
        detection_color = ["NOT_SPECIFIED"]
        additional_info = "Backpack"

        if human_input:
            user_input = input("What should I do? ").strip().lower()
            (high_level_skill, detection_target, detection_color, 
             location_target, additional_info) = self.say_model.parseNL2Param(
                user_input,
                self.available_detections,
                self.available_detection_colors,
                self.available_locations,
                self.available_high_level_skills
            )
        else:
            user_input = "Search for a person in a blue jacket on an open field."

        self.current_user_input = user_input

        if "search" in high_level_skill[0].lower():
            if location_target[0] == "NOT_SPECIFIED":
                formatted_goal = (f"Search for a {detection_target[0]}. "
                                  f"If you confirm a {detection_target[0]}, return home and land. "
                                  f"Additional information: {additional_info}")
            else:
                formatted_goal = (f"Search the area {location_target[0]} for a {detection_target[0]}. "
                                  f"If confirmed, return home and land. Additional information: {additional_info}")
        else:
            print(f"The skill: {high_level_skill[0]} is not supported.")
            return None

        # Update search parameters
        self.high_level_skill = high_level_skill[0]
        self.target_object = detection_target[0]
        self.target_color = detection_color[0]
        self.target_area = location_target[0]
        self.target_season = season[0]
        self.additional_information = additional_info

        return formatted_goal

    def get_next_action(self, 
            mode="sayCan", 
            state=None, 
            use_llm_state=False, 
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
            selected_skill_string  = self.manual_action_queue.pop(0)     
        else:
            '''
            Get LLM Scores
            '''

            if mode == "SayCan":
                '''
                Select skill and params
                '''
                print(len(actions_with_params))
                say_scores = self.say_model.compute_llm_score(stitched_prompt, state, use_llm_state, actions_with_params, _synchronous_calls=True)
                can_scores = self.can_model.compute_affordance_score(state, actions_with_params, self)

                combined_scores = say_scores * can_scores

                selected_skill_string = actions_with_params[np.argmax(combined_scores)]

                display_scores(say_scores, actions_with_params)
                n_actions_computed = len(actions_with_params)

            elif mode == "SayCanPay":
                '''
                Select skill and params
                '''
                say_scores = self.say_model.compute_llm_score(stitched_prompt, state, use_llm_state, actions_with_params, _synchronous_calls=True)
                can_scores = self.can_model.compute_affordance_score(state, actions_with_params, self)
                pay_scores = self.pay_model.compute_pay_score(state, actions_with_params)

                combined_scores = say_scores * can_scores * pay_scores

                selected_skill_string = actions_with_params[np.argmax(combined_scores)]

                display_scores(say_scores, actions_with_params)
                n_actions_computed = len(actions_with_params) 

            elif mode == "SayCan-CanSayPay":
                top_k = 10 #takes the top five for affordance

                '''
                Select skill
                '''
                say_scores = self.say_model.compute_llm_score(stitched_prompt, state, use_llm_state, actions_without_params, _synchronous_calls=False)

                can_scores = self.can_model.compute_affordance_score(state, actions_with_placeholders, self)
                #pay_scores = self._pay_model.compute_pay_score(state, al_with_placeholders)

                combined_scores = say_scores * can_scores  # * pay_scores

                selected_skill_string = actions_with_placeholders[np.argmax(combined_scores)]
                print("---------------------------------")

                print("SAY SCORES:")
                display_scores(say_scores, actions_with_placeholders)
                # time.sleep(10)
                print("\nCAN-PRE SCORES:")
                display_scores(can_scores, actions_with_placeholders)
                # time.sleep(10)
                print("\n")

                print(f'Selected action: {selected_skill_string}\n' )
                print("Now selecting parameters...\n")
                print("---------------------------------")

                '''
                Select parameter for skill
                '''
                parameterized_actions =  generate_actions(
                                                    [selected_skill_string], 
                                                    self.environment_description.heights, 
                                                    self.environment_description.wps, 
                                                    self.environment_description.durations, 
                                                    self.environment_description.objects)

                can_scores = self.can_model.compute_affordance_score(state, parameterized_actions, self)
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
                say_scores = self.say_model.compute_llm_score(stitched_prompt, state, use_llm_state, filtered_actions, _synchronous_calls=False)
                pay_scores = self.pay_model.compute_pay_score(state, filtered_actions)

                combined_scores = say_scores * filtered_can_scores * pay_scores

                selected_skill_string = filtered_actions[np.argmax(combined_scores)]

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
        print(f'\nSelected final parameterized action: {selected_skill_string}\n')
        return selected_skill_string, n_actions_computed


    def plan_mission(self, team_id):
        """
        Orchestrates the planning and execution of the mission.
        """
        print(f"[INFO]: LLM Planner Selected for team : {team_id}")

        goal_description = self.extract_goal_information(human_input=False)
        if not goal_description:
            return

        print(f"Goal Description: {goal_description}")

        # Build the initial prompt
        self.main_prompt = self.environment_description.env_description
        self.goal_prompt = f"\n**Goal:** {goal_description}\n"

        # Model selection
        self.reaction_mode = "llm"
        self.mode = "SayCan-CanSayPay"

        # Generate Initial Instruction
        instruction_prompt = self.environment_description.instruction_prompt.format(
            action_params_description=self.environment_description.action_string_with_description,
            skill=self.action_list_string,
            goal_prompt=self.goal_prompt,
            search_object=self.target_object,
            search_area=self.target_area
        )
        stitched_prompt = self.main_prompt + instruction_prompt

        # Select Next Action
        next_action, _ = self.get_next_action(
            mode=self.mode,
            state=self.drone_state,
            use_llm_state=False,
            actions_with_params=self.action_list_parameterized,
            actions_without_params=self.action_list_without_params,
            actions_with_placeholders=self.action_list_with_placeholders,
            stitched_prompt=stitched_prompt
        )

        print(f"Planned Next Action: {next_action}")
        '''
        Execute next action?
        '''
        self.current_action = next_action
        self.action_count = self.action_count + 1

        action_msg = self.environment_description.NL2action_msg(next_action, self.action_count)
        action_msg = create_up_msg([{"platform_id":"vhcl0"}], team_id, [[action_msg]])

        if action_msg == None:
            action_msg = []
        return action_msg

    def update_state(self, state):
        """
        Updates the drone state.
        """
        self.drone_state = state

    def feedback(self, team_id, platform_id, feedback_message):
        """
        Process feedback from the drone.
        """
        pass

    def result(self, team_id, platform_id, result_message):
        """
        Process the result of the executed action.
        """
        print("Finished action. Planning new if applicable...")
        if self.action_count >= 1:
            if self.current_action != self.final_action:
                self.drone_state._airborne_state = DroneStateInfo.AIRBORNE
            else:
                self.drone_state._airborne_state = DroneStateInfo.LANDED

        if result_message.success == True:
            '''
            Update the POMDP for the AEMS2 heuristic
            '''
            self.pay_model.update_pomdp(self.current_action, 1) # 1 means not detected and 0 mean detected

            '''
            Add skill to prompt
            '''
            self.action_list_string +=  self.current_action + "\n"

            if self.current_action != self.final_action and self.action_count <= self.max_allowed_actions:
                return self.plan_mission(team_id)
            else:
                print("Final Action. Shutting down LLM Planner.")
        else:
            pass


    def on_destroy(self):
        """
        Cleanup operations when the planner is destroyed.
        """
        if dist.is_initialized():
            dist.destroy_process_group()
