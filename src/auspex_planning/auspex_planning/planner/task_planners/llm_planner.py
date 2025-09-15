#!/usr/bin/env python3
import ast
import copy
import json
import time
from enum import Enum

import numpy as np
import torch.distributed as dist

from auspex_msgs.msg import ExecutorState, Plan
from auspex_planning.planner.task_planners.planner_base import PlannerBase
from auspex_planning.planner.utils.converter import AUSPEXConverter
from auspex_planning.planner.utils.llm_planner_utils.env_discription import EnvDiscription
from auspex_planning.planner.utils.llm_planner_utils.model_can import CanModel
from auspex_planning.planner.utils.llm_planner_utils.model_pay import PayModel
from auspex_planning.planner.utils.llm_planner_utils.model_say import SayModel
from auspex_planning.planner.utils.llm_planner_utils.utils import *

TAKE_OFF_ACTION = "Take off"
LAND_ACTION = "Land"
DEFAULT_MAX_ACTIONS = 15
DEFAULT_TOP_K = 10


class LLM_Planner(PlannerBase):
    planner_key = 'llm_planner'
    """
    A Planner class implementing Say-ReapEx framework for drone mission planning.
    """

    def __init__(self, kb_client):
        """
        Initialize the planner with environment description, knowledge base,
        and high-level models for decision making.
        
        Args:
            kb_client: Knowledge base client for data queries
        """
        self.initialized = False
        
        # Core components
        self._kb_client = kb_client
        self._environment_description = EnvDiscription()
        self._converter = AUSPEXConverter()

        # Current state
        self._drone_state = None
        self._current_platforms = []

        # Mission parameters (extracted from goals)
        self._target_object = ""
        self._target_color = ""
        self._target_area = ""
        self._target_season = ""
        self._goal = ""
        self._additional_information = []

        # Execution tracking
        self._current_action = ""
        self._action_count = 0
        self._max_allowed_actions = DEFAULT_MAX_ACTIONS
        self._action_history = ""
        self._final_action = "Return to Home and Land."
        
        # Predefined actions queue
        self._manual_action_queue = [
            "Take off to 20 metres altitude.",
            "Return to Home and Land."
        ]

        # Performance tracking
        self._total_time_action_selection = 0.0

        # Operation areas from knowledge base
        self._operation_areas = self._load_operation_areas()
        
        # Initialize models
        self._initialize_models()
        
        if self._pay_model is not None and self._can_model is not None:
            self.initialized = True
            self._setup_environment_and_actions()
            print(f"Initialized with {len(self._action_list_parameterized)} actions.")

    def _load_operation_areas(self):
        """Load operation areas from knowledge base."""
        operation_areas = {}
        areas_db = self._kb_client.query(collection='area', key='', value='')
        
        if len(areas_db) > 0:
            for area in areas_db:
                if area:
                    operation_areas[area['name']] = area
        
        return operation_areas
    
    def _initialize_models(self):
        """Initialize the AI models used for planning."""
        self._say_model = SayModel(provider="openai", model="gpt-4o-mini")
        self._can_model = CanModel()
        self._pay_model = PayModel(self, self._operation_areas)
    
    def _setup_environment_and_actions(self):
        """Setup environment description and preprocess actions."""
        # Add waypoints to environment description
        waypoints_json = json.dumps(self._operation_areas)
        self._environment_description.env_description += f"\n{waypoints_json}"

        # Preprocess actions
        self._action_list_with_placeholders = splitActionStringToList(
            self._environment_description.action_string_with_description
        )
        self._action_list_without_params = self._environment_description.stripActionList(
            self._action_list_with_placeholders
        )
        self._action_list_parameterized = generate_actions(
            self._action_list_with_placeholders,
            self._environment_description.heights,
            self._environment_description.wps,
            self._environment_description.durations,
            self._environment_description.objects
        )

        # Setup POMDP in Pay Model
        self._pay_model.setup_pomdp(
            planner=self,
            states=self._environment_description.wps,
            actions=copy.deepcopy(self._action_list_parameterized)
        )

    def extract_goal_information(self, goals):
        """
        Extract goal-related information from user input or default instruction.
        
        Args:
            goals: List of goal objects from knowledge base
            
        Returns:
            str: Formatted goal description
        """
        # Default values
        detection_target = ["person"]
        location_target = ["openareas"]
        default_season = ["summer"]
        detection_color = ["NOT_SPECIFIED"]
        additional_info = "Backpack"

        formatted_goals = ""

        for goal in goals:
            if goal.get('status', '').upper() != 'UNPLANNED':
                continue

            goal_type = goal.get('type')
            if goal_type in ["AND", "OR"]:
                continue

            self._goal += f"{goal_type},"
            formatted_goal = ""

            if goal_type == "SEARCH":
                parameters = goal.get('parameters_json', {})
                location_target = parameters.get('locations', ["NOT_SPECIFIED"])
                
                if location_target[0] == "NOT_SPECIFIED":
                    formatted_goal = "SEARCH the areas close to a point of interest. If you found nothing, return home and land."
                else:
                    formatted_goal = f"SEARCH the areas specified by {','.join(location_target)}. If you found nothing, return home and land."

            elif goal_type == "FIND":
                parameters = goal.get('parameters_json', {})
                detection_target = parameters.get('objects', detection_target)
                detection_color = parameters.get('colors', detection_color)
                additional_info = parameters.get('additional_info', additional_info)

                formatted_goal = (
                    f"FIND and Detect objects: {','.join(detection_target)}. "
                    f"If you confirm one {','.join(detection_target)}, return home and land. "
                    f"Additional information: {','.join(additional_info)}"
                )
                
            elif goal_type == "LAND":
                formatted_goal = "LAND after completing the mission."

            formatted_goals += f"{formatted_goal}\n"

        # Update mission parameters
        self._target_object = ','.join(detection_target)
        self._target_color = ','.join(detection_color)
        self._target_area = ','.join(location_target)
        self._target_season = default_season[0]
        self._additional_information = ','.join(additional_info)
        
        return formatted_goals

    def get_next_action(self, mode="Say'n'Fly", stitched_prompt=""):
        """
        Get the next action using the specified planning mode.
        
        Args:
            mode: Planning mode ("SayCan", "SayCanPay", or "Say'n'Fly")
            stitched_prompt: Combined prompt for the LLM
            
        Returns:
            tuple: (selected_action_string, number_of_actions_computed)
        """
        start_time = time.time()
        
        # Check if predefined actions are available
        if self._manual_action_queue:
            selected_action = self._manual_action_queue.pop(0)
            return selected_action, 0

        # Get detected objects from knowledge base
        detected_objects = self._kb_client.query(collection='object')
        platform_status = self._drone_state.get('platform_status', 'UNKNOWN')
        
        actions_computed = 0

        if mode == "SayCan":
            selected_action, actions_computed = self._plan_with_saycan(
                stitched_prompt, platform_status, detected_objects
            )
            
        elif mode == "SayCanPay":
            selected_action, actions_computed = self._plan_with_saycanpay(
                stitched_prompt, platform_status, detected_objects
            )
            
        elif mode == "Say'n'Fly":
            selected_action, actions_computed = self._plan_with_saynfly(
                stitched_prompt, platform_status, detected_objects
            )
        else:
            raise ValueError(f"Unknown planning mode: {mode}")

        # Update timing statistics
        end_time = time.time()
        self._total_time_action_selection += (end_time - start_time)
        
        print(f'\nSelected final parameterized action: {selected_action}\n')
        return selected_action, actions_computed

    def _plan_with_saycan(self, stitched_prompt, platform_status, detected_objects):
        """Plan using SayCan approach."""
        print(f"Using SayCan with {len(self._action_list_parameterized)} actions")
        
        say_scores = self._say_model.compute_llm_score(
            stitched_prompt, self._action_list_parameterized, platform_status, synchronous_call=True
        )
        can_scores = self._can_model.compute_affordance_score(
            self._drone_state, detected_objects, self._action_list_parameterized, self
        )
        
        combined_scores = say_scores * can_scores
        selected_action = self._action_list_parameterized[np.argmax(combined_scores)]
        
        display_scores(say_scores, self._action_list_parameterized)
        return selected_action, len(self._action_list_parameterized)

    def _plan_with_saycanpay(self, stitched_prompt, platform_status, detected_objects):
        """Plan using SayCanPay approach."""
        say_scores = self._say_model.compute_llm_score(
            stitched_prompt, self._action_list_parameterized, platform_status, synchronous_call=True
        )
        can_scores = self._can_model.compute_affordance_score(
            self._drone_state, detected_objects, self._action_list_parameterized, self
        )
        pay_scores = self._pay_model.compute_pay_score(
            self._drone_state, self._action_list_parameterized
        )
        
        combined_scores = say_scores * can_scores * pay_scores
        selected_action = self._action_list_parameterized[np.argmax(combined_scores)]
        
        display_scores(say_scores, self._action_list_parameterized)
        return selected_action, len(self._action_list_parameterized)

    def _plan_with_saynfly(self, stitched_prompt, platform_status, detected_objects):
        """Plan using Say'n'Fly approach with two-stage selection."""
        print("-" * 50)
        
        # Stage 1: Select action type
        say_scores = self._say_model.compute_llm_score(
            stitched_prompt, self._action_list_with_placeholders, 
            state=platform_status, synchronous_call=False
        )
        can_scores = self._can_model.compute_affordance_score(
            self._drone_state, detected_objects, self._action_list_with_placeholders, self
        )
        
        combined_scores = say_scores * can_scores
        selected_action_template = self._action_list_with_placeholders[np.argmax(combined_scores)]
        
        print("SAY SCORES:")
        display_scores(say_scores, self._action_list_with_placeholders)
        print("\nCAN-PRE SCORES:")
        display_scores(can_scores, self._action_list_with_placeholders)
        print(f'\nSelected action template: {selected_action_template}')
        print("Now selecting parameters...")
        print("-" * 50)
        
        # Stage 2: Select parameters for the chosen action
        parameterized_actions = generate_actions(
            [selected_action_template],
            self._environment_description.heights,
            self._environment_description.wps,
            self._environment_description.durations,
            self._environment_description.objects
        )
        
        # Filter top-k actions based on affordance scores
        can_scores = self._can_model.compute_affordance_score(
            self._drone_state, detected_objects, parameterized_actions, self
        )
        
        print("CAN-COMP SCORES:")
        display_scores(can_scores, parameterized_actions)
        
        filtered_actions, filtered_can_scores = self._filter_top_k_actions(
            parameterized_actions, can_scores, DEFAULT_TOP_K
        )
        
        print(f"Reduced action space from {len(self._action_list_parameterized)} to {len(filtered_actions)} actions.")
        
        # Final scoring with all three models
        say_scores = self._say_model.compute_llm_score(
            stitched_prompt, filtered_actions, state=platform_status, synchronous_call=False
        )
        pay_scores = self._pay_model.compute_pay_score(self._drone_state, filtered_actions)
        
        combined_scores = say_scores * filtered_can_scores * pay_scores
        selected_action = filtered_actions[np.argmax(combined_scores)]
        
        print("\nSAY SCORES:")
        display_scores(say_scores, filtered_actions)
        print("\nPAY SCORES:")
        display_scores(pay_scores, filtered_actions)
        print("-" * 50)
        
        total_actions_computed = len(self._action_list_without_params) + len(filtered_actions)
        return selected_action, total_actions_computed
    
    def _filter_top_k_actions(self, actions, scores, top_k):
        """Filter actions to keep only top-k based on scores."""
        if top_k >= len(scores):
            return np.array(actions), scores
        
        # Sort actions by scores and keep top-k
        paired_list = list(zip(scores, actions))
        sorted_list = sorted(paired_list, key=lambda x: x[0], reverse=True)
        top_scores, top_actions = zip(*sorted_list[:top_k])
        
        return np.array(top_actions), np.array(top_scores)


    def plan_mission(self, team_id):
        """
        Orchestrate the planning and execution of the mission.
        
        Args:
            team_id: The team identifier for which to plan the mission
            
        Returns:
            list: List of Plan messages for execution
        """
        if not self.initialized:
            return []

        print(f"[INFO]: LLM Planner selected for team: {team_id}")

        # Get platforms for the team
        self._current_platforms = self._kb_client.query(
            collection='platform', field='platform_id', key='team_id', value=team_id
        )

        if not self._current_platforms:
            print("No platforms found.")
            return []

        self._update_state(self._current_platforms[0]['platform_id'])

        # Get goals for the team
        goals = self._kb_client.query(collection='goal', field='', key='team_id', value=team_id)
        if not goals:
            print("No goals found.")
            return []

        # Extract constraints
        constraints_text = self._extract_constraints(goals)
        
        # Extract goal information and build prompts
        goal_description = self.extract_goal_information(goals)
        print(f"Goal Description: {goal_description}")

        main_prompt = self._environment_description.env_description
        goal_prompt = f"\n**Goal:** {goal_description}\n"
        if constraints_text:
            goal_prompt += f"\n**Constraints:**\n{constraints_text}\n"

        # Build instruction prompt
        instruction_prompt = self._environment_description.instruction_prompt.format(
            action_params_description=self._environment_description.action_string_with_description,
            actions=self._action_history,
            goal_prompt=goal_prompt,
            search_object=self._target_object,
            search_area=self._target_area
        )
        
        stitched_prompt = main_prompt + instruction_prompt

        # Select next action
        next_action, _ = self.get_next_action(
            mode="Say'n'Fly",
            stitched_prompt=stitched_prompt
        )

        print(f"Planned Next Action: {next_action}")

        if "continue" in next_action.lower():
            print("Continue action. No new action planned.")
            return ["Continue"]

        # Update execution state
        self._current_action = next_action
        self._action_count += 1

        # Convert to action message
        action_msg = self._environment_description.NL2action_msg(next_action, self._action_count)

        # Convert to AUSPEX plan
        planned_tasks = self._converter.convert_plan_llm2auspex(
            self._current_platforms[0]['platform_id'], team_id, [action_msg]
        )

        # Create plan message
        plan_msg = Plan()
        plan_msg.tasks = planned_tasks
        plan_msg.platform_id = self._current_platforms[0]['platform_id']
        plan_msg.team_id = team_id
        plan_msg.priority = 0

        return [plan_msg]
    
    def _extract_constraints(self, goals):
        """Extract and format constraints from goals."""
        constraint_ids = []
        for goal in goals:
            for constraint_id in goal.get('constraint_ids', []):
                if constraint_id not in constraint_ids:
                    constraint_ids.append(constraint_id)
        
        if not constraint_ids:
            print("[INFO]: No constraints found.")
            return ""
        
        constraints = []
        for constraint_id in constraint_ids:
            constraint = self._kb_client.query(
                collection='constraint', field='', key='constraint_id', value=constraint_id
            )
            if constraint:
                constraints.append(constraint[0])
        
        return "\n".join([f"- {c.get('description', 'No description')}" for c in constraints])

    def _update_state(self, platform_id):
        """
        Update the drone state from knowledge base.
        
        Args:
            platform_id: The platform identifier to query for
        """
        drone_state_data = self._kb_client.query(collection='platform', key='platform_id', value=platform_id)
        if not drone_state_data:
            print("No drone state found.")
            return
        self._drone_state = drone_state_data[0]

    def feedback(self, team_id, feedback_message):
        """
        Process feedback from the drone.
        
        Args:
            team_id: The team identifier
            feedback_message: Feedback message from the platform
        """
        # Currently no specific feedback processing implemented
        pass

    def result(self, team_id, result_message):
        """
        Process the result of the executed action and plan next action if needed.
        
        Args:
            team_id: The team identifier
            result_message: Result message from the completed action
            
        Returns:
            list: Next plan if mission should continue, None otherwise
        """
        print("Finished action. Planning new if applicable...")
        
        # Update the POMDP for the AEMS2 heuristic
        # 1 means not detected, 0 means detected
        self._pay_model.update_pomdp(self._current_action, 1)

        # Add action to history
        self._action_history += self._current_action + "\n"

        # Check for status flags and update prompt
        status_flags = getattr(result_message.info, 'status_flags', [])
        self._current_additional_prompt = (
            f"Status Flags: {','.join(status_flags)}\n" if status_flags else ""
        )

        # Continue planning if not final action and within action limits
        if (self._current_action != self._final_action and 
            self._action_count <= self._max_allowed_actions):
            return self.plan_mission(team_id)
        else:
            print("Final action reached or max actions exceeded. Shutting down LLM Planner.")
            return None

    def on_destroy(self):
        """
        Cleanup operations when the planner is destroyed.
        """
        if dist.is_initialized():
            dist.destroy_process_group()