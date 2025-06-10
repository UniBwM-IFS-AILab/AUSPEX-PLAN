#!/usr/bin/env python3
import json
import time
import rclpy
import time
import json

from rclpy.node import Node

from typing import Dict, Any

from builtin_interfaces.msg import Time as RosTime
from auspex_msgs.msg import Goal as AuspexGoal
from auspex_msgs.msg import Constraint as AuspexConstraint
from auspex_msgs.msg import GoalCondition
import rosidl_runtime_py
from .nlp_llm import NLP_LLM
from .nlp_classic import NLP_CLASSIC

class GoalHandler(Node):
    def __init__(self, kb_client=None):
        """
        Initialize the goal handler
        """
        super().__init__('goal_handler_node')

        self._nlp_llm = NLP_LLM(provider="openai", model="gpt-4o-mini", kb_client=kb_client)
        self._nlp_classic = NLP_CLASSIC(kb_client=kb_client)

        self._kb_client = kb_client

        self._goal_id_counter = 0

        self._constraint_id_counter = 0

        self._team_missions = {}

        self._timer = self.create_timer(1, self._extract_goals)

    def _extract_goals(self):
        mission_dict = self._kb_client.query('mission')
        # TODO delete mission message from database

        if not mission_dict:
            team_id = "drone_team"# TODO: Remove this line after testing
            if team_id in self._team_missions and self._team_missions[team_id] is not None: # TODO: Remove this line after testing
                return # TODO: Remove this line after testing
            self.get_logger().info("[ERROR]: No mission found.")

            self.get_logger().info("[DEV]: Default mission:")# TODO: Remove this line after testing
            mission_dict = [{# TODO: Remove this line after testing
                "team_id": team_id,# TODO: Remove this line after testing
                "mission_type": "SearchMission",# TODO: Remove this line after testing
                "mission_goal":  "Search the area openareas and find a person.",# TODO: Remove this line after testing
                "search_area": "openareas",# TODO: Remove this line after testing
                "target_object": "person"# TODO: Remove this line after testing
            }]# TODO: Remove this line after testing
            self.get_logger().info("[DEV]: DELETE ME AFTER TESTS")# TODO: Remove this line after testing

        for mission in mission_dict:
            if 'team_id' in mission:
                team_id = mission['team_id']
            else:
                continue

            if team_id in self._team_missions and self._team_missions[team_id] is not None:
                if self._team_missions[team_id] == mission:
                    continue # No new mission

            self.get_logger().info("Processing Mission")

            self._team_missions[team_id] = mission
            goals = []
            constraints = []

            # Check for search area
            if 'search_area' in self._team_missions[team_id] and 'points' in self._team_missions[team_id]['search_area']:
                self.get_logger().info(f"Search area defined with points")
                # Create a goal for the search area
                [search_area_goal, search_area_constraints] = self.process_mission(team_id, self._team_missions[team_id])
                if search_area_goal:
                    goals.extend(search_area_goal)
                if search_area_constraints:
                    constraints.extend(search_area_constraints)

            # Check for target objects
            elif 'target_objects' in self._team_missions[team_id] and self._team_missions[team_id]['target_objects'] and 'search_area' not in self._team_missions[team_id]:
                target_objects = self._team_missions[team_id]['target_objects']
                self.get_logger().info(f"Target objects defined: {target_objects}")
                # Create a goal for each target object
                [target_object_goal, target_object_constraints] = self.process_target(team_id, self._team_missions[team_id])
                if target_object_goal:
                    goals.extend(target_object_goal)
                if target_object_constraints:
                    constraints.extend(target_object_constraints)

            # Check for mission goal
            elif 'mission_goal' in self._team_missions[team_id] and self._team_missions[team_id]['mission_goal']:
                mission_goal = self._team_missions[team_id]['mission_goal']
                self.get_logger().info(f"Mission goal defined: {mission_goal}")
                # Process the mission goal as a natural language instruction
                [mission_goal_goals, mission_goal_constraints] = self.process_instruction(team_id, self._team_missions[team_id])
                if mission_goal_goals:
                    goals.extend(mission_goal_goals)
                if mission_goal_constraints:
                    constraints.extend(mission_goal_constraints)

            # TODO Concat goals here and remove elif

            if not goals:
                self.get_logger().info("[ERROR]: No goals found.")
                continue

            for goal in goals:
                self.insert_goal_to_KB(goal)
            # for constraint in constraints:
            #     self.insert_constraint_to_KB(constraints)

            self.get_logger().info("Mission processed and goals/constraints added to KB.")

    def insert_goal_to_KB(self, goal):
        """
        Inserts the goal into the knowledge base
        """
        params = goal.parameters_json
        goal.parameters_json = ""
        auspex_goal = rosidl_runtime_py.convert.message_to_ordereddict(goal)
        json_params = {}

        try:
            json_params = json.loads(params)
        except Exception as e:
            pass

        auspex_goal['parameters_json'] = json_params
        auspex_goal_json = ''
        try:
            auspex_goal_json = json.dumps(auspex_goal)
        except Exception as e:
            self.get_logger().info("[ERROR]: Can not dump json file.")

        self._kb_client.insert(collection='goal', entity=auspex_goal_json)

    def insert_constraint_to_KB(self, constraint):
        """
        Inserts the constraint into the knowledge base
        """
        auspex_constraint = rosidl_runtime_py.convert.message_to_ordereddict(constraint)
        auspex_constraint_json = ''
        try:
            auspex_constraint_json = json.dumps(auspex_constraint)
        except Exception as e:
            self.get_logger().info("[ERROR]: not a valid json")
        self._kb_client.insert(collection='constraint', entity=auspex_constraint_json)

    def delete_all_goals(self):
        """
        Deletes all goals from the knowledge base
        """
        self._kb_client.delete(collection='goal')
        #TODO

    """
    ###############################################################################################
    # For building goals and constraints from only a target description in the mission definition #
    ###############################################################################################
    """

    def process_target(self, team_id, mission):
        """
        Processes a SearchMission dictionary and generates a simplified target-focused goal and constraints.
        mission: SearchMission as dictionary containing target information, no-fly zones, etc.
        """
        goals = []
        constraints = []

        # Create the main search goal
        find_goal = AuspexGoal()
        find_goal.goal_id = f"goal_{self._goal_id_counter}"
        self._goal_id_counter += 1
        find_goal.description = f"Find {', '.join(mission['target_objects'])}" if mission['target_objects'] else "Search mission"
        find_goal.type = "FIND"
        find_goal.priority = 5  # Default priority
        find_goal.status = "UNPLANNED"
        find_goal.team_id = team_id
        find_goal.created_at = RosTime(sec=int(time.time()), nanosec=0)
        find_goal.deadline = RosTime(sec=0, nanosec=0)  # No specific deadline
        parameters = {
            "target_objects": mission['target_objects']
        }
        if 'starting_point' in mission:
            parameters["starting_point"] = {
                "latitude": mission['starting_point']['latitude'],
                "longitude": mission['starting_point']['longitude'],
                "altitude": mission['starting_point']['altitude']
            }
        find_goal.parameters_json = json.dumps(parameters)
        goals.append(find_goal)

        landed_goal = AuspexGoal()
        landed_goal.goal_id = f"goal_{self._goal_id_counter}"
        self._goal_id_counter += 1
        landed_goal.description = "Land the drone."
        landed_goal.type = "LAND"
        landed_goal.priority = 5
        landed_goal.status = "UNPLANNED"
        landed_goal.team_id = team_id
        landed_goal.created_at = RosTime(sec=int(time.time()), nanosec=0)
        landed_goal.deadline = RosTime(sec=0, nanosec=0)

        # Composite Goal
        and1_goal = AuspexGoal()
        and1_goal.goal_id = f"goal_{self._goal_id_counter}"
        self._goal_id_counter += 1
        and1_goal.description = "TOP_LEVEL_GOAL"
        and1_goal.type = "AND"
        and1_goal.status = "-"
        and1_goal.team_id = team_id
        and1_goal.goal_condition.logic_operator = GoalCondition.AND
        and1_goal.goal_condition.goal_ids = [landed_goal.goal_id, find_goal.goal_id]

        goals.append(landed_goal)
        goals.append(and1_goal)

        # Create constraints from no-fly zones
        if 'no_fly_zones' in mission:
            for i, no_fly_zone in enumerate(mission['no_fly_zones']):
                constraint_msg = AuspexConstraint()
                constraint_msg.constraint_id = f"constraint_{self._constraint_id_counter}"
                self._constraint_id_counter += 1
                constraint_msg.type = "NO_FLY_ZONE"
                constraint_msg.description = no_fly_zone['description'] if 'description' in no_fly_zone else f"No-fly zone {i+1}"

                constraint_params = {
                    "type": no_fly_zone['type'],  # Should be AREA_TYPE_NO_FLY (0)
                    "points": [
                        {"latitude": point['latitude'], "longitude": point['longitude'], "altitude": point['altitude']}
                        for point in no_fly_zone['points']
                    ]
                }
                constraint_msg.parameters_json = json.dumps(constraint_params)
                constraint_msg.severity = "HARD"  # No-fly zones should be strictly observed
                constraint_msg.violation_cost = 1000.0  # High cost for violation
                constraints.append(constraint_msg)

        # Create constraints for danger zones
        if 'danger_zones' in mission:
            for i, danger_zone in enumerate(mission['danger_zones']):
                constraint_msg = AuspexConstraint()
                constraint_msg.constraint_id = f"constraint_{self._constraint_id_counter}"
                self._constraint_id_counter += 1
                constraint_msg.type = "DANGER_ZONE"
                constraint_msg.description = danger_zone['description'] if 'description' in danger_zone else f"Danger zone {i+1}"

                constraint_params = {
                    "type": danger_zone['type'],  # Should be AREA_TYPE_DANGER (2)
                    "points": [
                        {"latitude": point['latitude'], "longitude": point['longitude'], "altitude": point['altitude']}
                        for point in danger_zone['points']
                    ]
                }
                constraint_msg.parameters_json = json.dumps(constraint_params)
                constraint_msg.severity = "SOFT"  # Danger zones can be entered if necessary
                constraint_msg.violation_cost = 100.0  # Medium cost for violation
                constraints.append(constraint_msg)

        # Height constraints
        if 'max_height' in mission and mission['max_height'] > 0:
            constraint_msg = AuspexConstraint()
            constraint_msg.constraint_id = f"constraint_{self._constraint_id_counter}"
            self._constraint_id_counter += 1
            constraint_msg.type = "MAX_HEIGHT"
            constraint_msg.description = f"Maximum flight height: {mission['max_height']}m AGL"
            constraint_msg.parameters_json = json.dumps({"max_height": mission['max_height']})
            constraint_msg.severity = "HARD"
            constraint_msg.violation_cost = 500.0
            constraints.append(constraint_msg)

        if 'min_height' in mission and mission['min_height'] > 0:
            constraint_msg = AuspexConstraint()
            constraint_msg.constraint_id = f"constraint_{self._constraint_id_counter}"
            self._constraint_id_counter += 1
            constraint_msg.type = "MIN_HEIGHT"
            constraint_msg.description = f"Minimum flight height: {mission['min_height']}m AGL"
            constraint_msg.parameters_json = json.dumps({"min_height": mission['min_height']})
            constraint_msg.severity = "HARD"
            constraint_msg.violation_cost = 500.0
            constraints.append(constraint_msg)

        for goal in goals:
            goal.constraint_ids = [constraint.constraint_id for constraint in constraints]

        return [goals, constraints]

    """
    ################################################################
    # For building goals and constraints from a mission definition #
    ################################################################
    """

    def process_mission(self, team_id, mission):
        """
        Processes a SearchMission dictionary and generates corresponding goals and constraints.
        mission: SearchMission as dictionary containing area information, no-fly zones, etc.
        """
        goals = []
        constraints = []

        search_goal = AuspexGoal()

        search_goal.goal_id = f"goal_{self._goal_id_counter}"
        self._goal_id_counter += 1
        search_goal.description = mission['mission_goal'] if mission['mission_goal'] else "Search mission"
        search_goal.type = "SEARCH"
        search_goal.priority = 5  # Default priority
        search_goal.status = "UNPLANNED"
        search_goal.team_id = team_id
        search_goal.created_at = RosTime(sec=int(time.time()), nanosec=0)
        search_goal.deadline = RosTime(sec=0, nanosec=0)  # No specific deadline

        search_area_points = [
            {"latitude": point['latitude'], "longitude": point['longitude'], "altitude": point['altitude']}
            for point in mission['search_area']['points']
        ]

        try:

            if mission['desired_ground_dist']:
                height = mission['desired_ground_dist']
            else:
                height = (mission['max_height'] + mission['min_height']) / 2 if mission['max_height'] and mission['min_height'] else 50.0

            # Compute the centroid of the given points
            lat_sum = sum(p['latitude'] for p in mission['search_area']['points'])
            lon_sum = sum(p['longitude'] for p in mission['search_area']['points'])
            num_points = len(mission['search_area']['points'])
            centroid = [lat_sum / num_points,lon_sum / num_points,height]
            area_str = json.dumps({
                "name": 'manual_search_area_' + team_id,
                "points": [centroid] + [[p['latitude'], p['longitude'], height] for p in mission['search_area']['points']]
            })
            self._kb_client.write(collection='area', entity=area_str, key='name', value='manual_search_area_' + team_id)
        except Exception as e:
            self.get_logger().info(f"[ERROR]: Could not write search area to Knowledge Base: {e}")

        pois = [
            {"latitude": poi['latitude'], "longitude": poi['longitude'], "altitude": poi['altitude']}
            for poi in mission['pois']
        ]

        parameters = {
            "search_area": {
                "type": mission['search_area']['type'],
                "description": mission['search_area']['description'],
                "points": search_area_points
            },
            "starting_point": {
                "latitude": mission['starting_point']['latitude'],
                "longitude": mission['starting_point']['longitude'],
                "altitude": mission['starting_point']['altitude']
            },
            "max_height": mission['max_height'],
            "min_height": mission['min_height'],
            "desired_ground_dist": mission['desired_ground_dist'],
            "pois": pois
        }

        if 'sensor_mode' in mission and 'mode' in mission['sensor_mode']:
            parameters["sensor_mode"] = mission['sensor_mode']['mode']

        if 'prio_areas' in mission and mission['prio_areas']:
            parameters["priority_areas"] = [
                {
                    "type": area['type'],
                    "description": area['description'],
                    "points": [
                        {"latitude": point['latitude'], "longitude": point['longitude'], "altitude": point['altitude']}
                        for point in area['points']
                    ]
                }
                for area in mission['prio_areas']
            ]

        search_goal.parameters_json = json.dumps(parameters)
        goals.append(search_goal)

        if mission['target_objects']:
            find_goal = AuspexGoal()
            find_goal.goal_id = f"goal_{self._goal_id_counter}"
            self._goal_id_counter += 1
            find_goal.description = mission['mission_goal'] if mission['mission_goal'] else "Search mission"
            find_goal.type = "FIND"
            find_goal.priority = 5  # Default priority
            find_goal.status = "UNPLANNED"
            find_goal.team_id = team_id
            find_goal.created_at = RosTime(sec=int(time.time()), nanosec=0)
            find_goal.deadline = RosTime(sec=0, nanosec=0)  # No specific deadline
            find_params_parameters = {
                "target_objects": mission['target_objects'],
            }
            find_goal.parameters_json = json.dumps(find_params_parameters)
            goals.append(find_goal)

        or1_goal = AuspexGoal()
        or1_goal.goal_id = f"goal_{self._goal_id_counter}"
        self._goal_id_counter += 1
        or1_goal.description = "OR"
        or1_goal.type = "OR"
        or1_goal.status = "-"
        or1_goal.team_id = team_id
        or1_goal.goal_condition.logic_operator = GoalCondition.OR
        or1_goal.goal_condition.goal_ids = [goal.goal_id for goal in goals]

        landed_goal = AuspexGoal()
        landed_goal.goal_id = f"goal_{self._goal_id_counter}"
        self._goal_id_counter += 1
        landed_goal.description = "Land the drone."
        landed_goal.type = "LAND"
        landed_goal.priority = 5
        landed_goal.status = "UNPLANNED"
        landed_goal.team_id = team_id
        landed_goal.created_at = RosTime(sec=int(time.time()), nanosec=0)
        landed_goal.deadline = RosTime(sec=0, nanosec=0)

        and1_goal = AuspexGoal()
        and1_goal.goal_id = f"goal_{self._goal_id_counter}"
        self._goal_id_counter += 1
        and1_goal.description = "TOP_LEVEL_GOAL"
        and1_goal.type = "AND"
        and1_goal.status = "-"
        and1_goal.team_id = team_id
        and1_goal.goal_condition.logic_operator = GoalCondition.AND
        and1_goal.goal_condition.goal_ids = [landed_goal.goal_id, search_goal.goal_id if len(goals) == 1 else or1_goal.goal_id]

        goals.append(or1_goal) if len(goals) != 1 else None
        goals.append(landed_goal)
        goals.append(and1_goal)

        #TODO Below ToDo Constraints
        # Create constraints from no-fly zones
        if 'no_fly_zones' in mission:
            for i, no_fly_zone in enumerate(mission['no_fly_zones']):
                constraint_msg = AuspexConstraint()
                constraint_msg.constraint_id = f"constraint_{self._constraint_id_counter}"
                self._constraint_id_counter += 1
                constraint_msg.type = "NO_FLY_ZONE"
                constraint_msg.description = no_fly_zone['description'] if 'description' in no_fly_zone else f"No-fly zone {i+1}"

                constraint_params = {
                    "type": no_fly_zone['type'],  # Should be AREA_TYPE_NO_FLY (0)
                    "points": [
                        {"latitude": point['latitude'], "longitude": point['longitude'], "altitude": point['altitude']}
                        for point in no_fly_zone['points']
                    ]
                }
                constraint_msg.parameters_json = json.dumps(constraint_params)
                constraint_msg.severity = "HARD"  # No-fly zones should be strictly observed
                constraint_msg.violation_cost = 1000.0  # High cost for violation
                constraints.append(constraint_msg)

        # Create constraints for danger zones
        if 'danger_zones' in mission:
            for i, danger_zone in enumerate(mission['danger_zones']):
                constraint_msg = AuspexConstraint()
                constraint_msg.constraint_id = f"constraint_{self._constraint_id_counter}"
                self._constraint_id_counter += 1
                constraint_msg.type = "DANGER_ZONE"
                constraint_msg.description = danger_zone['description'] if 'description' in danger_zone else f"Danger zone {i+1}"

                constraint_params = {
                    "type": danger_zone['type'],  # Should be AREA_TYPE_DANGER (2)
                    "points": [
                        {"latitude": point['latitude'], "longitude": point['longitude'], "altitude": point['altitude']}
                        for point in danger_zone['points']
                    ]
                }
                constraint_msg.parameters_json = json.dumps(constraint_params)
                constraint_msg.severity = "SOFT"  # Danger zones can be entered if necessary
                constraint_msg.violation_cost = 100.0  # Medium cost for violation
                constraints.append(constraint_msg)

        # Height constraints
        if 'max_height' in mission and int(mission['max_height']) > 0:
            constraint_msg = AuspexConstraint()
            constraint_msg.constraint_id = f"constraint_{self._constraint_id_counter}"
            self._constraint_id_counter += 1
            constraint_msg.type = "MAX_HEIGHT"
            constraint_msg.description = f"Maximum flight height: {mission['max_height']}m AGL"
            constraint_msg.parameters_json = json.dumps({"max_height": mission['max_height']})
            constraint_msg.severity = "HARD"
            constraint_msg.violation_cost = 500.0
            constraints.append(constraint_msg)

        if 'min_height' in mission and int(mission['min_height']) > 0:
            constraint_msg = AuspexConstraint()
            constraint_msg.constraint_id = f"constraint_{self._constraint_id_counter}"
            self._constraint_id_counter += 1
            constraint_msg.type = "MIN_HEIGHT"
            constraint_msg.description = f"Minimum flight height: {mission['min_height']}m AGL"
            constraint_msg.parameters_json = json.dumps({"min_height": mission['min_height']})
            constraint_msg.severity = "HARD"
            constraint_msg.violation_cost = 500.0
            constraints.append(constraint_msg)

        for goal in goals:
            goal.constraint_ids = [constraint.constraint_id for constraint in constraints]

        return [goals, constraints]


    """
    #########################################################################
    # For building goals and constraints from natural language instructions #
    #########################################################################
    """
    def process_instruction(self, team_id, mission):
        """
        Processes a natural language instruction and generates goals and constraints.

        Args:
            instruction (str): The natural language instruction.

        Returns:
            list[AuspexGoal]: A list of generated goal messages.
        """
        instruction = mission['mission_goal']
        parsed_vars = self._nlp_llm.parse_instruction2dict(instruction)

        goals = self._build_goals_from_nlp_llm(team_id, parsed_vars, instruction)
        constraints = self._build_constraints_from_nlp_llm(team_id, parsed_vars)

        constraint_ids = [constraint.constraint_id for constraint in constraints]
        for goal in goals:
            goal.constraint_ids = constraint_ids

        return [goals, constraints]

    def _build_goals_from_nlp_llm(self, team_id: str, parsed_vars: dict, instruction: str) -> list[AuspexGoal]:
        goals = []

        goals_dict = parsed_vars.get("goals", [])

        for goal_dict in goals_dict:
            goal_msg = AuspexGoal()
            goal_msg.goal_id = f"goal_{self._goal_id_counter}"
            goal_msg.type = goal_dict.get("type", "NOT_SPECIFIED").upper()
            goal_msg.priority = goal_dict.get("priority", 5)
            goal_msg.team_id = team_id

            goal_msg.description = instruction
            goal_msg.status = "UNPLANNED"

            goal_msg.created_at = RosTime(sec=int(time.time()), nanosec=0)
            goal_msg.deadline = RosTime(sec=parsed_vars.get("deadline_sec", 0), nanosec=0)

            goal_dict.pop("type", None)
            goal_dict.pop("team_id", None)
            goal_dict.pop("priority", None)
            goal_dict.pop("deadline_sec", None)

            goal_msg.parameters_json = json.dumps(goal_dict)
            goal_msg.constraint_ids = []
            goals.append(goal_msg)
            self._goal_id_counter += 1

        landed_goal = AuspexGoal()
        landed_goal.goal_id = f"goal_{self._goal_id_counter}"
        self._goal_id_counter += 1
        landed_goal.description = "Land the drone."
        landed_goal.type = "LAND"
        landed_goal.priority = 5
        landed_goal.status = "UNPLANNED"
        landed_goal.team_id = team_id
        landed_goal.created_at = RosTime(sec=int(time.time()), nanosec=0)
        landed_goal.deadline = RosTime(sec=0, nanosec=0)

        # Composite Goals
        or1_goal = AuspexGoal()
        or1_goal.goal_id = f"goal_{self._goal_id_counter}"
        self._goal_id_counter += 1
        or1_goal.description = "OR"
        or1_goal.type = "OR"
        or1_goal.status = "-"
        or1_goal.team_id = team_id
        or1_goal.goal_condition.logic_operator = GoalCondition.OR
        or1_goal.goal_condition.goal_ids = [goal.goal_id for goal in goals]

        and1_goal = AuspexGoal()
        and1_goal.goal_id = f"goal_{self._goal_id_counter}"
        self._goal_id_counter += 1
        and1_goal.description = "TOP_LEVEL_GOAL"
        and1_goal.type = "AND"
        and1_goal.status = "-"
        and1_goal.team_id = team_id
        and1_goal.goal_condition.logic_operator = GoalCondition.AND
        and1_goal.goal_condition.goal_ids = [landed_goal.goal_id, or1_goal.goal_id]

        goals.append(landed_goal)
        goals.append(or1_goal)
        goals.append(and1_goal)

        return goals

    def _build_constraints_from_nlp_llm(self, team_id:str, parsed_vars: dict) -> list[AuspexConstraint]:
        constraints = [] # safety vs replannable constraints

        if "constraints" in parsed_vars:
            for constraint_data in parsed_vars["constraints"]:
                constraint_msg = AuspexConstraint()
                # Metadata
                constraint_msg.constraint_id = f"constraint_{self._constraint_id_counter}"
                self._constraint_id_counter += 1
                constraint_msg.type = constraint_data.get("type", "NOT_SPECIFIED").upper()
                constraint_msg.description = constraint_data.get("description", "No description provided.")
                constraint_msg.parameters_json = json.dumps(constraint_data.get("parameters", {}))
                constraint_msg.severity = constraint_data.get("severity", "SOFT")
                constraint_msg.violation_cost = float(constraint_data.get("violation_cost", 0.0))

                constraints.append(constraint_msg)

        return constraints