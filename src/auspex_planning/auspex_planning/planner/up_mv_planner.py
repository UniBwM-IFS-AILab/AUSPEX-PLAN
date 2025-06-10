 #!/usr/bin/env python3
import time
import rclpy
from .planner_base import PlannerBase
from unified_planning.shortcuts import *
from unified_planning.model import Problem
from unified_planning.engines import PlanGenerationResultStatus

from auspex_msgs.msg import Plan, ActionInstance

from .converter import AUSPEXConverter

class UP_MV_Planner(PlannerBase):
    planner_key = 'up_mv_planner'
    def __init__(self, kb_client):
        up.shortcuts.get_environment().credits_stream = None # disables the credits stream
        self._problem = None
        self._problem_name = "multi_uav_mission"

        self._converter = AUSPEXConverter()

        self._current_plan = None

        self._kb_client = kb_client

        self.set_problem_domain()
        print(f"[INFO]: Initialized Multi UP Problem {self._problem_name}")


    def plan_mission(self, team_id):
        vhcl_dict = self._kb_client.query('platform', 'platform_id', 'team_id', team_id)
        if not vhcl_dict:
            return []
        platform_id = vhcl_dict[0]['platform_id']

        self.set_problem_instance(team_id)

        if self._problem.goals == []:
            print(f"[INFO]: No goals found for team {team_id}.")
            return []

        """ Requests a plan from the planner. """
        with OneshotPlanner(problem_kind=self._problem.kind) as planner:
            result = planner.solve(self._problem)
            if result.status == PlanGenerationResultStatus.SOLVED_SATISFICING:
                print("Plan found.")
                self._current_plan = result.plan
            else:
                print("No plan found.")
                self._current_plan = None

        if self._current_plan is None:
            return []

        print(self._current_plan)
        plan_msg = Plan()
        plan_msg.tasks = self._converter.convert_plan_up2auspex(self._current_plan)
        plan_msg.team_id = team_id
        plan_msg.priority = 0
        plan_msg.platform_id = platform_id

        plans = []
        plans.append(plan_msg)
        return plans

    def set_problem_instance(self, team_id):
        self._problem.clear_goals()

        goal_expressions = {}
        goals = self._kb_client.query(collection='goal', key='team_id', value=team_id)

        top_level_goal_id = None
        composite_goals = {}

        if not goals:
            return []

        for goal in goals:
            goal_expression = None

            goal_status = goal.get('status', '').upper()
            goal_type = goal['type']
            goal_id = goal['goal_id']



            if goal['description'] == 'TOP_LEVEL_GOAL':
                top_level_goal_id = goal_id

            if goal_type == 'AND' or goal_type == 'OR':
                composite_goals[goal_id] = goal
                continue

            if goal_status != 'UNPLANNED':
                continue

            if goal_type == 'SEARCH':

                if goal.get('parameters_json',{}).get('search_area',{}).get('points',[]):

                    print("Got search area points adding search are to Problem...")
                    if self._problem.has_object('manual_search_area'):
                        search_area = self._problem.object('manual_search_area')
                    else:
                        search_area = Object('manual_search_area', self._problem.user_type('location'))
                        self._problem.add_object(search_area)

                    goal_expression = self._problem.fluent('searched')(searcharea)

                elif goal.get('parameters_json',{}).get('locations',[]):

                    locations = goal.get('parameters_json',{}).get('locations',[])
                    goal_expression_tmp = []
                    for location in goal.get('parameters_json',{}).get('locations',[]):
                        areas_db = self._kb_client.query(collection='area', key='', value='')
                        for area in areas_db:
                            if location in area['name']:
                                goal_expression_tmp.append(self._problem.fluent('searched')(self._problem.object(area['name'])))
                    goal_expression = And(*goal_expression_tmp)

            elif goal_type == 'FIND':

                pass
            elif goal_type == 'LAND':
                goal_expression_tmp = []
                goal_expression_tmp.append(self._problem.fluent('landed')(self._problem.object('myUAV')))
                goal_expression = And(*goal_expression_tmp)

            if goal_expression:
                goal_expressions[goal_id] = goal_expression

        combined_goals =  list(goal_expressions.values())[0]

        if top_level_goal_id is not None:
            combined_goals = self.build_combined_goals([top_level_goal_id], composite_goals, goal_expressions)[0]

        print("Combined Goals computed for UP: ", combined_goals)
        self._problem.add_goal(combined_goals)

    def build_combined_goals(self, child_goals_ids, composite_goals, goal_expressions):
        combined_goals = []
        for goal_id in child_goals_ids:
            if goal_id in composite_goals:

                composite_goal = composite_goals[goal_id]
                new_child_goals_ids = composite_goal.get('goal_condition', {}).get('goal_ids', [])

                if not new_child_goals_ids:
                    continue
                if len(new_child_goals_ids) == 1:
                    combined_goals.append(goal_expressions[new_child_goals_ids[0]])
                    continue

                nodes = self.build_combined_goals(new_child_goals_ids, composite_goals, goal_expressions)

                if composite_goal['type'] == 'AND':
                    combined_goals.append(And(*nodes))
                elif composite_goal['type'] == 'OR':
                    combined_goals.append(Or(*nodes))
                else:
                    continue

            elif goal_id in goal_expressions:
                combined_goals.append(goal_expressions[goal_id])
            else:
                continue

        return combined_goals

    def set_problem_domain(self):
         # === Types ===
        Location = UserType('location')
        UAV = UserType('uav')
        # === Fluents ===
        uav_at = Fluent('uav_at', BoolType(), uav=UAV, location=Location)

        landed = Fluent('landed', BoolType(), uav=UAV)
        taken_off = Fluent('taken_off', BoolType(), uav=UAV)

        visited = Fluent('visited', BoolType(), location=Location)

        connected = Fluent('connected', BoolType(), from_wp=Location, to_wp=Location)
        searched = Fluent('searched', BoolType(), Location=Location)

        # === Problem ===
        problem = Problem(self._problem_name)
        problem.add_fluent(uav_at)
        problem.add_fluent(landed)
        problem.add_fluent(taken_off)
        problem.add_fluent(visited)
        problem.add_fluent(connected)
        problem.add_fluent(searched)

        # === Objects ===
        myUAV = Object('myUAV', UAV)
        problem.add_object(myUAV)

        areas_db = self._kb_client.query(collection='area', key='', value='')
        waypoints = {area['name']: Object(area['name'], Location) for area in areas_db}
        for wp in waypoints.values():
            problem.add_object(wp)

        # === Initial State ===
        problem.set_initial_value(landed(myUAV), True)
        problem.set_initial_value(taken_off(myUAV), False)

        for wp in waypoints.values():
            problem.set_initial_value(uav_at(myUAV, wp), False)
            problem.set_initial_value(searched(wp), False)
            problem.set_initial_value(visited(wp), False)

        problem.set_initial_value(uav_at(myUAV, waypoints['home']), True)

        connections = []
        waypoint_values = list(waypoints.values())
        for i in range(len(waypoint_values)):
            for j in range(len(waypoint_values)):
                if i != j:
                    connections.append((waypoint_values[i], waypoint_values[j]))
        for a, b in connections:
            problem.set_initial_value(connected(a, b), True)

        # === Actions ===

        ## take_off
        take_off = InstantaneousAction('take_off', uav=UAV)
        uav = take_off.parameter('uav')
        take_off.add_precondition(landed(uav))
        take_off.add_effect(landed(uav), False)
        take_off.add_effect(taken_off(uav), True)
        problem.add_action(take_off)

        ## land
        land = InstantaneousAction('land', uav=UAV)
        uav = land.parameter('uav')
        land.add_precondition(taken_off(uav))
        land.add_precondition(uav_at(uav, waypoints['home']))
        land.add_effect(taken_off(uav), False)
        land.add_effect(landed(uav), True)
        problem.add_action(land)

        ## fly
        fly = InstantaneousAction('fly', uav=UAV, from_wp=Location, to_wp=Location)
        uav = fly.parameter('uav')
        from_wp = fly.parameter('from_wp')
        to_wp = fly.parameter('to_wp')
        fly.add_precondition(connected(from_wp, to_wp))
        fly.add_precondition(uav_at(uav, from_wp))
        fly.add_precondition(taken_off(uav))
        fly.add_effect(uav_at(uav, from_wp), False)
        fly.add_effect(uav_at(uav, to_wp), True)
        fly.add_effect(visited(to_wp), True)
        problem.add_action(fly)

        ## search
        search = InstantaneousAction('search_area', uav=UAV, location=Location)
        uav = search.parameter('uav')
        location = search.parameter('location')
        search.add_precondition(uav_at(uav, location))
        search.add_precondition(taken_off(uav))
        search.add_effect(searched(location), True)
        search.add_effect(visited(location), True)
        problem.add_action(search)

        self._problem = problem

    def feedback(self, team_id, feedback_msg):
        pass

    def result(self, team_id, result_msg):
        pass

