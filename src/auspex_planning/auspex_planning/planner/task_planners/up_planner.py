 #!/usr/bin/env python3
import time
import rclpy
from auspex_planning.planner.task_planners.planner_base import PlannerBase
from unified_planning.shortcuts import *
from unified_planning.model import Problem
from unified_planning.engines import PlanGenerationResultStatus
from auspex_msgs.msg import Plan, ActionInstance

from unified_planning.model.metrics import MinimizeSequentialPlanLength
from auspex_planning.planner.utils.converter import AUSPEXConverter

class UP_Planner(PlannerBase):
    planner_key = 'up_planner'
    def __init__(self, kb_client):
        up.shortcuts.get_environment().credits_stream = None # disables the credits stream
        self._problem = None
        self._problem_name = "uav_mission"

        self._converter = AUSPEXConverter()

        self._current_plan = None

        self._kb_client = kb_client

        print(f"[INFO]: Initialized UP Problem {self._problem_name}")


    def plan_mission(self, team_id):
        vhcl_dict = self._kb_client.query(collection='platform', field='', key='team_id', value=team_id)
        if not vhcl_dict:
            return []

        self.set_problem_domain(vhcl_dict)
        self.set_problem_instance(team_id)

        if self._problem.goals == []:
            print(f"[INFO]: No goals found for team {team_id}.")
            return []

        """ Requests a plan from the planner. """
        with OneshotPlanner(problem_kind=self._problem.kind, optimality_guarantee=PlanGenerationResultStatus.SOLVED_OPTIMALLY) as planner:
            result = planner.solve(self._problem)
            if result.status == PlanGenerationResultStatus.SOLVED_OPTIMALLY or result.status == PlanGenerationResultStatus.SOLVED_SATISFICING:
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
        plan_msg.platform_id = vhcl_dict[0]['platform_id']
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
            print("Processing goal:", goal_id, "of type:", goal_type)
            if goal_type == 'SEARCH':

                goal_expression_tmp = []
                if goal.get('parameters_json',{}).get('search_area',{}).get('points',[]):

                    search_area = self._problem.object('manual_search_area_' + team_id)
                    self._problem.set_initial_value(self._problem.fluent('uav_at')(self._problem.object('uav1'), search_area), False)
                    self._problem.set_initial_value(self._problem.fluent('searched')(search_area), False)
                    #self._problem.set_initial_value(self._problem.fluent('visited')(search_area), False)
                    goal_expression_tmp.append(self._problem.fluent('searched')(search_area))

                elif goal.get('parameters_json',{}).get('locations',[]):
                    locations = goal.get('parameters_json',{}).get('locations',[])
                    for location in goal.get('parameters_json',{}).get('locations',[]):
                        areas_db = self._kb_client.query(collection='area', key='', value='')
                        for area in areas_db:
                            if location in area['name']:
                                goal_expression_tmp.append(self._problem.fluent('searched')(self._problem.object(area['name'])))

                if goal.get('parameters_json',{}).get('pois',[]):

                    goal_expression_tmp.append(self._problem.fluent('searched')(self._problem.object("pois")))
                    self._problem.set_initial_value(self._problem.fluent('searched')(self._problem.object("pois")), False)


                if goal.get('parameters_json',{}).get('last_known_position',[]) and goal.get('parameters_json',{}).get('last_known_position',[]) == 'true':

                    last_known_position = self._problem.object('last_known_position')
                    self._problem.set_initial_value(self._problem.fluent('visited')(self._problem.object("last_known_position")), False)
                    goal_expression_tmp.append(self._problem.fluent('visited')(last_known_position))

                if goal_expression_tmp:
                    goal_expression = And(*goal_expression_tmp)

            elif goal_type == 'FIND':
                goal_expression_tmp = []
                if goal.get('parameters_json',{}).get('objects',[]):
                    objects = goal.get('parameters_json',{}).get('objects',[])
                    for obj in objects:
                        object_up = None
                        if self._problem.has_object(obj):
                            object_up = self._problem.object(obj)
                        else:
                            object_up = Object(obj, self._problem.user_type('object'))
                            self._problem.add_object(object_up)
                        goal_expression_tmp.append(self._problem.fluent('found')(object_up))
                        goal_expression_tmp.append(self._problem.fluent('confirmed')(object_up))

                        obj_prio = self._kb_client.query(collection='object', field='priority', key='detection_class', value=obj)
                        obj_conf = self._kb_client.query(collection='object', field='confidence', key='detection_class', value=obj)

                        print(f'Object Prio:{obj_prio}')
                        print(f'Object Conf:{obj_conf}')

                        if obj_conf and obj_conf == 0.6:
                            self._problem.set_initial_value(self._problem.fluent('found')(object_up), True)
                            self._problem.set_initial_value(self._problem.fluent('confirmed')(object_up), False)
                        elif obj_prio and obj_prio == 10:
                            self._problem.set_initial_value(self._problem.fluent('found')(object_up), True)
                            self._problem.set_initial_value(self._problem.fluent('confirmed')(object_up), True)
                        else:
                            self._problem.set_initial_value(self._problem.fluent('found')(object_up), False)
                            self._problem.set_initial_value(self._problem.fluent('confirmed')(object_up), False)

                if goal_expression_tmp:
                    goal_expression = And(*goal_expression_tmp)

            elif goal_type == 'LAND':
                goal_expression_tmp = []
                goal_expression_tmp.append(self._problem.fluent('landed')(self._problem.object('uav1')))
                goal_expression = And(*goal_expression_tmp)

            if goal_expression:
                goal_expressions[goal_id] = goal_expression

        if goal_expressions == {}:
            print(f"[INFO]: No goal expressions found for team {team_id}.")
            return []

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

    def set_problem_domain(self, vhcl_dict):
         # === Types ===
        Location = UserType('location')
        object = UserType('object')
        UAV = UserType('uav')

        # === Problem ===
        problem = Problem(self._problem_name)

        current_vhcls = []

        # === Objects ===
        uav1 = Object('uav1', UAV)
        problem.add_object(uav1)
        current_vhcls.append(uav1)

        areas_db = self._kb_client.query(collection='area', key='', value='')
        waypoints = {}
        for area in areas_db:
            if area['name'].startswith('home_'):
                waypoints['home'] = Object('home', Location)
            else:
                waypoints[area['name']] = Object(area['name'], Location)
        waypoints['pois'] = Object('pois', Location)
        waypoints['last_known_position'] = Object('last_known_position', Location)
        for wp in waypoints.values():
            problem.add_object(wp)

        # === Fluents ===
        uav_at = Fluent('uav_at', BoolType(), uav=UAV, location=Location)
        landed = Fluent('landed', BoolType(), uav=UAV)
        taken_off = Fluent('taken_off', BoolType(), uav=UAV)
        visited = Fluent('visited', BoolType(), location=Location)
        searched = Fluent('searched', BoolType(), Location=Location)
        found = Fluent('found', BoolType(), object=object)
        confirmed = Fluent('confirmed', BoolType(), object=object)

        problem.add_fluent(found)
        problem.add_fluent(uav_at)
        problem.add_fluent(landed)
        problem.add_fluent(taken_off)
        problem.add_fluent(visited)
        problem.add_fluent(searched)
        problem.add_fluent(confirmed)

        # === Initial State ===
        for index, vhcl in enumerate(current_vhcls):
            if vhcl_dict[index]['platform_status'] == 'LANDED':
                problem.set_initial_value(landed(vhcl), True)
                problem.set_initial_value(taken_off(vhcl), False)
            else:
                problem.set_initial_value(landed(vhcl), False)
                problem.set_initial_value(taken_off(vhcl), True)

        for wp in waypoints.values():
            problem.set_initial_value(uav_at(uav1, wp), False)
            problem.set_initial_value(searched(wp), False)
            problem.set_initial_value(visited(wp), False)

        problem.set_initial_value(searched(waypoints['pois']), True)
        problem.set_initial_value(searched(waypoints['last_known_position']), True)
        problem.set_initial_value(uav_at(uav1, waypoints['home']), True)

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
        fly.add_precondition(uav_at(uav, from_wp))
        fly.add_precondition(taken_off(uav))
        fly.add_effect(uav_at(uav, from_wp), False)
        fly.add_effect(uav_at(uav, to_wp), True)
        fly.add_effect(visited(to_wp), True)
        problem.add_action(fly)

        ## search
        search = InstantaneousAction('search_area', uav=UAV, from_wp=Location, to_wp=Location)
        uav = search.parameter('uav')
        from_wp = search.parameter('from_wp')
        to_wp = search.parameter('to_wp')

        search.add_precondition(uav_at(uav, from_wp))
        search.add_precondition(taken_off(uav))
        search.add_precondition(searched(waypoints['pois']))
        search.add_precondition(searched(waypoints['last_known_position']))
        search.add_effect(searched(to_wp), True)
        search.add_effect(uav_at(uav, from_wp), False)
        search.add_effect(uav_at(uav, to_wp), True)
        problem.add_action(search)

        ## search poi
        search_poi = InstantaneousAction('search_poi', uav=UAV, from_wp=Location, to_wp=Location)
        uav = search_poi.parameter('uav')
        from_wp = search_poi.parameter('from_wp')
        to_wp = search_poi.parameter('to_wp')
        search_poi.add_precondition(uav_at(uav, from_wp))
        search_poi.add_precondition(taken_off(uav))
        search_poi.add_precondition(Equals(to_wp, waypoints['pois']))
        search_poi.add_effect(searched(to_wp), True)
        search_poi.add_effect(uav_at(uav, from_wp), False)
        search_poi.add_effect(uav_at(uav, to_wp), True)
        problem.add_action(search_poi)

        confirm = InstantaneousAction('confirm', uav=UAV, object=object)
        object = confirm.parameter('object')
        uav = confirm.parameter('uav')
        confirm.add_precondition(found(object))
        confirm.add_precondition(taken_off(uav))
        confirm.add_effect(confirmed(object), True)
        problem.add_action(confirm)

        problem.add_quality_metric(MinimizeSequentialPlanLength())

        self._problem = problem

    def feedback(self, team_id, feedback_msg):
        pass

    def result(self, team_id, result_msg):
        return self.plan_mission(team_id)

