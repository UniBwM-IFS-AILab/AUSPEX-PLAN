 #!/usr/bin/env python3
import time
import rclpy
from auspex_planning.planner.task_planners.planner_base import PlannerBase
from unified_planning.shortcuts import *
from unified_planning.model.multi_agent import *
from collections import namedtuple
from unified_planning.io.ma_pddl_writer import MAPDDLWriter
from unified_planning.environment import get_environment
from unified_planning.engines import PlanGenerationResultStatus

from auspex_msgs.msg import Plan, ActionInstance

from auspex_planning.planner.utils.converter import AUSPEXConverter

class UP_MV_Planner(PlannerBase):
    planner_key = 'up_mv_planner'
    def __init__(self, kb_client):

        get_environment().credits_stream = None # disables the credits stream

        self._problem = None
        self._problem_name = "multi_uav_mission"

        self._converter = AUSPEXConverter()

        self._current_plan = None

        self._kb_client = kb_client

        print(f"[INFO]: Initialized Multi UP Problem {self._problem_name}")


    def plan_mission(self, team_id):
        platform_dict = self._kb_client.query(collection='platform', field='', key='team_id', value=team_id)
        if not platform_dict:
            return []

        self.set_problem_domain(platform_dict=platform_dict)
        print(f"[INFO]: Domain set for team {team_id}.")

        print(self._problem)
        if self._problem.goals == []:
            print(f"[INFO]: No goals found for team {team_id}.")
            return []

        print(f"Problem kind: {self._problem.kind}")
        print(f"Problem features: {self._problem.kind.features}")

        """ Requests a plan from the planner. """
        with OneshotPlanner(name='fmap') as planner:
            result = planner.solve(self._problem)
            print(f"Plan generation result: {result.status}")
            print(f"Plan generation time: {result.plan}")
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
        plan_msg.platform_id = platform_dict[0]['platform_id']
        plans = []
        plans.append(plan_msg)
        return plans

    def set_problem_domain(self, platform_dict):
        # === Problem ===
        problem = MultiAgentProblem(self._problem_name)

        # === Agents ===
        agent0 = Agent("simulation_0", problem)
        agent1 = Agent("simulation_1", problem)

        # === Types ===
        Location = UserType('location')

        # === Fluents ===
        uav_at = Fluent('uav_at', BoolType(), location=Location)
        landed = Fluent('landed', BoolType())
        taken_off = Fluent('taken_off', BoolType())
        visited = Fluent('visited', BoolType(), location=Location)

        # Add private fluents per agent
        for ag in [agent0, agent1]:
            ag.add_public_fluent(taken_off)
            ag.add_public_fluent(landed)
            ag.add_public_fluent(uav_at)

        # === Objects ===
        locations = ['home', 'openareas0', 'openareas1', 'openareas2']
        for loc in locations:
            problem.add_object(Object(loc, Location))

        # === Actions ===

        # take_off
        take_off = InstantaneousAction('take_off')
        take_off.add_precondition(landed())
        take_off.add_effect(landed(), False)
        take_off.add_effect(taken_off(), True)

        # land
        land = InstantaneousAction('land')
        land.add_precondition(taken_off())
        land.add_effect(taken_off(), False)
        land.add_effect(landed(), True)

        # fly
        fly = InstantaneousAction('fly', from_wp=Location, to_wp=Location)
        fly.add_precondition(uav_at(fly.parameter('from_wp')))
        fly.add_precondition(taken_off())
        fly.add_effect(uav_at(fly.parameter('from_wp')), False)
        fly.add_effect(uav_at(fly.parameter('to_wp')), True)
        fly.add_effect(visited(fly.parameter('to_wp')), True)

        problem.ma_environment.add_fluent(visited, default_initial_value=False)

        # Add actions to each agent
        for ag in [agent0, agent1]:
            ag.add_action(take_off)
            ag.add_action(land)
            ag.add_action(fly)
            problem.add_agent(ag)

        # === Initial State ===
        for ag in [agent0, agent1]:
            problem.set_initial_value(Dot(ag, landed()), True)
            problem.set_initial_value(Dot(ag, taken_off()), False)
            problem.set_initial_value(Dot(ag, uav_at(problem.object('home'))), True)
            for loc in locations[1:]:
                problem.set_initial_value(Dot(ag, uav_at(problem.object(loc))), False)

        problem.set_initial_value(visited(problem.object('home')), True)
        for loc in locations[1:]:
            problem.set_initial_value(visited(problem.object(loc)), False)

        # === Goals ===
        goals = [
            visited(problem.object('openareas0')),
            visited(problem.object('openareas1')),
            Dot(agent0, landed()),
            Dot(agent1, landed())
        ]

        problem.add_goal(And(*goals))
        self._problem = problem

    def feedback(self, team_id, feedback_msg):
        pass

    def result(self, team_id, result_msg):
        return self.plan_mission(team_id)
