 #!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from auspex_planning.planner.task_planners.planner_base import PlannerBase
from rclpy.executors import SingleThreadedExecutor
from upf_msgs.srv import SetProblem, AddAction, SetInitialValue, AddGoal,AddObject, GetProblem, AddFluent, NewProblem
from upf_msgs.msg import Expression, ExpressionItem, Atom, Goal,TypeDeclaration, Problem
from auspex_planning.planner.utils.converter import AUSPEXConverter

from upf4ros2.ros2_interface_reader import ROS2InterfaceReader
from upf4ros2.ros2_interface_writer import ROS2InterfaceWriter


from unified_planning import model
from unified_planning.shortcuts import UserType, BoolType, Forall, And, Equals, TRUE, Not
from auspex_msgs.msg import Plan, ActionInstance
from upf_msgs.action import PlanOneShot

ACTION_BASED = 0
SIMPLE_NUMERIC_PLANNING = 30


class UPF4ROS2_Planner(PlannerBase, Node):
    planner_key = 'upf4ros2_planner'
    def __init__(self, kb_client):
        super().__init__('upf4ros2_planner')

        self._problem_name = ''
        self._plan_result = {}
        self._kb_client = kb_client
        self._up_executor = SingleThreadedExecutor()
        self._current_plan = None

        self._ros2_interface_writer = ROS2InterfaceWriter()
        self._ros2_interface_reader = ROS2InterfaceReader()


        self._converter = AUSPEXConverter()

        # Create service clients
        self._get_problem = self.create_client(GetProblem, '/upf4ros2/srv/get_problem')
        self._new_problem = self.create_client(NewProblem, '/upf4ros2/srv/new_problem')

        self._add_fluent = self.create_client(AddFluent, '/upf4ros2/srv/add_fluent')
        self._add_object = self.create_client(AddObject, '/upf4ros2/srv/add_object')
        self._add_action = self.create_client(AddAction, '/upf4ros2/srv/add_action')
        self._add_goal = self.create_client(AddGoal, '/upf4ros2/add_goal')

        self._set_initial_value = self.create_client(SetInitialValue, '/upf4ros2/srv/set_initial_value')
        self._set_problem = self.create_client(SetProblem, '/upf4ros2/srv/set_problem')

        # Create an action client for planning
        self._plan_one_shot_client = ActionClient(self, PlanOneShot, '/upf4ros2/action/planOneShot')

        self.init_problem()
        self.get_logger().info(f"[INFO]: Initialized UP Problem {self._problem_name}")

    def plan_result_feedback(self, feedback_msg):
        self._current_plan = feedback_msg.feedback.plan_result

    def plan_mission(self, team_id):
        vhcl_dict = self._kb_client.query('platform', 'platform_id', 'team_id', team_id)
        if not vhcl_dict:
            return []
        platform_id = vhcl_dict[0]['platform_id']

        """ Requests a plan from the planner. """
        self.get_logger().info(f"[INFO]: UP Planner Selected for team : {team_id}")
        problem = self.get_problem()
        req = PlanOneShot.Goal()
        req.plan_request.problem = problem
        future = self._plan_one_shot_client.send_goal_async(req, feedback_callback=self.plan_result_feedback)
        rclpy.spin_until_future_complete(self, future, self._up_executor)
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Plan request rejected...')
            return []

        future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, future, self._up_executor)
        if future.result().result.success:
            if self._current_plan == None:
                return []
        self.get_logger().info('Plan found.')

        plans = []
        plan_msg = Plan()
        plan_msg.tasks = self._converter.convert_plan_upf2auspex(self._current_plan.plan.actions)
        plan_msg.platform_id = platform_id
        plan_msg.priority = 0
        plan_msg.team_id = team_id
        plans.append(plan_msg)
        return plans

    def new_problem(self, problem_name):
        req = NewProblem.Request()
        req.problem_name = problem_name
        self._new_problem.wait_for_service()
        future = self._new_problem.call_async(req)
        rclpy.spin_until_future_complete(self, future, self._up_executor)
        self._problem_name = problem_name
        self.get_logger().info(f'Created problem: {problem_name}')

    def get_problem(self):
        req = GetProblem.Request()
        req.problem_name = self._problem_name
        self._get_problem.wait_for_service()
        future = self._get_problem.call_async(req)
        rclpy.spin_until_future_complete(self, future, self._up_executor)
        self.res = future.result()
        return self.res.problem

    def add_fluent(self, problem, fluent_name, user_type):
        fluent = model.Fluent(fluent_name, BoolType(), object=user_type)
        req = AddFluent.Request()
        req.problem_name = self._problem_name
        req.fluent = self._ros2_interface_writer.convert(fluent, problem)

        item = ExpressionItem()
        item.atom.append(Atom())
        item.atom[0].boolean_atom.append(False)
        item.type = 'up:bool'
        item.kind = ExpressionItem.CONSTANT
        value = Expression()
        value.expressions.append(item)
        value.level.append(0)
        req.default_value = value

        self._add_fluent.wait_for_service()
        future = self._add_fluent.call_async(req)
        rclpy.spin_until_future_complete(self, future, self._up_executor)
        self.get_logger().info(f'Added fluent: {fluent_name}')
        return fluent

    def add_object(self, object_name, user_type):
        obj = model.Object(object_name, user_type)
        req = AddObject.Request()
        req.problem_name = self._problem_name
        req.object = self._ros2_interface_writer.convert(obj)
        self._add_object.wait_for_service()
        future = self._add_object.call_async(req)
        rclpy.spin_until_future_complete(self, future, self._up_executor)
        self.get_logger().info(f'Added object: {object_name}')
        return obj

    def set_initial_value(self, fluent, obj, value_bool):
        req = SetInitialValue.Request()
        req.problem_name = self._problem_name
        req.expression = self._ros2_interface_writer.convert(fluent(obj))
        item = ExpressionItem()
        item.atom.append(Atom())
        item.atom[0].boolean_atom.append(value_bool)
        item.type = 'up:bool'
        item.kind = ExpressionItem.CONSTANT
        value = Expression()
        value.expressions.append(item)
        value.level.append(0)
        req.value = value
        self._set_initial_value.wait_for_service()
        future = self._set_initial_value.call_async(req)
        rclpy.spin_until_future_complete(self, future, self._up_executor)
        self.get_logger().info(f'Set initial value of {fluent.name}({obj.name}) to {value_bool}')

    def add_action(self, action):
        req = AddAction.Request()
        req.problem_name = self._problem_name
        req.action = self._ros2_interface_writer.convert(action)
        self._add_action.wait_for_service()
        future = self._add_action.call_async(req)
        rclpy.spin_until_future_complete(self, future, self._up_executor)
        self.get_logger().info(f'Added action: {action.name}')

    def add_goal(self, goal):
        req = AddGoal.Request()
        req.problem_name = self._problem_name
        up_goal = Goal()
        up_goal.goal = self._ros2_interface_writer.convert(goal)
        req.goal.append(up_goal)
        self._add_goal.wait_for_service()
        future = self._add_goal.call_async(req)
        rclpy.spin_until_future_complete(self, future, self._up_executor)
        self.get_logger().info('Added goal.')

    def init_problem(self):
        self.new_problem('uav_problem')
        problem_msg = self.get_problem()
        problem = self._ros2_interface_reader.convert(problem_msg) # type unified_planning::Problem
        location_type = UserType('location')
        uav_type = UserType('uav')

        uav_at = self.add_fluent(problem=problem, fluent_name='uav_at', user_type=location_type)
        landed_state = self.add_fluent(problem=problem, fluent_name='landed', user_type=uav_type)
        taken_off_state = self.add_fluent(problem=problem, fluent_name='taken_off', user_type=uav_type)
        visited_wp = self.add_fluent(problem=problem, fluent_name='visited', user_type=location_type)

        myUAV_obj = self.add_object('myUAV', uav_type)
        home = self.add_object('home', location_type )
        openareas2 = self.add_object('openareas2', location_type)
        openareas3 = self.add_object('openareas3', location_type)
        openareas4 = self.add_object('openareas4', location_type)
        openareas5 = self.add_object('openareas5', location_type)
        openareas6 = self.add_object('openareas6', location_type)
        openareas7 = self.add_object('openareas7', location_type)

        self.set_initial_value(landed_state, myUAV_obj, True)
        self.set_initial_value(uav_at, home, True)
        self.set_initial_value(taken_off_state, myUAV_obj, False)

        fly = model.InstantaneousAction('fly', from_wp=location_type , to_wp=location_type )
        from_wp = fly.parameter('from_wp')
        to_wp = fly.parameter('to_wp')
        fly.add_precondition(uav_at(from_wp))
        fly.add_precondition(taken_off_state(myUAV_obj))
        fly.add_effect(uav_at(from_wp), False)
        fly.add_effect(uav_at(to_wp), True)
        fly.add_effect(visited_wp(to_wp), True)
        self.add_action(fly)

        take_off = model.InstantaneousAction('take_off')
        take_off.add_precondition(landed_state(myUAV_obj))
        take_off.add_effect(landed_state(myUAV_obj), False)
        take_off.add_effect(taken_off_state(myUAV_obj), True)
        self.add_action(take_off)

        land = model.InstantaneousAction('land')
        land.add_precondition(taken_off_state(myUAV_obj))
        land.add_effect(taken_off_state(myUAV_obj), False)
        land.add_effect(landed_state(myUAV_obj), True)
        self.add_action(land)

        goal_expressions = []

        # 1. UAV has visited all waypoints
        for wp_obj in [openareas2, openareas3, openareas4, openareas5, openareas6, openareas7]:
            goal_expressions.append(visited_wp(wp_obj))

        goal_expressions.append(landed_state(myUAV_obj))
        goal_expressions.append(uav_at(home))
        full_goal = And(*goal_expressions)

        self.add_goal(full_goal)

    def feedback(self, team_id, feedback_msg):
        pass

    def result(self, team_id, result_msg):
        pass

