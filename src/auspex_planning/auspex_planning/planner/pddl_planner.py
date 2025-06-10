#!/usr/bin/env python3
import rclpy

from rclpy.action import ActionClient
from rclpy.node import Node

from ament_index_python.packages import get_package_share_directory
from upf_msgs import msg as msgs
from .converter import AUSPEXConverter

from upf4ros2.ros2_interface_reader import ROS2InterfaceReader
from upf4ros2.ros2_interface_writer import ROS2InterfaceWriter

from .planner_base import PlannerBase
from auspex_msgs.msg import ActionInstance, Plan

from upf_msgs.action import (
    PDDLPlanOneShot
)

from upf_msgs.srv import (
    GetProblem,
    SetInitialValue,
)
from upf_msgs.srv import PDDLPlanOneShot as PDDLPlanOneShotSrv

class PDDL_Planner(PlannerBase, Node):
    planner_key = 'pddl_planner'
    """
    Ros2 node used to create a plan

    """

    def __init__(self, kb_client):
        """
        Constructor method
        """
        super().__init__('PDDL_Planner_Default')
        self._kb_client = kb_client

        self._converter = AUSPEXConverter()

        self._ros2_interface_writer = ROS2InterfaceWriter()
        self._ros2_interface_reader = ROS2InterfaceReader()

        #####################################
        #           static init             #
        #####################################

        # declare params
        self.declare_parameter('domain', '/pddl/domain.pddl')
        self.declare_parameter('problem', '/pddl/problem.pddl')

        self._domain = self.get_parameter('domain')
        self._problem = self.get_parameter('problem')
        self._platform_id = "simulation_0"

        # stores the problem instance in UPF format
        self._problem = None
        # maps all availabe action names to their UPF representation
        self._actions = {}
        # maps all availabe object names to their UPF representation
        self._objects = {}
        # maps all availabe fluent names to their UPF representation
        self._fluents = {}
        # stores the plan as a sequence of UPF actions
        self._plan = []

        self.sub_node = rclpy.create_node('sub_node_ppdl_planner_' + self._platform_id, use_global_arguments=False)


        self._plan_pddl_one_shot_client = ActionClient(self, PDDLPlanOneShot, 'upf4ros2/action/planOneShotPDDL')

        # create up4ros service clients
        self._get_problem = self.sub_node.create_client(GetProblem, 'upf4ros2/srv/get_problem')
        self._set_initial_value = self.sub_node.create_client(SetInitialValue, 'upf4ros2/srv/set_initial_value')
        self._plan_pddl_one_shot_client_srv = self.sub_node.create_client(PDDLPlanOneShotSrv, 'upf4ros2/srv/planOneShotPDDL')

    def set_initial_value(self, fluent, object, value_fluent):
        """
        Updates the initial value of a problem in the up4ros problem manager

        :param fluent: the fluent to be updated (e.g. fluent: drone_at)
        :param object: the object(s) for the fluent (e.g. object: waypoint)
        :param value_fluent: the new value (of type <object>) assigned to the fluent (e.g. forest1)
        """
        srv = SetInitialValue.Request()
        srv.expression = self._ros2_interface_writer.convert(fluent(*object))

        item = msgs.ExpressionItem()
        item.atom.append(msgs.Atom())
        item.atom[0].boolean_atom.append(value_fluent)
        item.type = 'up:bool'
        item.kind = msgs.ExpressionItem.CONSTANT
        value = msgs.Expression()
        value.expressions.append(item)
        value.level.append(0)

        srv.value = value

        while not self._set_initial_value.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for _set_initial_value to come online...')

        future = self._set_initial_value.call_async(srv)
        rclpy.spin_until_future_complete(self.sub_node, future)

    def feedback(self, team_id, feedback_msg):
        pass

    def result(self, team_id, result_msg):
        pass

    def update_state(self, state):
        """
        Updates the problem with all the effects from an executed action

        :param upf_msgs/Action action: the executed action with it's associated effects
        :param parameters: the parameters (i.e. objects) of the executed action
        """
        action, parameters = state
        action = self.getActionsOfProblem(action.action_name)
        self.get_logger().info("Updating initial state")
        # maps identifiers of action parameters to their concrete instance
        # e.g. x : myuav, y : urbanArea, z : home
        paramMap = {action.parameters[i].name : parameters[i]  for i in range(len(parameters))}
        for effect in action.effects:
            fluent = effect.fluent.fluent()
            """
            self.get_logger().info(f"current effect: {effect}")
            self.get_logger().info(f"current effect.fluent.args: {effect.fluent.args}")

            # calling x.parameter in any way on myroute crashes python --> File "/home/companion/.local/lib/python3.8/site-packages/unified_planning/model/fnode.py", line 205, in parameter --> assert self.is_parameter_exp() --> AssertionError
            # exist and forall are variables (fnode.py", line 213)

            for x in effect.fluent.args:
                self.get_logger().info(f"arg_dir: {dir(x)}")
                try:
                    self.get_logger().info(f"arg___repr__: {x.__repr__()}")
                except Exception as error:
                    traceback.print_exc()
                    self.get_logger().info(f"arg.parameter() errors: {type(error).__name__}")
            """

            # myroute is not in parameters and therefore not in paramMap (as it is derived from exist keyword); also it seems z is not in effect.fluent.args but we dont need z, as the condition is not checked anyway
            # luckily we won't need paramMap for exist objects, as the variable ?r was already resolved to 'myroute'
            # so lets assume that all elements of effect.fluent.args that are not contained in paramMap are already resolved exist objects
            # it is still important to keep the order of elements in effect.fluent.args so lets map the list using if/else list comprehension
            # https://stackoverflow.com/questions/4260280/if-else-in-a-list-comprehension

            # there seems to be a bug in UPF where fluent.fluent() unpacks the arguments from the predicate declaration instead of  -> workaround: access the arguments directly with effect.fluent.args
            fluent_signature = [self._objects[paramMap[x_name]] if (x_name := x.__repr__()) in paramMap else self._objects[x_name] for x in effect.fluent.args]

            value = effect.value.constant_value()
            self.set_initial_value(fluent, fluent_signature, value)

    def get_problem(self):
        """
        Retrieves the current state of the problem from the up4ros problem manager

        :returns: problem: The current state of the problem
        :rtype: upf_msgs/Problem
        """
        srv = GetProblem.Request()

        while not self._get_problem.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for _get_problem to come online...')

        future = self._get_problem.call_async(srv)
        rclpy.spin_until_future_complete(self.sub_node, future)
        problem = self._ros2_interface_reader.convert(future.result().problem)
        return problem

    def get_plan_from_pddl(self):
        """
        This function reads in the pddl domain file and pddl problem file as specified in the init function.
        These files are then passed to upf4ros2 main where a plan is calculated. The resulting plan (along with actions, objects, fluents)
        are then saved in this node, so the plan can be executed.

        """
        self.get_logger().info('Planning...')
        srv = PDDLPlanOneShotSrv.Request()
        srv.plan_request.mode = msgs.PDDLPlanRequest.FILE
        self._problem = self.get_parameter('problem')
        srv.plan_request.domain = (get_package_share_directory('auspex_planning')
                                        + str(self._domain.value))
        srv.plan_request.problem = (get_package_share_directory('auspex_planning')
                                        + str(self._problem.value))

        while not self._plan_pddl_one_shot_client_srv.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for _plan_pddl_one_shot_client_srv to come online...')
        future = self._plan_pddl_one_shot_client_srv.call_async(srv)

        #waits till plan computed
        rclpy.spin_until_future_complete(self.sub_node, future)


        plan_result = future.result().plan_result
        self._problem = self.get_problem()

        self._objects = {self._problem.all_objects[i].name : self._problem.all_objects[i] for i in range(len(self._problem.all_objects))}
        self._fluents = {self._problem.fluents[i].name : self._problem.fluents[i] for i in range(len(self._problem.fluents))}
        self._actions = {self._problem.actions[i].name : self._problem.actions[i] for i in range(len(self._problem.actions))}
        self._plan = plan_result.plan.actions
        return self._plan

    def getPlan(self):
        """
        Returns the current plan.
        """
        return self._plan


    def getActionsOfProblem(self, action):
        """
        Returns the current plan.
        """
        return self._actions[action]

    def plan_mission(self, team_id):
        """
        Launch a computation of the plan
        """
        vhcl_dict = self._kb_client.query('platform', 'platform_id', 'team_id', team_id)
        if not vhcl_dict:
            return []
        platform_id = vhcl_dict[0]['platform_id']

        self.get_logger().info(f"[INFO]: PDDL Planner Selected for team : {team_id}")

        plans = []
        plan_msg = Plan()
        plan_msg.tasks = self._converter.convert_plan_upf2auspex(self.get_plan_from_pddl())
        plan_msg.platform_id = platform_id
        plan_msg.priority = 0
        plan_msg.team_id = team_id
        self.get_logger().info(f"[INFO]: Found Plan")
        plans.append(plan_msg)
        return plans
