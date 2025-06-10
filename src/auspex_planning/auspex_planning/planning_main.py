#!/usr/bin/env python3
import rclpy
import time
import json
from enum import Enum
from action_msgs.msg import GoalStatus

from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from .planner.mock_planner import PlannerBase # needed for create_planner_from_key
from .planner.mock_planner import Mock_Planner # needed for create_planner_from_key
from .planner.pddl_planner import PDDL_Planner # needed for create_planner_from_key
from .planner.pattern_planner import PatternPlanner # needed for create_planner_from_key
from .planner.llm_planner import LLM_Planner # needed for create_planner_from_key
from .planner.mvrp_alns_planner import MVPR_ALNS_Planner # needed for create_planner_from_key
from .planner.up_planner import UP_Planner # needed for create_planner_from_key
from .planner.up_mv_planner import UP_MV_Planner # needed for create_planner_from_key
from .planner.up4ros2_planner import UPF4ROS2_Planner # needed for create_planner_from_key

from .planner.path_planner import Path_Planner

from .planner.goal_utils.goal_handler import GoalHandler
from auspex_msgs.msg import ExecutorCommand, PlannerCommand, PlanStatus

from auspex_planning.interfaces.monitor_interface import MonitorInterface
from auspex_db_client.kb_client import KB_Client
from .planner.converter import enum_to_str
import rosidl_runtime_py

from auspex_msgs.msg import UserCommand
from auspex_msgs.msg import Plan

class EXECUTION_MODE(Enum):
    HITL = 1 #Human in the loop
    AUTO = 2

def create_planner_from_key(kb_client, planner_key: str):
    """
    Iterates over all subclasses of PlannerBase to find one
    with a matching planner_key and returns an instance.
    """
    for planner_cls in PlannerBase.__subclasses__():
        if getattr(planner_cls, "planner_key", None) == planner_key:
            return planner_cls(kb_client)
    return None

class PlanningMain(Node):
    def __init__(self, kb_client=None):
        super().__init__('main_planner_node')

        self._planner_command_subscriber = self.create_subscription(UserCommand,'planner_command', self.planner_command_callback,10)

        self._monitor_interfaces = {}

        self._kb_client = kb_client

        self._EXECUTION_MODE = EXECUTION_MODE.AUTO

        self._plan_id = 0

        self._team_planners = {}

        self._path_planner = Path_Planner(self._kb_client)

        self._default_planner = Mock_Planner(self._kb_client)

        self.get_logger().info('Planner Main Ready...')


    def execute_command(self, team_id: str, execution_command):
        """
        Calls the respective executor monitor for the team.
        """
        if not(team_id in self._monitor_interfaces):
            self._monitor_interfaces[team_id] = MonitorInterface(self, team_id, self.feedback_callback, self.result_callback, self.cancel_done_callback)
        self._monitor_interfaces[team_id].send_command(execution_command)


    def result_callback(self, monitor_interface, msg):
        self.get_logger().info(f'Result: {enum_to_str(PlannerCommand, msg.command)}')

        team_id = monitor_interface._team_id

        if msg.info.success == True:
            self.get_logger().info("Plan for team: " + team_id+" succeeded")
        else:
            self.get_logger().info("Plan for team: " + team_id + " returned")

        if msg.command == PlannerCommand.UPDATEPLAN:
            # Get the new plan
            new_plans = self.get_planning_engine(team_id).result(team_id, msg)

            if new_plans:
                self.insert_plan_to_KB(new_plans)
                if self._EXECUTION_MODE == EXECUTION_MODE.AUTO:
                    self.execute_command(team_id, ExecutorCommand.EXECUTE)


    def feedback_callback(self, monitor_interface, feedback_msg):
        self.get_logger().info('Received feedback')
        team_id = monitor_interface._team_id
        self.get_planning_engine(team_id).feedback(team_id, feedback_msg)


    def cancel_done_callback(self, monitor_interface, msg):
        self.get_logger().info("Canceled Plan of team"+ monitor_interface._team_id)


    def cancel(self, team_id):
        if team_id.lower() != "all":
            self.get_logger().info("Got Cancel "+team_id+" Command")
            if team_id not in self._monitor_interfaces:
                self.get_logger().info(f"[ERROR]: No monitor interface found for team {team_id}. Cannot cancel plan.")
                return
            self._monitor_interfaces[team_id].cancel_plan()
        elif team_id.lower() == "all":
            self.get_logger().info("Got Cancel All Command")
            for interface in self._monitor_interfaces.values():
                self.get_logger().info("Trying to cancel goals for team: "+interface._team_id+".")
                interface.cancel_plan()
        else:
            return


    def get_planning_engine(self, team_id):
        planner_list = self._kb_client.query(collection='config', field='planner',key='team_id',value=team_id)
        if not planner_list:
            self.get_logger().info('[ERROR]: Empty planner queried from Knowledge Base.')
            return None
        planner_key = planner_list[0]['planner']

        if team_id in self._team_planners:
            if planner_key == self._team_planners[team_id].planner_key:
                return self._team_planners[team_id]

        planner_instance = create_planner_from_key(self._kb_client, planner_key)
        if planner_instance is None:
            self.get_logger().info(f'[ERROR]: No planner found with key "{planner_key}".')
            return None

        self._team_planners[team_id] = planner_instance
        return planner_instance


    def insert_plan_to_KB(self, plans):
        if plans == None or plans == []:
            self.get_logger().info("[INFO]: Empty plan received, not inserting into Knowledge Base.")
            return

        for plan in plans:
            plan.plan_id = self._plan_id
            plan.status =  enum_to_str(PlanStatus, PlanStatus.INACTIVE)
            self._plan_id += 1
            plan_dict = rosidl_runtime_py.convert.message_to_ordereddict(plan)
            plan_json = ''
            try:
                plan_json = json.dumps(plan_dict, indent=4)
            except Exception as e:
                self.get_logger().info("[ERROR]: not a valid json")

            self.get_logger().info(f"[INFO]: Inserting plan with ID {plan.plan_id} into Knowledge base.")
            self._kb_client.write(collection='plan', entity=plan_json, key='plan_id', value=str(plan.plan_id))


    def plan(self, team_id):
        engine = self.get_planning_engine(team_id)
        if engine == None:
            self.get_logger().info("[ERROR]: No planner selected.")
            return
        task_plans = engine.plan_mission(team_id)

        if not task_plans:
            return

        self._kb_client.update(collection='goal', key='team_id', value=team_id, field='status', new_value='PLANNED')

        path_plans = self._path_planner.plan_path(team_id, task_plans)

        self.insert_plan_to_KB(path_plans)


    def planner_command_callback(self, user_command_msg):
        usr_cmd = user_command_msg.user_command
        team_id = user_command_msg.team_id
        platform_id = user_command_msg.platform_id # Not used yet

        DEFAULT_TEAM_ID = 'drone_team'

        if team_id == "":
            team_id = DEFAULT_TEAM_ID

        if usr_cmd != 1 and team_id == "all":
            self.get_logger().info("[ERROR]: Team ID 'all' only defined for cancel.")
            return

        try:
            match usr_cmd:
                case 1: # USER_CANCEL
                    self.cancel(team_id)
                    #TODO get drones to team id and publish empty plan
                case 2: # USER_PAUSE
                    self.execute_command(team_id, ExecutorCommand.PAUSE)
                case 3: # USER_RESUME
                    self.execute_command(team_id, ExecutorCommand.CONTINUE)
                case 4: # USER_START
                    self._EXECUTION_MODE = EXECUTION_MODE.AUTO
                    self.plan(team_id)
                    time.sleep(1)
                    self.execute_command(team_id, ExecutorCommand.EXECUTE)
                case 5: # USER_ACCEPT
                    self._EXECUTION_MODE = EXECUTION_MODE.HITL
                    self.execute_command(team_id, ExecutorCommand.EXECUTE)
                case 6: # USER_REJECT
                    #TODO publish empty plan to KB
                    pass
                case 7: # USER_PLAN
                    self._EXECUTION_MODE = EXECUTION_MODE.HITL
                    self.plan(team_id)
                case 8: # USER_RTH
                    self.cancel(team_id)
                    time.sleep(4)
                    task_plans = self._default_planner.plan_rth(team_id)
                    path_plans = self._path_planner.plan_path(task_plans)
                    self.insert_plan_to_KB(path_plans)
                    time.sleep(1)
                    self.execute_command(team_id, ExecutorCommand.EXECUTE)
                case 9: # USER_TERMINATE
                    self.cancel(team_id)
                    # TODO Publish to DRONE to terminate
                case _ : # DEFAULT CASE
                    self.get_logger().info("[ERROR]: Unknown command.")
        except Exception as e:
            self.get_logger().error(f"An unexpected error occurred during Command Callback: {e}")


def main():
    rclpy.init(args=None)

    mte = MultiThreadedExecutor()
    kb_client = KB_Client(node_name_prefix='planning_main_')

    goal_handler_node = GoalHandler(kb_client)
    planning_main_node = PlanningMain(kb_client)

    mte.add_node(goal_handler_node)
    mte.add_node(planning_main_node)
    mte.spin()

    """
    Destroying nodes
    """
    planning_main_node.destroy_node()
    goal_handler_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
