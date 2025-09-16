#!/usr/bin/env python3
import rclpy
import time
import json
from enum import Enum
from action_msgs.msg import GoalStatus

from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from .planner.task_planners.planner_base import PlannerBase # needed for create_planner_from_key
from .planner.task_planners.mock_planner import Mock_Planner # needed for create_planner_from_key
from .planner.task_planners.pddl_planner import PDDL_Planner # needed for create_planner_from_key
from .planner.task_planners.pattern_planner import PatternPlanner # needed for create_planner_from_key
from .planner.task_planners.llm_planner import LLM_Planner # needed for create_planner_from_key
from .planner.task_planners.mvrp_alns_planner import MVPR_ALNS_Planner # needed for create_planner_from_key
from .planner.task_planners.up_planner import UP_Planner # needed for create_planner_from_key
from .planner.task_planners.up_mv_planner import UP_MV_Planner # needed for create_planner_from_key
from .planner.task_planners.up4ros2_planner import UPF4ROS2_Planner # needed for create_planner_from_key

from .path_planner import Path_Planner

from .goal_handler import GoalHandler
from auspex_msgs.msg import ExecutorCommand, PlannerCommand, PlanStatus

from .interfaces.monitor_interface import MonitorInterface
from auspex_db_client.kb_client import KB_Client
from .planner.utils.converter import enum_to_str
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
        self._add_plan_subscriber = self.create_subscription(Plan, 'add_plan', self.add_plan_callback, 10)

        self._monitor_interfaces = {}

        self._kb_client = kb_client

        self._EXECUTION_MODE = EXECUTION_MODE.AUTO

        self._plan_id = 0

        self._team_planners = {}

        self._path_planner = Path_Planner(self._kb_client)

        self._default_planner = Mock_Planner(self._kb_client)

        self.get_logger().info('Planner Main Ready with planner_command and add_plan subscribers...')


    def execute_command(self, team_id: str, platform_id: str, execution_command):
        """
        Calls the respective executor monitor for the team.
        """
        if not(team_id in self._monitor_interfaces):
            self._monitor_interfaces[team_id] = MonitorInterface(self, team_id, self.feedback_callback, self.result_callback, self.cancel_done_callback)
        self._monitor_interfaces[team_id].send_command(command=execution_command, platform_id=platform_id)


    def result_callback(self, monitor_interface, msg):
        self.get_logger().info(f'Result: {enum_to_str(PlannerCommand, msg.command)}')

        team_id = monitor_interface._team_id
        platform_id = ""

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
                    self.execute_command(team_id, platform_id, ExecutorCommand.EXECUTE)


    def feedback_callback(self, monitor_interface, feedback_msg):
        self.get_logger().info('Received feedback')
        team_id = monitor_interface._team_id
        self.get_planning_engine(team_id).feedback(team_id, feedback_msg)


    def cancel_done_callback(self, monitor_interface, msg):
        self.get_logger().info("Canceled Plan of team"+ monitor_interface._team_id)


    def add_plan_callback(self, plan_msg):
        """
        Callback for the add_plan subscriber.
        Receives a Plan message and inserts it into the Knowledge Base.
        """
        self.get_logger().info(f'Received plan for team: {plan_msg.team_id}, platform: {plan_msg.platform_id}')
        
        try:
            self.insert_plan_to_KB([plan_msg])
            self.get_logger().info(f'Successfully added plan to Knowledge Base for team: {plan_msg.team_id}')
        except Exception as e:
            self.get_logger().error(f'Failed to insert plan into Knowledge Base: {e}')


    def cancel(self, team_id, platform_id=""):
        if platform_id != "" and team_id != "all":
            self.get_logger().info("Got Cancel "+platform_id+" Command")
            # Get the monitor interface for the given team_id
            monitor_interface = self._monitor_interfaces.get(team_id)
            if monitor_interface is None:
                self.get_logger().info(f"[ERROR]: No monitor interface found for team {team_id}. Cannot cancel plan for platform {platform_id}.")
                return
            monitor_interface.cancel_plan(platform_id=platform_id)
        elif team_id.lower() != "all":
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

    def delete_plans_for_team(self, team_id: str):
        """Delete all plans in the Knowledge Base for a given team_id.

        Uses the KB client's delete method with a json path filter on team_id.
        """
        if not team_id:
            self.get_logger().info('[ERROR]: Cannot delete plans. team_id is empty.')
            return False
        try:
            self.get_logger().info(f'[INFO]: Deleting plans for team_id={team_id} from Knowledge Base.')
            success = self._kb_client.delete(collection='plan', key='team_id', value=str(team_id))
            if success:
                self.get_logger().info(f'[INFO]: Successfully deleted plans for team {team_id}.')
            else:
                self.get_logger().info(f'[WARN]: No plans deleted (team {team_id} may have none or delete failed).')
            return success
        except Exception as e:
            self.get_logger().error(f'[ERROR]: Exception while deleting plans for team {team_id}: {e}')
            return False

    def delete_plans_for_platform(self, platform_id: str):
        """Delete all plans in the Knowledge Base for a given platform_id.

        Uses the KB client's delete method with a json path filter on platform_id.
        """
        if not platform_id:
            self.get_logger().info('[ERROR]: Cannot delete plans. platform_id is empty.')
            return False
        try:
            self.get_logger().info(f'[INFO]: Deleting plans for platform_id={platform_id} from Knowledge Base.')
            success = self._kb_client.delete(collection='plan', key='platform_id', value=str(platform_id))
            if success:
                self.get_logger().info(f'[INFO]: Successfully deleted plans for platform {platform_id}.')
            else:
                self.get_logger().info(f'[WARN]: No plans deleted (platform {platform_id} may have none or delete failed).')
            return success
        except Exception as e:
            self.get_logger().error(f'[ERROR]: Exception while deleting plans for platform {platform_id}: {e}')
            return False
    


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
        platform_id = user_command_msg.platform_id 

        if team_id == "":
            rclpy.logging.get_logger("PlanningMain").info("[WARN]: No team ID specified.")
            return 

        if platform_id == "" and usr_cmd > 10:
            rclpy.logging.get_logger("PlanningMain").info("[ERROR]: Platform ID must be specified for a platform command.")
            return

        if usr_cmd != 1 and team_id == "all":
            self.get_logger().info("[ERROR]: Team ID 'all' only defined for cancel.")
            return

        try:
            match usr_cmd:
                case 1: # USER_CANCEL_TEAM
                    rclpy.logging.get_logger("PlanningMain").info("Canceling Plan for team: "+team_id)
                    self.cancel(team_id)
                case 2: # USER_PAUSE_TEAM
                    self.execute_command(team_id=team_id, platform_id="", execution_command=ExecutorCommand.PAUSE)
                case 3: # USER_RESUME_TEAM
                    self.execute_command(team_id=team_id, platform_id="", execution_command=ExecutorCommand.CONTINUE)
                case 4: # USER_START_TEAM
                    self._EXECUTION_MODE = EXECUTION_MODE.AUTO
                    self.plan(team_id)
                    time.sleep(1)
                    self.execute_command(team_id=team_id, platform_id="", execution_command=ExecutorCommand.EXECUTE)
                case 5: # USER_ACCEPT_TEAM
                    self._EXECUTION_MODE = EXECUTION_MODE.HITL
                    self.execute_command(team_id=team_id, platform_id="", execution_command=ExecutorCommand.EXECUTE)
                case 6: # USER_REJECT_TEAM
                    self.delete_plans_for_team(team_id)
                case 7: # USER_PLAN_TEAM
                    self._EXECUTION_MODE = EXECUTION_MODE.HITL
                    self.plan(team_id)
                case 8: # USER_RTH_TEAM
                    rclpy.logging.get_logger("PlanningMain").info("Returning to Home for team: "+team_id)
                    self.cancel(team_id=team_id)
                    time.sleep(4)
                    task_plans = self._default_planner.plan_rth(team_id=team_id)
                    path_plans = self._path_planner.plan_path(team_id, task_plans)
                    self.insert_plan_to_KB(path_plans)
                    time.sleep(1)
                    self.execute_command(team_id=team_id, platform_id="", execution_command=ExecutorCommand.EXECUTE)
                case 9: # USER_TERMINATE_TEAM
                    self.execute_command(team_id=team_id, platform_id="", execution_command=ExecutorCommand.TERMINATE)
                    rclpy.logging.get_logger("PlanningMain").info("Terminating all drones of team: "+team_id)
                case 10: # USER_KILL_TEAM
                    self.execute_command(team_id=team_id, platform_id="", execution_command=ExecutorCommand.KILL)
                    rclpy.logging.get_logger("PlanningMain").info("Killing all drones of team: "+team_id)
                case 11: # USER_CANCEL_PLATFORM
                    rclpy.logging.get_logger("PlanningMain").info("Canceling Plan for platform: "+platform_id)
                    self.cancel(team_id=team_id, platform_id=platform_id)
                case 12: # USER_PAUSE_PLATFORM
                    self.execute_command(team_id=team_id, platform_id=platform_id, execution_command=ExecutorCommand.PAUSE)
                case 13: # USER_RESUME_PLATFORM
                    self.execute_command(team_id=team_id, platform_id=platform_id, execution_command=ExecutorCommand.CONTINUE)
                case 14: # USER_START_PLATFORM
                    rclpy.logging.get_logger("PlanningMain").info("Not implemented yet.")
                    pass
                case 15: # USER_ACCEPT_PLATFORM
                    self._EXECUTION_MODE = EXECUTION_MODE.HITL
                    self.execute_command(team_id=team_id, platform_id=platform_id, execution_command=ExecutorCommand.EXECUTE)
                case 16: # USER_REJECT_PLATFORM
                    self.delete_plans_for_platform(platform_id)
                case 17: # USER_PLAN_PLATFORM
                    rclpy.logging.get_logger("PlanningMain").info("Not implemented yet.")
                    pass
                case 18: # USER_RTH_PLATFORM
                    rclpy.logging.get_logger("PlanningMain").info("Returning to Home for platform: "+platform_id)
                    self.cancel(team_id=team_id, platform_id=platform_id)
                    time.sleep(4)
                    task_plans = self._default_planner.plan_rth(team_id=team_id, platform_id=platform_id)
                    path_plans = self._path_planner.plan_path(team_id, task_plans)
                    self.insert_plan_to_KB(path_plans)
                    time.sleep(1)
                    self.execute_command(team_id=team_id, platform_id=platform_id, execution_command=ExecutorCommand.EXECUTE)
                case 19: # USER_TERMINATE_PLATFORM
                    self.execute_command(team_id=team_id, platform_id=platform_id, execution_command=ExecutorCommand.TERMINATE)
                    rclpy.logging.get_logger("PlanningMain").info("Terminating platform: "+platform_id)
                case 20: # USER_KILL_PLATFORM
                    self.execute_command(team_id=team_id, platform_id=platform_id, execution_command=ExecutorCommand.KILL)
                    rclpy.logging.get_logger("PlanningMain").info("Killing platform: "+platform_id)
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
