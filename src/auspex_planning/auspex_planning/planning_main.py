#!/usr/bin/env python3
import rclpy
import time
import json
from enum import Enum
from action_msgs.msg import GoalStatus

from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor

from .planner.mock_planner import Mock_Planner
from .planner.pddl_planner import PDDL_Planner
from .planner.pattern_planner import PatternPlanner
from .planner.llm_planner import LLM_Planner
from .planner.mvrp_alns_planner import MVPR_ALNS_Planner
from .planner.up_planner import UP_Planner


from auspex_planning.action_clients.executor_action_client import ExecutorActionClient
from .planner.kb_client import KB_Client
from .planner.utils import parse_plan_from_dict
import rosidl_runtime_py

from auspex_msgs.action import ExecutePlan
from auspex_msgs.msg import UserCommand
from auspex_msgs.msg import Plan

class EXECUTION_MODE(Enum):
    HITL = 1 #Human in the loop
    AUTO = 2

class PlanningMain(Node):
    def __init__(self):
        super().__init__('main_planner_node')

        """
        Create a subscriber to the planner command topic
        """
        self._planner_command_subscriber = self.create_subscription(UserCommand,'planner_command', self.planner_command_callback,10)

        """
        Placeholder for the action clients
        """
        self._executor_clients = {}

        """
        Knowledge Base Interface
        """
        self._kb_client = KB_Client()

        """
        The execution mode. HITL -> human in the loop
        """
        self._EXECUTION_MODE = EXECUTION_MODE.AUTO

        """
        Create Planner
        """
        self._planner_dict = {}
        self._planner_dict['mock_planner'] = Mock_Planner(self._kb_client)
        self._planner_dict['pddl_planner'] = PDDL_Planner(self._kb_client)
        self._planner_dict['pattern_planner'] = PatternPlanner(self._kb_client)
        self._planner_dict['llm_planner'] = LLM_Planner(self._kb_client)
        self._planner_dict['mvrp_alns_planner'] = MVPR_ALNS_Planner(self._kb_client)
        self._planner_dict['up_planner'] = UP_Planner(self._kb_client)

        self.get_logger().info('Planner Main Ready...')


    def execute_plans(self, plans: list):
        """
        Calls the respective executor for the platform for which the plan is stored, with the plan.
        """
        for plan in plans:
            platform_id = plan.platform_id
            actions = plan.actions
            if not(platform_id in self._executor_clients): 
                # Create an Actionclient  
                self._executor_clients[platform_id] = ExecutorActionClient(self, platform_id, self.feedback_callback, self.result_callback, self.cancel_done_callback)

            self._executor_clients[platform_id].send_plan(actions)


    def result_callback(self, action_client, future):
        """
        Gets result
        """
        self.get_logger().info("Goal Status: " + future.result().result.result)

        result = future.result().result
        team_id = self._kb_client.query('platform', 'team_id', 'platform_id', action_client._platform_id)[0]['team_id']

        if result.success == True:
            self.get_logger().info("Plan for "+action_client._platform_id+ " in team: " + team_id+" succeeded")
        else:
            self.get_logger().info("Plan "+action_client._platform_id+ " failed")

        # Notify planner
        new_plans = self.get_planning_engine(team_id).result(team_id, action_client._platform_id, result)

        if new_plans:
            self.insert_plan_to_KB(new_plans)
            if self._EXECUTION_MODE == EXECUTION_MODE.AUTO:
                self.execute_plans(new_plans)


    def feedback_callback(self, action_client, feedback_msg):
        """
        Feedback Callback
        """
        self.get_logger().info('Received feedback')
        team_id = self._kb_client.query('platform', 'team_id', 'platform_id', action_client._platform_id)[0]['team_id']
        # Notify Planner
        self.get_planning_engine(team_id).feedback(team_id, action_client._platform_id, feedback_msg)

    def cancel(self, team_id):
        """
        Cancel goal(s)
        """
        if team_id.lower() != "all":
            vhcl_dict = self._kb_client.query('platform', 'platform_id', 'team_id', team_id)
            self.get_logger().info("Got Cancel "+team_id+" Command")
            for platform in vhcl_dict:
                self._executor_clients[platform['platform_id']].cancel_action_goal()
                
        elif team_id.lower() == "all":
            self.get_logger().info("Got Cancel All Command")
            for client in self._executor_clients.values():
                self.get_logger().info("Trying to cancel goals for "+client._platform_id+".")
                client.cancel_action_goal()
        else:
            return 

    def cancel_done_callback(self, action_client, future):
        self.get_logger().info("Canceled Action Goal of "+ action_client._platform_id)

    def get_planning_engine(self, team_id):
        planner_list = self._kb_client.query(collection='config', field='planner',key='team_id',value=team_id)

        if not planner_list:
            self.get_logger().info('[ERROR]: Empty planner queried from Knowledge Base.')
            return None
        return self._planner_dict[planner_list[0]['planner']]

    def insert_plan_to_KB(self, plans):
        if plans == []:
            self.get_logger().info("[INFO]: Inserting empty plan into Knowledge base.")
            plans = [Plan()]
        for plan in plans:
            plan_dict = rosidl_runtime_py.convert.message_to_ordereddict(plan)
            plan_json = ''
            try:
                plan_json = json.dumps(plan_dict, indent=4)
            except Exception as e:
                self.get_logger().info("[ERROR]: not a valid json")

            self._kb_client.write(collection='plan', entity=plan_json, field='', key='platform_id', value=plan.platform_id)

    def planner_command_callback(self, user_command_msg):
        usr_cmd = user_command_msg.user_command
        target_platforms = user_command_msg.target_platforms

        DEFAULT_TEAM_ID = 'drone_team'

        if target_platforms == "":
            target_platforms = DEFAULT_TEAM_ID

        try:
            match usr_cmd:
                case 1: # USER_CANCEL
                    self.cancel(target_platforms)
                    #TODO get drones to team id and publish empty plan
                case 2: # USER_PAUSE
                    pass
                case 3: # USER_RESUME
                    pass
                case 4: # USER_START
                    self._EXECUTION_MODE = EXECUTION_MODE.AUTO
                    engine = self.get_planning_engine(target_platforms)

                    if engine == None:
                        self.get_logger().info("[ERROR]: No planner selected.")
                        return

                    plans = engine.plan_mission(target_platforms)

                    self.execute_plans(plans)
                    self.insert_plan_to_KB(plans)
                case 5: # USER_ACCEPT
                    self._EXECUTION_MODE = EXECUTION_MODE.HITL
                    plans = self._kb_client.query(collection='plan', key='team_id', value=target_platforms)
                    plans_msg = parse_plan_from_dict(plans)
                    self.execute_plans(plans_msg)
                case 6: # USER_REJECT
                    #TODO publish empty plan to KB
                    pass
                case 7: # USER_PLAN
                    self._EXECUTION_MODE = EXECUTION_MODE.HITL
                    engine = self.get_planning_engine(target_platforms)

                    if engine == None:
                        self.get_logger().info("[ERROR]: No planner selected.")
                        return

                    plans = engine.plan_mission(target_platforms)

                    self.insert_plan_to_KB(plans)
                case 8: # USER_RTH
                    self.cancel(target_platforms)
                    time.sleep(4)
                    plans = self._planner_dict['mock_planner'].plan_rth(target_platforms)
                    self.execute_plans(plans)
                    self.insert_plan_to_KB(plans)
                case 9: # USER_TERMINATE
                    self.cancel(target_platforms)
                    # TODO Publish to DRONE to terminate
        except Exception as e:
            self.get_logger().info(f"An unexpected error occurred: {e}")


def main():
    rclpy.init(args=None)

    mte = SingleThreadedExecutor()
    planning_main_node = PlanningMain()

    mte.add_node(planning_main_node)
    mte.spin()

    """
    Destroying nodes
    """
    planning_main_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
