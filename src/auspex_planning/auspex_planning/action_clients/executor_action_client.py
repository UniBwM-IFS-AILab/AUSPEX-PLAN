from rclpy import logging
from rclpy.action import ActionClient
from auspex_msgs.action import ExecutePlan

from action_msgs.msg import GoalStatus
from rclpy.callback_groups import ReentrantCallbackGroup
import traceback 

class ExecutorActionClient:
    """
    Aciton Client for each executor

    """
    def __init__(self, node, platform_id,  feedback_callback, result_callback, cancel_callback):
        """
        Constructor method

        :param Node node: the planner node
        :param int platform_id: the id of the platform this executor belongs to
        :param Callable feedback_callback: function pointer for feedback callback function in plan executor
        :param Callable result_callback: function pointer for result callback function in plan executor
        """
        self._name = "ActionServer for " + platform_id
        self._node = node
        self._logger = logging.get_logger(self.__class__.__name__)
        self._platform_id = platform_id
        self._callback_group = ReentrantCallbackGroup()
        self._action_client = ActionClient(self._node, ExecutePlan, "/"+ platform_id +"/plan_execution")

        self._goal_handle = None
        self._feedback_callback = feedback_callback
        self._result_callback = result_callback
        self._cancel_done_callback = cancel_callback
        self._future_handle = None

    def send_plan(self, plan: list):
        """
        Sends a plan to a given action server
        """   
        goal_msg = ExecutePlan.Goal()
        goal_msg.plan.actions = plan
        while not self._action_client.wait_for_server(timeout_sec=1.0):
            self._logger.info('Waiting for action server to come online...')
        self._future_handle = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_wrapper)
        self._future_handle.add_done_callback(self.goal_response_callback)    

    def goal_response_callback(self, future):
        """
        Checks if goal accepted
        """
        self._goal_handle = future.result()
        if not self._goal_handle.accepted:
            self._logger.info('Plan rejected by executor...')
            return

        self._logger.info('Plan accepted by executor...')

        self._future_handle = self._goal_handle.get_result_async()
        self._future_handle.add_done_callback(self.get_result_callback)    

    def get_result_callback(self, future):
        """
        Handles the result from the action server and forwards it to the plan executor

        :param Future future: the future containing the response from the action server

        """
        if self._goal_handle:
            if self._goal_handle._status == GoalStatus.STATUS_CANCELED:
                self._logger().info("Goal for "+self._platform_id +"canceled.")

        self._result_callback(self, future)

    def cancel_action_goal(self):
        """
        Cancels the current action
        """
        if self._goal_handle:
            if self._goal_handle._status == GoalStatus.STATUS_ACCEPTED or self._goal_handle._status == GoalStatus.STATUS_EXECUTING:  
                try:
                    #self._goal_handle.cancel_goal()
                    self._cancel_future  = self._goal_handle.cancel_goal_async()
                    #rclpy.spin_until_future_complete(self._node, cancel_future)
                    self._cancel_future.add_done_callback(self.cancel_done_callback_wrapper)  
                except:
                    traceback.print_exc() 

    def cancel_done_callback_wrapper(self, future):
        self._cancel_done_callback(self, future)


    def feedback_wrapper(self, feedback_msg):
        """
        Processes Feedback messages from the action server and forwards it to the plan executor, can be overwritten in inheriting classes

        :param _action_type.Feedback() feedback_msg: the message containing the feedback from the action server

        """
        feedback = feedback_msg.feedback
        self._feedback_callback(self, feedback)
