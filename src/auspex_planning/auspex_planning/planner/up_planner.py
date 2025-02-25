from rclpy.node import Node
from rclpy.action import ActionClient
from up_msgs.srv import (AddAction, SetInitialValue, AddGoal)
from up_msgs.action import PlanOneShot
from up_msgs.msg import ActionInstance, Fluent, Problem, Goal
from unified_planning.model import Problem as UPProblem
from .planner_base import PlannerBase

class UP_Planner(Node, PlannerBase):
    def __init__(self, kb_client):
        super().__init__('up_planner')

        self._kb_client = kb_client
        # Create service clients

        self.add_action_client = self.create_client(AddAction, '/up4ros2/srv/add_action')
        self.set_initial_state_client = self.create_client(SetInitialValue, '/up4ros2/srv/set_initial_value')
        self.set_goal_client = self.create_client(AddGoal, '/up4ros2/srv/add_goal')

        # Create an action client for planning
        self.plan_client = ActionClient(self, PlanOneShot, '/up4ros2/action/planOneShot')

    def plan_mission(self, team_id):
        """ Requests a plan from the planner. """
        return []

    def update_state(self, state):
        """ Updates the state representation. """
        pass

    def feedback(self, team_id, platform_id, feedback_msg):
        pass

    def result(self, team_id, platform_id, result_msg):
        pass
