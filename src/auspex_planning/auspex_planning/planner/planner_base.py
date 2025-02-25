from abc import ABC, abstractmethod

class PlannerBase(ABC):

    @abstractmethod
    def plan_mission(self, team_id):
        """
        Defines a blueprint for the plan method. 
        This method must be implemented by any subclass of Planner.
        Returning a auspex_msgs::Plan
        """
        pass

    @abstractmethod
    def update_state(self, state):
        """
        Updates the new state of the platform. Must be implemented by any subclass. 

        Parameters:
        - state: The new state returned by the platform.
        """
        pass

    @abstractmethod
    def feedback(self, team_id, platform_id, feedback_msg):
        """
        Is invoked by each platform to update th eplanner if necessary according to this feedback.

        Parameters:
        - team_id: The team the platform is part of.
        - platform_id: The name of platform, which received feedback
        - feedback_msg: The feedback_msg
        """
        pass

    @abstractmethod
    def result(self, team_id, platform_id, result_msg):
        """
        Is invoked by each platform after an action was finished.

        Parameters:
        - team_id: The team the platform is part of.
        - platform_id: The name of platform, which received feedback
        - result_msg: The result_msg
        """
        pass
