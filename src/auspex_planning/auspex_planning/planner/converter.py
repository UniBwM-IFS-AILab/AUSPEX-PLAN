from .up_planner_utils.ros2_interface_writer import ROS2InterfaceWriter
from .up_planner_utils.ros2_interface_reader import ROS2InterfaceReader
from auspex_msgs.msg import PlanStatus
from msg_context.loader import ActionInstance, Plan, ActionStatus

from fractions import Fraction

from upf_msgs.msg import (
    Atom,
    Real
)

import json
from typing import Dict, Any, List

def enum_to_str(msg_cls, value, prefix=None):
    return next((name for name in dir(msg_cls)
                 if ((prefix and name.startswith(prefix)) or (not prefix and name.isupper()))
                 and getattr(msg_cls, name) == value), "UNKNOWN")

class AUSPEXConverter:
    """
    A basic structure for a converter utility.
    This class is intended to handle conversions between different data formats or representations.
    """

    def __init__(self):
        """
        Initialize the converter with any necessary configurations or parameters.
        """
        self._ros2_interface_writer = ROS2InterfaceWriter()
        self._ros2_interface_reader = ROS2InterfaceReader()

    def convert_plan_upf2auspex(self, tasks):
        parsed_tasks = []
        id_task = 0
        for task in tasks:
            new_task = ActionInstance()
            new_task.id = id_task
            new_task.action_name = task.action_name
            if task.action_name == "fly":
                new_task.action_name = "fly_3D"
                new_task.parameters = task.parameters[:1] + task.parameters[2:] if len(task.parameters) > 1 else task.parameters
            elif "search" in task.action_name:
                new_task.action_name = task.action_name
                new_task.parameters = task.parameters[:1] + task.parameters[2:] if len(task.parameters) > 1 else task.parameters
            else:
                new_task.parameters = task.parameters
            new_task.status = enum_to_str(ActionStatus, ActionStatus.INACTIVE)
            parsed_tasks.append(new_task)
            id_task += 1
        return parsed_tasks

    def convert_up2upf(self, msg):
        return self._ros2_interface_writer.convert(msg)

    def convert_plan_up2auspex(self, plan):
        """
        Perform the conversion on the input data.

        Args:
            input_data: The data to be converted.

        Returns:
            The converted data.
        """
        _plan_upf = self.convert_up2upf(plan)
        return self.convert_plan_upf2auspex(_plan_upf.actions)

    def convert_plan_llm2auspex(self, platform_id, team_id, action_list):
        """
        Creates an ActionInstance message for each action in action_list and assigns a platform name to it
        """
        planned_tasks = []
        for id_a, action in enumerate(action_list):
            new_action = ActionInstance()
            new_action.action_name = action[0]

            new_action.id = id_a
            new_action.status = enum_to_str(ActionStatus, ActionStatus.INACTIVE)

            new_parameters = []
            atom = Atom()
            atom.symbol_atom = [platform_id]
            new_parameters.append(atom)
            action.pop(0)
            for param in action:
                atom = Atom()
                if type(param) == str:
                    atom.symbol_atom = [param]
                elif type(param) == float:
                    fraction_representation = Fraction.from_float(param)
                    real_msg = Real()
                    real_msg.numerator = fraction_representation.numerator
                    real_msg.denominator = fraction_representation.denominator

                    atom.real_atom = [real_msg]
                elif type(param) == bool:
                    atom.bool_atom = [param]
                elif type(param) == int:
                    fraction_representation = Fraction.from_float(param)
                    real_msg = Real()
                    real_msg.numerator = fraction_representation.numerator
                    real_msg.denominator = fraction_representation.denominator
                    atom.real_atom = [real_msg]
                else:
                    atom.symbol_atom = [param]
                new_parameters.append(atom)

            new_action.parameters = new_parameters
            planned_tasks.append(new_action)
        return planned_tasks

    def convert_plan_mvrp2auspex(self, platform_id, waypoints):
        """
        Converts a list of points into a sequence of auspex_msgs ActionInstance messages.

        :param points: List of POINT Z (lat, lon, alt) objects.
        :return: List of auspex_msgs.msg.ActionInstance messages.
        """
        actions = []
        # Take-off action
        take_off_action = ActionInstance(
            id=len(actions),
            action_name="take_off",
            parameters=[
                Atom(symbol_atom=[platform_id], int_atom=[], real_atom=[], boolean_atom=[]),
                Atom(symbol_atom=[], int_atom=[], real_atom=[Real(numerator=10, denominator=1)], boolean_atom=[])
            ],
            status=enum_to_str(ActionStatus, ActionStatus.INACTIVE)
        )
        actions.append(take_off_action)


        # Fly_3D actions for each point
        for wp_id in waypoints:
            fly_action = ActionInstance(
                id=len(actions),
                action_name="fly_3D",
                parameters=[
                    Atom(symbol_atom=[platform_id], int_atom=[], real_atom=[], boolean_atom=[]),
                    Atom(symbol_atom=[wp_id], int_atom=[], real_atom=[], boolean_atom=[])
                ],
                status=enum_to_str(ActionStatus, ActionStatus.INACTIVE)
            )
            actions.append(fly_action)

        # Fly back to home action
        fly_home_action = ActionInstance(
            id=len(actions),
            action_name="fly_3D",
            parameters=[
                Atom(symbol_atom=[platform_id], int_atom=[], real_atom=[], boolean_atom=[]),
                Atom(symbol_atom=["home"], int_atom=[], real_atom=[], boolean_atom=[])
            ],
            status=enum_to_str(ActionStatus, ActionStatus.INACTIVE)
        )
        actions.append(fly_home_action)

        # Land action
        land_action = ActionInstance(
            id=len(actions),
            action_name="land",
            parameters=[
                Atom(symbol_atom=[platform_id], int_atom=[], real_atom=[], boolean_atom=[])
            ],
            status=enum_to_str(ActionStatus, ActionStatus.INACTIVE)
        )
        actions.append(land_action)

        return actions

    def convert_plan_pattern2auspex(self, platform_id, points):
        """
        Converts a list of points into a sequence of auspex_msgs ActionInstance messages.

        :param points: List of POINT Z (lat, lon, alt) objects.
        :return: List of auspex_msgs.msg.ActionInstance messages.
        """
        actions = []

        # Take-off action
        take_off_action = ActionInstance(
            id=len(actions),
            action_name="take_off",
            parameters=[
                Atom(symbol_atom=[platform_id], int_atom=[], real_atom=[], boolean_atom=[]),
                Atom(symbol_atom=[], int_atom=[], real_atom=[Real(numerator=10, denominator=1)], boolean_atom=[])
            ],
            status = enum_to_str(ActionStatus, ActionStatus.INACTIVE)
        )
        actions.append(take_off_action)

        # Fly_3D actions for each point
        for point in points:
            lat, lon, alt = point.x, point.y, point.z  # Extract lat, lon, alt from the POINT Z object

            fraction_representation_lat = Fraction.from_float(point.x)
            real_msg_lat = Real()
            real_msg_lat.numerator = fraction_representation_lat.numerator
            real_msg_lat.denominator = fraction_representation_lat.denominator

            fraction_representation_lon = Fraction.from_float(point.y)
            real_msg_lon = Real()
            real_msg_lon.numerator = fraction_representation_lon.numerator
            real_msg_lon.denominator = fraction_representation_lon.denominator

            fraction_representation_alt = Fraction.from_float(point.z)
            real_msg_alt = Real()
            real_msg_alt.numerator = fraction_representation_alt.numerator
            real_msg_alt.denominator = fraction_representation_alt.denominator

            fly_action = ActionInstance(
                id=len(actions),
                action_name="fly_3D",
                parameters=[
                    Atom(symbol_atom=[platform_id], int_atom=[], real_atom=[], boolean_atom=[]),
                    Atom(symbol_atom=[], int_atom=[], real_atom=[real_msg_lat], boolean_atom=[]),
                    Atom(symbol_atom=[], int_atom=[], real_atom=[real_msg_lon], boolean_atom=[]),
                    Atom(symbol_atom=[], int_atom=[], real_atom=[real_msg_alt], boolean_atom=[])
                ],
                status=enum_to_str(ActionStatus, ActionStatus.INACTIVE)
            )
            actions.append(fly_action)

        # Fly back to home action
        fly_home_action = ActionInstance(
            id=len(actions),
            action_name="fly_3D",
            parameters=[
                Atom(symbol_atom=[platform_id], int_atom=[], real_atom=[], boolean_atom=[]),
                Atom(symbol_atom=["home"], int_atom=[], real_atom=[], boolean_atom=[])
            ],
            status=enum_to_str(ActionStatus, ActionStatus.INACTIVE)
        )
        actions.append(fly_home_action)

        # Land action
        land_action = ActionInstance(
            id=len(actions),
            action_name="land",
            parameters=[
                Atom(symbol_atom=[platform_id], int_atom=[], real_atom=[], boolean_atom=[])
            ],
            status=enum_to_str(ActionStatus, ActionStatus.INACTIVE)
        )
        actions.append(land_action)

        return actions


    def convert_plan_patternPath2auspex(self, platform_id, points):
        """
        Converts a list of points into a sequence of auspex_msgs ActionInstance messages.
        """
        actions = []

        for point in points:
            lat, lon, alt = point.x, point.y, point.z  # Extract lat, lon, alt from the POINT Z object

            fraction_representation_lat = Fraction.from_float(point.x)
            real_msg_lat = Real()
            real_msg_lat.numerator = fraction_representation_lat.numerator
            real_msg_lat.denominator = fraction_representation_lat.denominator

            fraction_representation_lon = Fraction.from_float(point.y)
            real_msg_lon = Real()
            real_msg_lon.numerator = fraction_representation_lon.numerator
            real_msg_lon.denominator = fraction_representation_lon.denominator

            fraction_representation_alt = Fraction.from_float(point.z)
            real_msg_alt = Real()
            real_msg_alt.numerator = fraction_representation_alt.numerator
            real_msg_alt.denominator = fraction_representation_alt.denominator

            fly_action = ActionInstance(
                id=len(actions),
                action_name="fly_3D",
                parameters=[
                    Atom(symbol_atom=[platform_id], int_atom=[], real_atom=[], boolean_atom=[]),
                    Atom(symbol_atom=[], int_atom=[], real_atom=[real_msg_lat], boolean_atom=[]),
                    Atom(symbol_atom=[], int_atom=[], real_atom=[real_msg_lon], boolean_atom=[]),
                    Atom(symbol_atom=[], int_atom=[], real_atom=[real_msg_alt], boolean_atom=[])
                ],
                status=enum_to_str(ActionStatus, ActionStatus.INACTIVE)
            )
            actions.append(fly_action)

        return actions