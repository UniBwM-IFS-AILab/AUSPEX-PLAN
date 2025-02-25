import up_msgs
from msg_context.loader import ActionInstance, Plan
from fractions import Fraction
from up_msgs.msg import (
    Atom,
    Real
)

def parse_plan_from_dict(plan_dict_list):
        """
        Parses a list of dictionaries into a list of auspex_msgs.msg.loader.Plan messages.

        :param plan_dict_list: List of dictionaries representing the plan.
        :return: List of auspex_msgs.msg.Plan messages.
        """
        parsed_plans = []

        for plan_dict in plan_dict_list:
            platform_id = plan_dict["platform_id"]
            actions = []

            for action in plan_dict["actions"]:
                action_id = action["id"]
                action_name = action["action_name"]
                status = action["status"]

                parameters = []
                for param in action["parameters"]:
                    symbol_atom = param["symbol_atom"]
                    int_atom = param["int_atom"]
                    boolean_atom = param["boolean_atom"]

                    # Convert real_atom entries to up_msgs.msg.Real
                    real_atom = [
                        up_msgs.msg.Real(numerator=real["numerator"], denominator=real["denominator"])
                        for real in param["real_atom"]
                    ]

                    # Create up_msgs.msg.Atom instance
                    atom_msg = up_msgs.msg.Atom(
                        symbol_atom=symbol_atom,
                        int_atom=int_atom,
                        real_atom=real_atom,
                        boolean_atom=boolean_atom
                    )
                    parameters.append(atom_msg)

                # Create auspex_msgs.msg.ActionInstance
                action_instance = ActionInstance(
                    id=action_id,
                    action_name=action_name,
                    parameters=parameters,
                    status=status
                )
                actions.append(action_instance)

            # Create auspex_msgs.msg.Plan
            plan_msg = Plan(
                platform_id=platform_id,
                actions=actions
            )

            parsed_plans.append(plan_msg)

        return parsed_plans
    
def convert_points_to_actions(platform_id, points):
    """
    Converts a list of points into a sequence of auspex_msgs ActionInstance messages.
    
    :param points: List of POINT Z (lat, lon, alt) objects.
    :return: List of auspex_msgs.msg.ActionInstance messages.
    """
    actions = []

    # Take-off action
    take_off_action = ActionInstance(
        id=platform_id,
        action_name="take_off",
        parameters=[
            up_msgs.msg.Atom(symbol_atom=[platform_id], int_atom=[], real_atom=[], boolean_atom=[]),
            up_msgs.msg.Atom(symbol_atom=[], int_atom=[], real_atom=[up_msgs.msg.Real(numerator=10, denominator=1)], boolean_atom=[])
        ],
        status=1
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
            id=platform_id,
            action_name="fly_3D",
            parameters=[
                up_msgs.msg.Atom(symbol_atom=[platform_id], int_atom=[], real_atom=[], boolean_atom=[]),
                up_msgs.msg.Atom(symbol_atom=[], int_atom=[], real_atom=[real_msg_lat], boolean_atom=[]),
                up_msgs.msg.Atom(symbol_atom=[], int_atom=[], real_atom=[real_msg_lon], boolean_atom=[]),
                up_msgs.msg.Atom(symbol_atom=[], int_atom=[], real_atom=[real_msg_alt], boolean_atom=[])         
            ],
            status=1
        )
        actions.append(fly_action)

    # Fly back to home action
    fly_home_action = ActionInstance(
        id=platform_id,
        action_name="fly_3D",
        parameters=[
            up_msgs.msg.Atom(symbol_atom=[platform_id], int_atom=[], real_atom=[], boolean_atom=[]),
            up_msgs.msg.Atom(symbol_atom=["home"], int_atom=[], real_atom=[], boolean_atom=[])
        ],
        status=1
    )
    actions.append(fly_home_action)

    # Land action
    land_action = ActionInstance(
        id=platform_id,
        action_name="land",
        parameters=[
            up_msgs.msg.Atom(symbol_atom=[platform_id], int_atom=[], real_atom=[], boolean_atom=[])
        ],
        status=1
    )
    actions.append(land_action)

    return actions

def convert_waypointsIDs_to_actions(platform_id, waypoints):
    """
    Converts a list of points into a sequence of auspex_msgs ActionInstance messages.
    
    :param points: List of POINT Z (lat, lon, alt) objects.
    :return: List of auspex_msgs.msg.ActionInstance messages.
    """
    actions = []

    # Take-off action
    take_off_action = ActionInstance(
        id=platform_id,
        action_name="take_off",
        parameters=[
            up_msgs.msg.Atom(symbol_atom=[platform_id], int_atom=[], real_atom=[], boolean_atom=[]),
            up_msgs.msg.Atom(symbol_atom=[], int_atom=[], real_atom=[up_msgs.msg.Real(numerator=10, denominator=1)], boolean_atom=[])
        ],
        status=1
    )
    actions.append(take_off_action)

    # Fly_3D actions for each point
    for wp_id in waypoints: 
        fly_action = ActionInstance(
            id=platform_id,
            action_name="fly_3D",
            parameters=[
                up_msgs.msg.Atom(symbol_atom=[platform_id], int_atom=[], real_atom=[], boolean_atom=[]),
                up_msgs.msg.Atom(symbol_atom=[wp_id], int_atom=[], real_atom=[], boolean_atom=[])       
            ],
            status=1
        )
        actions.append(fly_action)

    # Fly back to home action
    fly_home_action = ActionInstance(
        id=platform_id,
        action_name="fly_3D",
        parameters=[
            up_msgs.msg.Atom(symbol_atom=[platform_id], int_atom=[], real_atom=[], boolean_atom=[]),
            up_msgs.msg.Atom(symbol_atom=["home"], int_atom=[], real_atom=[], boolean_atom=[])
        ],
        status=1
    )
    actions.append(fly_home_action)

    # Land action
    land_action = ActionInstance(
        id=platform_id,
        action_name="land",
        parameters=[
            up_msgs.msg.Atom(symbol_atom=[platform_id], int_atom=[], real_atom=[], boolean_atom=[])
        ],
        status=1
    )
    actions.append(land_action)

    return actions