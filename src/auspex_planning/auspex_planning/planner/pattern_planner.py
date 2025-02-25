#!/usr/bin/env python3
import math
from rclpy.node import Node
from geographic_msgs.msg import GeoPoint
from .pattern_planner_utils.search_pattern_generator import SearchPatternGenerator
from .planner_base import PlannerBase
from .utils import convert_points_to_actions

from msg_context.loader import Waypoints, Plan


class PatternPlanner(PlannerBase, Node):
    def __init__(self, kb_client):
        super().__init__('pattern_planner')

        self._kb_client = kb_client
        self.waypoint_pub = self.create_publisher(Waypoints, 'waypoints', 10)

    def feedback(self, team_id, platform_id, feedback_msg):
        pass

    def result(self, team_id, platform_id, result_msg):
        pass

    def update_state(self, state):
        pass

    def plan_mission(self,team_id):
        print(f"[INFO]: Pattern Planner Selected for team : {team_id}")
        mission_dict = self._kb_client.query('mission', '', 'team_id', team_id)

        if mission_dict == []:
            return []
        mission_dict = mission_dict[0]

        points =  mission_dict['search_area']['points']
        search_area = []

        for point in points:
            search_area.append((float(point['latitude']), float(point['longitude'])))

        starting_point = (float(mission_dict['starting_point']['latitude']), float(mission_dict['starting_point']['longitude']))

        flight_height = int(mission_dict['desired_ground_dist'])

        lawnmower = SearchPatternGenerator()

        platforms_of_team = self._kb_client.query('platform', 'platform_id', 'team_id', team_id)

        action_list = []

        for platform_json in platforms_of_team:
            platform_capabilities = self._kb_client.query('capabilities', '', 'platform_id', platform_json['platform_id'])
            if platform_capabilities == []:
                print("Can not plan, because respective capabilities are empty.")
                return []

            # TODO get fov from caps
            fov = math.pi / 6.0 # in radians
            turning_radius = 100.0 # meter

            waypoints = lawnmower.mow_the_lawn(search_area, starting_point, fov, flight_height, turning_radius)
            # TODO if platform_capabilities[0]['platform_class'] == PlatformClass.PLATFORM_CLASS_UL:
            if waypoints == []:
                print("computed empty waypoints")
                continue
            if team_id == 'ul_team':
                self.publish_waypoints(waypoints,platform_json['platform_id'])
            else:
                self.publish_waypoints(waypoints,platform_json['platform_id'])
                action_list.append(self.parse_waypoints2actions(waypoints, platform_json['platform_id'], team_id))
        print("Successfully planned waypoints with the pattern planner.")
        return action_list

    def parse_waypoints2actions(self, waypoints, platform_id, team_id):
        plan = Plan()
        plan.platform_id = platform_id
        plan.team_id = team_id 
        plan.actions = convert_points_to_actions(platform_id, waypoints)
        return plan

    def publish_waypoints(self, waypoints, platform_id):
        msg = Waypoints()
        msg.route_id = platform_id + 'route'
        msg.platform_id = platform_id

        # TODO get highest point in the search area in AGL using copernicus or move to pattern generator
        highest_area_point = 1000.0
        for waypoint in waypoints:
            ros_point = GeoPoint(latitude=waypoint.x, longitude=waypoint.y, altitude = waypoint.z + highest_area_point)
            msg.points.append(ros_point)

        self.waypoint_pub.publish(msg)
