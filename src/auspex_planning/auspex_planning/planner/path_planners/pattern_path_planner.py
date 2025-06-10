#!/usr/bin/env python3
import json
import math
from rclpy.node import Node
from geographic_msgs.msg import GeoPoint
from auspex_planning.planner.pattern_planner_utils.search_pattern_generator import SearchPatternGenerator
from auspex_planning.planner.planner_base import PlannerBase
from auspex_planning.planner.converter import AUSPEXConverter
from msg_context.loader import Waypoints, Plan


class PatternPathPlanner(Node):
    planner_key = 'pattern_planner'

    def __init__(self, kb_client):
        super().__init__('pattern_path_planner')

        self._kb_client = kb_client

        self._converter = AUSPEXConverter()
        self.waypoint_pub = self.create_publisher(Waypoints, 'waypoints', 10)

        self.lawnmower = SearchPatternGenerator()

    def plan_search_area(self, platform_id, search_area, flight_height=None):
        starting_point = search_area[0] if search_area else None
        waypoints_dict = self.compute_pattern({platform_id: {'area': search_area, 'starting_point': starting_point, 'flight_height': flight_height}})
        if waypoints_dict == {}:
            print("computed empty waypoints")
            return []
        actions = self._converter.convert_plan_patternPath2auspex(platform_id, waypoints_dict[platform_id])
        self.get_logger().info("Successfully planned pattern for search area.")

        return actions

    def compute_pattern(self, assigned_search_areas):

        if assigned_search_areas == {}:
            print("No search area found.")
            return []

        waypoints_dict = {}

        for platform_id, value in assigned_search_areas.items():
            search_area = value.get('area')
            starting_point = value.get('starting_point')
            flight_height = value.get('flight_height')

            platform_capabilities = self._kb_client.query('capabilities', '', 'platform_id', platform_id)
            if platform_capabilities == []:
                print("Can not plan, because respective capabilities are empty.")
                return []

            platform_capabilities = platform_capabilities[0]
            if not starting_point:
                platform_position = self._kb_client.query('platform', 'platform_gps_position', 'platform_id', platform_id)
                starting_point = (float(platform_position['latitude']), float(platform_position['longitude']))

            if platform_capabilities.get('turning_radius'):
                turning_radius = platform_capabilities.get('turning_radius')
            else:
                if platform_capabilities.get('platform_class') and platform_capabilities.get('platform_class').get('value') == 'PLATFORM_CLASS_UL':
                    turning_radius = 100.0
                else:
                    turning_radius = 2.0

            if platform_capabilities.get('sensor_caps') and platform_capabilities.get('sensor_caps')[0].get('fov'):
                fov = platform_capabilities.get('sensor_caps')[0].get('fov_vert_max')
            else:
                if platform_capabilities.get('platform_class') and platform_capabilities.get('platform_class').get('value') == 'PLATFORM_CLASS_UL':
                    fov = math.pi / 6.0 # in radians
                else:
                    fov = 2*math.pi / 9.0 # 40 degrees in radians

            if not flight_height:
                if platform_capabilities.get('platform_class') and platform_capabilities.get('platform_class').get('value') == 'PLATFORM_CLASS_UL':
                    flight_height = 100.0
                else:
                    flight_height = 100.0

            waypoints = self.lawnmower.mow_the_lawn(search_area, starting_point, fov, flight_height, turning_radius)
            waypoints_dict[platform_id] = waypoints
        return waypoints_dict

    def parse_waypoints2actions(self, waypoints_dict, team_id):
        plans = []
        for platform_id, waypoints in waypoints_dict.items():
            if not waypoints:
                print(f"No waypoints found for platform {platform_id}.")
                continue
            plan_msg = Plan()
            plan_msg.platform_id = platform_id
            plan_msg.team_id = team_id
            plan_msg.priority = 0
            plan_msg.tasks = self._converter.convert_plan_pattern2auspex(platform_id, waypoints)
            plan_msg.actions = self._converter.convert_plan_pattern2auspex(platform_id, waypoints)
            plans.append(plan_msg)
        return plans
