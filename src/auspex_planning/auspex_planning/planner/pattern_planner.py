#!/usr/bin/env python3
import json
import math
from rclpy.node import Node
from geographic_msgs.msg import GeoPoint
from .pattern_planner_utils.search_pattern_generator import SearchPatternGenerator
from .planner_base import PlannerBase
from .converter import AUSPEXConverter
from msg_context.loader import Waypoints, Plan


class PatternPlanner(PlannerBase, Node):
    planner_key = 'pattern_planner'

    def __init__(self, kb_client):
        super().__init__('pattern_planner')

        self._kb_client = kb_client

        self._converter = AUSPEXConverter()
        self.waypoint_pub = self.create_publisher(Waypoints, 'waypoints', 10)

        self.lawnmower = SearchPatternGenerator()

    def feedback(self, team_id, feedback_msg):
        pass

    def result(self, team_id, result_msg):
        pass

    def plan_mission(self, team_id):
        print(f"[INFO]: Pattern Planner Selected for team : {team_id}")
        goals = self._kb_client.query(collection='goal', key='team_id', value=team_id)

        if not goals:
            print("No goals found.")
            return []

        search_areas = []
        starting_points = []
        flight_heights = []

        for goal in goals:
            goal_status = goal.get('status', '').upper()
            if goal_status != 'UNPLANNED':
                continue

            if goal['type'] != 'SEARCH':
                continue
            else:
                search_area = []
                goal_id = goal.get('goal_id', '')
                goal_status = goal.get('status', '').upper()
                if goal_status != 'UNPLANNED':
                    continue
                params = goal.get('parameters_json',{})
                if params.get('search_area',{}).get('points', []):
                    search_area_points = params.get('search_area',{}).get('points', [])
                    for point in params.get('search_area',{}).get('points', []):
                        search_area.append((float(point['latitude']), float(point['longitude'])))
                if params.get('starting_point'):
                    starting_point = (float(params['starting_point']['latitude']), float(params['starting_point']['longitude']))
                    starting_points.append(starting_point)
                if params.get('desired_ground_dist'):
                    flight_height = int(params['desired_ground_dist'])
                    flight_heights.append(flight_height)
            if search_area:
                search_areas.append(search_area)

        if search_areas == []:
            print("No search area found.")
            return []

        platform_ids = self._kb_client.query('platform', 'platform_id', 'team_id', team_id)

        # Split Areas and do Assignment
        assigned_search_areas = {}
        for index, search_area in enumerate(search_areas):
            assigned_search_areas[platform_ids[0]['platform_id']] = {'area': search_area, 'starting_point': starting_points[index], 'flight_height': flight_heights[index]}

        waypoints_dict = self.compute_pattern(assigned_search_areas)

        if waypoints_dict == {}:
            print("computed empty waypoints")
            return []

        plans = []

        if team_id == 'ul_team':
            self.publish_waypoints(waypoints_dict)
        else:
            self.publish_waypoints(waypoints_dict)
            plans = self.parse_waypoints2actions(waypoints_dict, team_id)

        self.get_logger().info("Successfully planned waypoints with the pattern planner.")
        return plans

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

    def publish_waypoints(self, waypoints_dict):
        for platform_id, waypoints in waypoints_dict.items():
            msg = Waypoints()
            msg.route_id = platform_id + 'route'
            msg.platform_id = platform_id

            # TODO get highest point in the search area in AGL using copernicus or move to pattern generator
            # highest_area_point = 1000.0
            # waypoint.z is above ground level currently
            for waypoint in waypoints:
                ros_point = GeoPoint(latitude=waypoint.x, longitude=waypoint.y, altitude = waypoint.z)
                msg.points.append(ros_point)

            self.waypoint_pub.publish(msg)
