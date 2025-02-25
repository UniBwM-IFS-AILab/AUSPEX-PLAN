#!/usr/bin/env python3
from fractions import Fraction
from auspex_msgs.msg import ActionInstance, Plan
from up_msgs.msg import (
    Atom,
    Real
)
from enum import Enum
from geographic_msgs.msg import GeoPoint
from math import radians, sin, cos, sqrt, atan2
from itertools import product
import time
import copy
import json
import os
import csv
import re



class DroneStateInfo(Enum):
    LANDED = 1
    AIRBORNE = 2


class DetectedObject():
    def __init__(self, lat, lon, alt, label):
        self._detection_time = time.time()
        self._detection_location = GeoPoint()
        self._detection_specifier = ""
        self._confirmed = False
        self._veloctiy = [0,0,0]
        self._disposable = False

    def set_velocity(self, x, y, z):
        self._velocity[0] = x
        self._velocity[1] = y
        self._velocity[2] = z

class DroneState():

    def __init__(self, airborne_state=DroneStateInfo.LANDED, currentWaypoint="home"):
        self._lat = 0.0
        self._lon = 0.0
        self._alt = 0.0
        self._airborne_state = airborne_state
        self._detected_objects = [] # object -> detection count | object -> detection count | ...
        self._camera = "RGB_CAMERA" # "NO_CAMERA" | "RGB_CAMERA" | "THERMAL_CAMERA"
        self._payload = "FIRST_AID_KIT" # ....

        self._detection_state = ""
        self._currentWaypoint = currentWaypoint
        self._searchedWaypoints = []
        self._battery_state = 1.0

        self._goal_reached = False
        
def display_scores(scores, actions):
    count = 1
    paired_list = list(zip(copy.deepcopy(scores), copy.deepcopy(actions)))

    sorted_list = sorted(paired_list, key=lambda x: x[0], reverse=True)

    for prob, action in sorted_list:
        print(f'{count}.: score: {prob} : for action : {action}')
        count += 1

def fill_and_write_csv(name_list, value_list, file_name):
    data = {}
    for index, name in enumerate(name_list):
       data[name] = value_list[index]
    header = data.keys()
    write_to_csv(data=data, header=header, file_name="output/"+file_name)

def write_to_csv(data, header=["col1", "col2"], file_name='results.csv'):
    file_exists = os.path.isfile(file_name)
    with open(file_name, 'a', newline='') as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=header)
        if not file_exists:
            writer.writeheader()
        writer.writerow(data)
        
def haversine(lat1, lon1, lat2, lon2):
        # Convert latitude and longitude from degrees to radians
        lat1, lon1, lat2, lon2 = map(radians, [lat1, lon1, lat2, lon2])
        
        # Haversine formula
        dlat = lat2 - lat1
        dlon = lon2 - lon1
        a = sin(dlat / 2)**2 + cos(lat1) * cos(lat2) * sin(dlon / 2)**2
        c = 2 * atan2(sqrt(a), sqrt(1 - a))
        
        # Earth radius in meters
        R = 6371000
        return R * c

def is_within_bounds(lat1, lon1, lat2, lon2, distance):
    """
    Check if the point (lat2, lon2) is within a certain distance (in meters) from (lat1, lon1).
    
    Parameters:
    lat1, lon1 : float : Latitude and Longitude of the reference point.
    lat2, lon2 : float : Latitude and Longitude of the point to check.
    distance    : float : Distance in meters to check against.
    
    Returns:
    bool : True if within distance, False otherwise.
    """
    return haversine(lat1, lon1, lat2, lon2) <= distance








def splitActionStringToList(action_string):
    pattern = r"^- \s*(.*)$"
    lines = action_string.strip().split('\n')
    lines = [re.sub(pattern, r'\1', line.strip()) for line in lines]# cut - away 
    
    action_list_with_placeholders = []

    for line in lines:
        left_part, right_part = line.split('|', 1)
        left_part = left_part.strip()
        right_part = right_part.strip()
        #action_description.append(right_part)
        action_list_with_placeholders.append(left_part)
    return action_list_with_placeholders

'''
Generates a list of all possible actions in all possible permutations
'''
def generate_actions(templates, heights, wps, durations, objects):
    generated_actions = []
    
    # Generate all combinations of height and levels
    for height, wp,duration, objects in product(heights, wps, durations, objects):
        for template in templates:
            # Replace placeholders with actual values
            action = template
            action = action.replace('height_in_metres', str(height) )
            action = action.replace('wp', wp)
            action = action.replace('duration', str(duration))
            action = action.replace('object description', objects)
            action = action.replace('operation_area', wp)

            # Add to the list of generated actions
            if action in generated_actions:
                continue
            generated_actions.append(action)
    
    return generated_actions

def getIndexInJoinedString(joined_str, entry):
    for index, val in enumerate(joined_str.split(',')):
        if val == entry:
            return index
    return -1
    
    
def create_up_msg(vhcl_dict, team_id, action_list):
    """
    Creates an ActionInstance message for each action in action_list and assigns a platform name to it
    """
    drone_plans = []
    for index, platform_dict in enumerate(vhcl_dict):
        planned_actions = []
        for id_a, action in enumerate(action_list[index]):
            new_action = ActionInstance()               
            new_action.action_name = action[0]
            
            new_action.id = str(id_a)
            new_action.status = ActionInstance.ACTION_INACTIVE
            
            new_parameters = []
            atom = Atom()
            atom.symbol_atom = [platform_dict['platform_id']]
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
            planned_actions.append(new_action)
        plan_msg = Plan()
        plan_msg.actions = planned_actions
        plan_msg.platform_id = platform_dict['platform_id']
        plan_msg.team_id = team_id
        drone_plans.append(plan_msg)
        
    return drone_plans
        
        


@staticmethod
def load_json(file_path):
    """Load and return JSON data from a file."""
    with open(file_path) as file:
        return json.load(file)

@staticmethod
def load_file_content(file_path):
    """Load and return the content of a file."""
    with open(file_path, 'r') as file:
        return file.read()
