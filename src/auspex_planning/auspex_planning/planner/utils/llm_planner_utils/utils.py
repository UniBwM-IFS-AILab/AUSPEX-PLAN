#!/usr/bin/env python3
from fractions import Fraction
from auspex_msgs.msg import ActionInstance, Plan, ActionStatus, PlanStatus
from auspex_planning.planner.utils.converter import enum_to_str
from upf_msgs.msg import (
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
