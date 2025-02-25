#!/usr/bin/env python3
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
import time 

def launch_setup(context, *args, **kwargs):
    pkg_name = "auspex_planning"
    launch_array =  []
    
    print(f"-----------------------------------------------------")
    print(f"Starting planning main...")
    print(f"-----------------------------------------------------")
    planning_main = Node(
        package=pkg_name,
        executable="planning_main_node",
        namespace="",
        emulate_tty=True,
        output='screen',)
    launch_array.append(planning_main)
        
    return launch_array

def generate_launch_description():
    return LaunchDescription(
        [OpaqueFunction(function=launch_setup)]
    )
