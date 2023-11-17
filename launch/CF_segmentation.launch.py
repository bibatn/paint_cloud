from launch import LaunchDescription
from launch_ros.actions import Node

import os
import sys

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory

def add_launch(package_name, launch_name):
    return launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(

                os.path.join(get_package_share_directory(package_name), launch_name)
            ))


def generate_launch_description():
    return LaunchDescription([
            add_launch('segmentation_trt', 'launch/CF.launch.py'),
            add_launch('paint_cloud', 'launch/CF.launch.py')
                            ])

