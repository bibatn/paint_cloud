import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    tuning_file_path = f"{get_package_share_directory('paint_cloud')}/config/tuning.yml"
    calibration_file_path = f"{get_package_share_directory('paint_cloud')}/config/CF.yml"
    lidar_frame = "lidar_top"
    base_frame = "base_link"
    camera_frame = "CF"
    image_topic = "/segmentation_mask"
    cloud_topic = "/lidar_top/points"
    return LaunchDescription([
        Node(
            package='paint_cloud',
            executable='paint_cloud_node',
            parameters=[{"tuning_file_path": tuning_file_path},
            {"calibration_file_path": calibration_file_path},
            {"lidar_frame": lidar_frame},
            {"base_frame": base_frame},
            {"camera_frame": camera_frame},
            {"image_topic": image_topic},
            {"cloud_topic": cloud_topic}],
            output='screen',
        )
    ])
