"""
This file is based on the sas_kuka_control_template
https://github.com/MarinhoLab/sas_kuka_control_template/blob/main/launch/real_robot_launch.py

Run this script in a different terminal window or tab. Be ready to close this, as this activates the real robot if the
connection is successful.
"""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument


def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument(
                    'sigterm_timeout',
                    default_value='30'
                ),
        Node(
            package='sas_robot_driver_unitree_z1',
            executable='sas_robot_driver_unitree_z1_node',
            name='z1_1',
            namespace="sas_z1",
            parameters=[{
                "robot_name": "z1_1",
                "thread_sampling_time_sec": 0.002,
                "mode": "PositionControl",       
                "gripper_attached": True,
                "verbosity": True,         
                "joint_limits_min": [-150.0, 0.0, -160.0, -80.0, -80.0, -160.0, 0.0],
                "joint_limits_max": [150.0, 180.0, 0.0, 80.0, 80.0, 160.0, -40.0],
            }]
        ),

    ])
