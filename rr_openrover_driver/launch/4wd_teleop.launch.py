import os

import ament_index_python.packages
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
import launch.substitutions
from launch_ros.actions import Node


def generate_launch_description():
    driver_path = ament_index_python.packages.get_package_share_directory('rr_openrover_driver')
    input_mgr_path = ament_index_python.packages.get_package_share_directory('rr_control_input_manager')
    input_mgr_topics = os.path.join(input_mgr_path, 'config', 'input_topics.yaml')
    twist_mux_locks = os.path.join(driver_path, 'config', 'twist_mux_locks.yaml')
    twist_mux_topics = os.path.join(driver_path, 'config', 'twist_mux_topics.yaml')
    # <arg name="config_locks"  default="$(find rr_openrover_driver)/config/twist_mux_locks.yaml"/>
    # <arg name="config_topics" default="$(find rr_openrover_driver)/config/twist_mux_topics.yaml"/>
    # <rosparam file="$(find rr_control_input_manager)/config/input_topics.yaml" command="load"/>

    return LaunchDescription([
        DeclareLaunchArgument('openrover_node_name',
                              default_value='rr_openrover_driver'),
        Node(
            package='rr_openrover_driver',
            node_executable='openrover_driver_node',
            name=launch.substitutions.LaunchConfiguration('openrover_node_name'),
            output='screen',
            parameters=[{
                'port': '/dev/rover',
                'drive_type': '4wd',
                'enable_timeout': True,
                'timeout': 0.3,
                'closed_loop_control_on': False,
                'total_weight': 20.41,
                'traction_factor': 0.67,
                'odom_covariance_0': 0.01,
                'odom_covariance_35': 0.03,
                'fast_data_rate': 10.0,
                'medium_data_rate': 2.0,
                'slow_data_rate': 1.0,
                'use_legacy': False
            }]
        ),
        Node(
            package='rr_openrover_driver',
            node_executable='diagnostics.py',
            name='rr_openrover_diagnostics_node',
            output='log',
            remappings=[('raw_slow_rate_data', 'raw_slow_rate_data')]
        ),
        Node(
            package='rr_control_input_manager',
            node_executable='xbox_mapper.py',
            name='rr_xbox_mapper_node',
            output='screen',
            remappings=[
                ('joy', '/joystick'),
                ('/joystick/a_button', '/soft_estop/enable'),
                ('/joystick/b_button', '/soft_estop/reset'),
                ('/joystick/y_button', '/joy_priority'),
                ('/joystick/x_button', '/pause_navigation')
            ],
            parameters=[{
                'max_vel_fwd': 0.4,
                'max_vel_turn': 9.0,
                'adjustable_throttle': True,
                'drive_increment': 20,
                'flipper_increment': 20,
                'x_button_toggle': True,
                'y_button_toggle': True
            }]
        ),
        Node(
            package='twist_mux',
            node_executable='twist_mux',
            name='twist_mux',
            output='screen',
            remappings=[
                ('cmd_vel_out', '/cmd_vel/managed')
            ],
            parameters=[twist_mux_topics]
        ),
        Node(
            package='rr_control_input_manager',
            node_executable='control_input_manager.py',
            name='rr_control_input_manager_node',
            output='log',
            parameters=[{
                'driver': 'xboxdrv',
                'wired_or_wireless': 'wireless',
                'control_inputs_file': input_mgr_topics
            }]
        ),
        Node(
            package='joy',
            node_executable='joy_node',
            name='joy_node',
            output='screen',
            remappings=[
                ('/joy', '/joystick')
            ],
            parameters=[{
                'autorepeat_rate': 10.0
            }]
        )
    ])

