#!/usr/bin/env python3

import json
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare launch arguments
    spot_names_arg = DeclareLaunchArgument(
        'spot_names',
        default_value='["spot", "spot2"]',
        description='List of spot robot names'
    )

    pivot_spot_arg = DeclareLaunchArgument(
        'pivot_spot',
        default_value='spot',
        description='Pivot spot robot for GraphNav localization'
    )

    graph_path_arg = DeclareLaunchArgument(
        'graph_path',
        default_value='~/spot_configs/map/demo',
        description='Path to GraphNav map files'
    )

    update_rate_arg = DeclareLaunchArgument(
        'update_rate',
        default_value='0.1',
        description='Rate (in seconds) for service translator to check for new requests'
    )

    # GraphNav Localization Node
    graphnav_loc_node = Node(
        package='spot_multi',
        executable='spot_graphnav_loc.py',
        name='spot_graphnav_localization',
        output='screen',
        parameters=[{
            'spots': LaunchConfiguration('spot_names'),
            'pivot_spot': LaunchConfiguration('pivot_spot'),
            'graph_path': LaunchConfiguration('graph_path'),
        }]
    )

    # Synchronized Drive Node
    sync_drive_node = Node(
        package='spot_multi',
        executable='spot_sync_drive_position.py',
        name='spot_sync_drive',
        output='screen',
        parameters=[{
            'spot_names': LaunchConfiguration('spot_names'),
        }]
    )

    # Point Cloud Processor Node (C++)
    point_cloud_processor_node = Node(
        package='spot_multi',
        executable='point_cloud_processor',
        name='point_cloud_processor',
        output='screen'
    )

    # Function to create gripper translator nodes for each spot
    def launch_setup(context, *args, **kwargs):
        spot_names_str = context.launch_configurations['spot_names']
        spot_names = json.loads(spot_names_str)
        update_rate = LaunchConfiguration('update_rate')

        translator_nodes = []
        for spot_name in spot_names:
            translator_node = Node(
                package='spot_multi',
                executable='gripper_angle_translator.py',
                name=f'gripper_angle_translator_{spot_name}',
                output='screen',
                parameters=[{
                    'spot_name': spot_name,
                    'update_rate': update_rate,
                }]
            )
            translator_nodes.append(translator_node)

        return translator_nodes

    return LaunchDescription([
        spot_names_arg,
        pivot_spot_arg,
        graph_path_arg,
        update_rate_arg,
        graphnav_loc_node,
        sync_drive_node,
        point_cloud_processor_node,
        OpaqueFunction(function=launch_setup),
    ])
