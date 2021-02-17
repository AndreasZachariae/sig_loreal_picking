import os
import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    odtf_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            get_package_share_directory('object_detector_tensorflow') + '/launch/detection.launch.py'))

    rieki_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            get_package_share_directory('robot_interface_eki') + '/launch/robot_interface.launch.py'))

    return LaunchDescription([
        odtf_launch,
        rieki_launch,
        Node(
            package='point_transformation',
            node_executable='point_transformation_node',
            node_name='point_transformation_node',
            output='screen'
        )
        # Node(
        #     package='sig_loreal_picking',
        #     node_executable='picking_node',
        #     node_name='picking_node',
        #     output='screen'
        # )
    ])
