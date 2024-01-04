from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
import xacro


def generate_launch_description():

    pkg_name = 'my_robot'
    file_subpath = 'urdf/my_robot.xacro'

    xacro_file = os.path.join(get_package_share_directory(pkg_name),file_subpath)
    robot_description_raw =xacro.process_file(xacro_file).toxml()
    gazebo_pkg = get_package_share_directory('gazebo_ros')

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output="screen",
        parameters=[{'robot_description':robot_description_raw,
        'use_sim_time':True}]
    )
    
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            gazebo_pkg, 'launch'), '/gazebo.launch.py'
            ])
    )

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description", "-entity", "my_robot_description"],
        output="screen",
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
    )

    return LaunchDescription([ 
        gazebo,
        robot_state_publisher_node,
        spawn_entity,
        rviz_node,])
