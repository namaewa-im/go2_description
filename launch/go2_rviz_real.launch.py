from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    urdf_file = PathJoinSubstitution([
        FindPackageShare('go2_description'),
        'urdf',
        'go2_description.urdf'
    ])

    robot_description = ParameterValue(
        Command(['xacro ', urdf_file]),
        value_type=str
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'use_sim_time': False,
            'robot_description': robot_description,
        }],
        output='screen'
    )

    lowstate_to_joint_node = ExecuteProcess(
        cmd=['python3', '/home/tree/go2_lowstate_to_joint.py'],
        output='screen'
    )

    rviz_config = PathJoinSubstitution([
        FindPackageShare('go2_description'),
        'rviz',
        'go2_real.rviz'
    ])

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher_node,
        lowstate_to_joint_node,
        rviz_node
    ])

