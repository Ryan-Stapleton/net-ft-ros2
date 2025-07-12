from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace

def generate_launch_description():
    # Declare all launch arguments
    declared_arguments = [
        DeclareLaunchArgument('ip_address_1', default_value='128.178.145.89'),
        DeclareLaunchArgument('ip_address_2', default_value='128.178.145.248'),
        DeclareLaunchArgument('bias', default_value='False'),
        DeclareLaunchArgument('rate', default_value='200'),
        DeclareLaunchArgument('frame_id', default_value='myframe'),
        DeclareLaunchArgument('alpha', default_value='0'),
        DeclareLaunchArgument('rot', default_value='0'),
        DeclareLaunchArgument('scale_x', default_value='1'),
        DeclareLaunchArgument('scale_y', default_value='1'),
        DeclareLaunchArgument('scale_z', default_value='1'),
    ]

    # Node arguments as CLI args (to match your executable's expected CLI)
    node_args = [
        '--address', LaunchConfiguration('ip_address_1'),
        '--rate', LaunchConfiguration('rate'),
        '--bias', LaunchConfiguration('bias'),
        '--rot', LaunchConfiguration('rot'),
        '--alpha', LaunchConfiguration('alpha'),
        '--scale_x', LaunchConfiguration('scale_x'),
        '--scale_y', LaunchConfiguration('scale_y'),
        '--scale_z', LaunchConfiguration('scale_z'),
    ]

    node_args_2 = [
        '--address', LaunchConfiguration('ip_address_2'),
        '--rate', LaunchConfiguration('rate'),
        '--bias', LaunchConfiguration('bias'),
        '--rot', LaunchConfiguration('rot'),
        '--alpha', LaunchConfiguration('alpha'),
        '--scale_x', LaunchConfiguration('scale_x'),
        '--scale_y', LaunchConfiguration('scale_y'),
        '--scale_z', LaunchConfiguration('scale_z'),
    ]

    # Group for left sensor
    left_sensor = GroupAction([
        PushRosNamespace('ft_sensor_left'),
        Node(
            package='netft_rdt_driver',
            executable='netft_node',
            name='force_torque_sensor',
            output='screen',
            parameters=[{'frame_id': LaunchConfiguration('frame_id')}],
            arguments=node_args,
            emulate_tty=True
        )
    ])

    # Group for right sensor
    right_sensor = GroupAction([
        PushRosNamespace('ft_sensor_right'),
        Node(
            package='netft_rdt_driver',
            executable='netft_node',
            name='force_torque_sensor',
            output='screen',
            parameters=[{'frame_id': LaunchConfiguration('frame_id')}],
            arguments=node_args_2,
            emulate_tty=True
        )
    ])

    return LaunchDescription(declared_arguments + [left_sensor, right_sensor])
