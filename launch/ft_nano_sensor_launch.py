from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace

def generate_launch_description():
    # Declare all launch arguments
    declared_arguments = [
        DeclareLaunchArgument('ip_address', default_value='128.178.145.58'),
        DeclareLaunchArgument('bias', default_value='False'),
        DeclareLaunchArgument('rate', default_value='100'),
        DeclareLaunchArgument('frame_id', default_value='myframe'),
        DeclareLaunchArgument('alpha', default_value='0'),
        DeclareLaunchArgument('rot', default_value='0'),
        DeclareLaunchArgument('scale_x', default_value='1'),
        DeclareLaunchArgument('scale_y', default_value='1'),
        DeclareLaunchArgument('scale_z', default_value='1'),
    ]

    node_args = [
        '--address', LaunchConfiguration('ip_address'),
        '--rate', LaunchConfiguration('rate'),
        '--bias', LaunchConfiguration('bias'),
        '--rot', LaunchConfiguration('rot'),
        '--alpha', LaunchConfiguration('alpha'),
        '--scale_x', LaunchConfiguration('scale_x'),
        '--scale_y', LaunchConfiguration('scale_y'),
        '--scale_z', LaunchConfiguration('scale_z'),
    ]

    sensor_group = GroupAction([
        PushRosNamespace('ft_sensor'),
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

    return LaunchDescription(declared_arguments + [sensor_group])
