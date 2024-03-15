from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'output_path',
            default_value='/tmp/',
            description='Output path for PNG files'),
        DeclareLaunchArgument(
            'output_prefix',
            default_value='image',
            description='Prefix for output PNG files'),
        DeclareLaunchArgument(
            'compressed',
            default_value='True',
            description='Use Compressed Image Transport'),

        Node(
            package='listener',
            executable='listener_node',
            name='listener_node',
            output='screen',
            parameters=[{
                'output/path': LaunchConfiguration('output_path'),
                'output/prefix': LaunchConfiguration('output_prefix'),
                'compressed': LaunchConfiguration('compressed'),
            }],
        ),
    ])
