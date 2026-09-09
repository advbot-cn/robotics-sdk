from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'output',
            default_value='/tmp',
        ),
        DeclareLaunchArgument(
            'device',
            default_value='/dev/video2',
        ),

        DeclareLaunchArgument(
            'width',
            default_value='1920',
        ),

        DeclareLaunchArgument(
            'height',
            default_value='1080',
        ),

        DeclareLaunchArgument(
            'fps',
            default_value='30',
        ),

        DeclareLaunchArgument(
            'buffer_count',
            default_value='4',
        ),

        DeclareLaunchArgument(
            'topic',
            default_value='/camera/image_raw',
        ),

        Node(
            package='adv_gmsl_camera_baseline',
            executable='camera_node',
            name='camera_node',
            #output='log',
            parameters=[{
                'device': LaunchConfiguration('device'),
                'width': LaunchConfiguration('width'),
                'height': LaunchConfiguration('height'),
                'fps': ParameterValue(
                    LaunchConfiguration('fps'),
                    value_type=int,
                ),
                'buffer_count': LaunchConfiguration('buffer_count'),
                'topic': LaunchConfiguration('topic'),
            }],
        ),

        Node(
            package='adv_gmsl_camera_baseline',
            executable='benchmark_subscriber',
            name='benchmark_subscriber',
            output='screen',
            parameters=[{
                'topic': LaunchConfiguration('topic'),
                'target_fps': ParameterValue(
                    LaunchConfiguration('fps'),
                    value_type=float,
                ),
                'report_every_n': 100,
                'csv_output': PathJoinSubstitution([
                    LaunchConfiguration('output'),
                    'camera_benchmark.csv',
                ]),
            }],
        ),
    ])
