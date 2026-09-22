from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node

package_name = 'adv_gmsl_camera_baseline'
camera_executable = 'camera_node'
benchmark_executable = 'benchmark_subscriber'

def generate_launch_description():
    # Camera 0
    camera_node0 = Node(
        package=package_name,
        executable=camera_executable,
        name='camera_node_0',
        output='log',
        parameters=[{
            'device': '/dev/video0',
            'width': LaunchConfiguration('width'),
            'height': LaunchConfiguration('height'),
            'fps': ParameterValue(
                LaunchConfiguration('fps'),
                value_type=int,
             ),
             'buffer_count': LaunchConfiguration('buffer_count'),
             'topic': '/camera0/image_raw',
        }],
    )

    subscriber_node0 = Node(
        package=package_name,
        executable=benchmark_executable,
        name='benchmark_subscriber_0',
        output='screen',
        parameters=[{
            'topic': '/camera0/image_raw',
            'target_fps': ParameterValue(
                LaunchConfiguration('fps'),
                value_type=float,
            ),
            'report_every_n': 100,
            'csv_output': PathJoinSubstitution([
                LaunchConfiguration('output'),
                'camera0_benchmark.csv',
            ]),
        }],
    )
        
    # ============================================================
    # Camera 1
    # ============================================================
    camera_node1 = Node(
        package=package_name,
        executable=camera_executable,
        name='camera_node_1',
        # output='log',
        parameters=[{
            'device': '/dev/video1',
            'width': LaunchConfiguration('width'),
            'height': LaunchConfiguration('height'),
            'fps': ParameterValue(
                LaunchConfiguration('fps'),
                value_type=int,
            ),
            'buffer_count': LaunchConfiguration('buffer_count'),
            'topic': '/camera1/image_raw',
        }],
    )

    subscriber_node1 = Node(
        package=package_name,
        executable=benchmark_executable,
        name='benchmark_subscriber_1',
        output='screen',
        parameters=[{
            'topic': '/camera1/image_raw',
            'target_fps': ParameterValue(
                LaunchConfiguration('fps'),
                value_type=float,
            ),
            'report_every_n': 100,
            'csv_output': PathJoinSubstitution([
                LaunchConfiguration('output'),
                'camera1_benchmark.csv',
            ]),
        }],
    )

    # ============================================================
    # Camera 3
    # ============================================================
    camera_node2= Node(
        package=package_name,
        executable=camera_executable,
        name='camera_node_2',
        # output='log',
        parameters=[{
            'device': '/dev/video2',
            'width': LaunchConfiguration('width'),
            'height': LaunchConfiguration('height'),
            'fps': ParameterValue(
                LaunchConfiguration('fps'),
                value_type=int,
            ),
            'buffer_count': LaunchConfiguration('buffer_count'),
            'topic': '/camera2/image_raw',
        }],
    )

    subscriber_node2 = Node(
        package=package_name,
        executable=benchmark_executable,
        name='benchmark_subscriber_2',
        output='screen',
        parameters=[{
                'topic': '/camera2/image_raw',
                'target_fps': ParameterValue(
                    LaunchConfiguration('fps'),
                    value_type=float,
                ),
                'report_every_n': 100,
                'csv_output': PathJoinSubstitution([
                    LaunchConfiguration('output'),
                    'camera2_benchmark.csv',
                ]),
        }],
    )
    # ============================================================
    # Camera 4
    # ============================================================
    camera_node3 = Node(
        package=package_name,
        executable=camera_executable,
        name='camera_node_3',
        # output='log',
        parameters=[{
            'device': '/dev/video3',
            'width': 2880,
            'height': 1860,
            'fps': ParameterValue(
                LaunchConfiguration('fps'),
                value_type=int,
            ),
            'buffer_count': LaunchConfiguration('buffer_count'),
            'topic': '/camera3/image_raw',
        }],
    )

    subscriber_node3 = Node(
        package=package_name,
        executable=benchmark_executable,
        name='benchmark_subscriber_3',
        output='screen',
        parameters=[{
                'topic': '/camera3/image_raw',
                'target_fps': ParameterValue(
                    LaunchConfiguration('fps'),
                    value_type=float,
                ),
                'report_every_n': 100,
                'csv_output': PathJoinSubstitution([
                    LaunchConfiguration('output'),
                    'camera3_benchmark.csv',
                ]),
        }],
    )
    
    # ============================================================
    # Camera 5
    # ============================================================
    camera_node4 = Node(
        package=package_name,
        executable=camera_executable,
        name='camera_node_4',
        # output='log',
        parameters=[{
            'device': '/dev/video4',
            'width': 3840,
            'height': 2160,
            'fps': ParameterValue(
                LaunchConfiguration('fps'),
                value_type=int,
            ),
            'buffer_count': LaunchConfiguration('buffer_count'),
            'topic': '/camera4/image_raw',
        }],
    )

    subscriber_node4 = Node(
        package=package_name,
        executable=benchmark_executable,
        name='benchmark_subscriber_4',
        output='screen',
        parameters=[{
                'topic': '/camera4/image_raw',
                'target_fps': ParameterValue(
                    LaunchConfiguration('fps'),
                    value_type=float,
                ),
                'report_every_n': 100,
                'csv_output': PathJoinSubstitution([
                    LaunchConfiguration('output'),
                    'camera4_benchmark.csv',
                ]),
        }],
    )
 
    return LaunchDescription([
         DeclareLaunchArgument(
            'output',
            default_value='/tmp',
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
        
        camera_node0,
        subscriber_node0,
        
        camera_node1,
        subscriber_node1,
        
        camera_node2,
        subscriber_node2,
        
        camera_node3,
        subscriber_node3,
        
        camera_node4,
        subscriber_node4,
    ])
