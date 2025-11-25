from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'anomaly_threshold',
            default_value='0.85',
            description='Threshold for anomaly detection confidence'
        ),
        
        Node(
            package='digital_twin_monitoring',
            executable='digital_twin_node',
            name='digital_twin_monitoring',
            output='screen',
            parameters=[{
                'anomaly_threshold': LaunchConfiguration('anomaly_threshold'),
                'robot_description': 'robot_description'
            }]
        ),
        
        # Example: Add your robot state publisher here
        # Node(
        #     package='robot_state_publisher',
        #     executable='robot_state_publisher',
        #     name='robot_state_publisher',
        #     output='screen',
        #     parameters=[{'robot_description': LaunchConfiguration('robot_description')}]
        # ),
    ])
