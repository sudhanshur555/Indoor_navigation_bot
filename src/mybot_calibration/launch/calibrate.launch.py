from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    distance_arg = DeclareLaunchArgument(
        'distance_m',
        default_value='2.0',
        description='The distance in meters for the robot to travel.'
    )
    
    speed_arg = DeclareLaunchArgument(
        'speed_mps',
        default_value='0.2',
        description='The speed in meters per second for the robot to travel.'
    )

    calibration_node = Node(
        package='mybot_calibration',
        executable='calibration_node',
        name='calibration_node',
        output='screen',
        parameters=[{
            'distance_m': LaunchConfiguration('distance_m'),
            'speed_mps': LaunchConfiguration('speed_mps')
        }]
    )

    return LaunchDescription([
        distance_arg,
        speed_arg,
        calibration_node,
    ])