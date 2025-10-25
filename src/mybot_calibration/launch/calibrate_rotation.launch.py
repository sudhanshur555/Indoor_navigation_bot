from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    rotations_arg = DeclareLaunchArgument(
        'rotations',
        default_value='1.0',
        description='The number of full 360-degree rotations for the robot to perform.'
    )
    
    rot_speed_arg = DeclareLaunchArgument(
        'rot_speed_rps',
        default_value='0.5',
        description='The speed in radians per second for the robot to rotate.'
    )

    calibration_node = Node(
        package='mybot_calibration',
        executable='calibration_node',
        name='calibration_node',
        output='screen',
        parameters=[{
            'move_type': 'rotate', # Hard-code this to 'rotate' for this launch file
            'rotations': LaunchConfiguration('rotations'),
            'rot_speed_rps': LaunchConfiguration('rot_speed_rps')
        }]
    )

    return LaunchDescription([
        rotations_arg,
        rot_speed_arg,
        calibration_node,
    ])