import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # --- Declare Launch Arguments ---
    map_arg = DeclareLaunchArgument(
        "map",
        default_value=os.path.join(
            get_package_share_directory("mybot_mapping"), "maps", "my_map.yaml"),
        description="Full path to map file to load"
    )

    # --- Node and Include Actions ---

    # Start the robot's hardware interface (for ESP32 communication)
    hardware_interface = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("mybot_firmware"),
            "launch",
            "hardware_interface.launch.py"
        ),
    )

    # Start the RPLiDAR driver
    laser_driver = Node(
            package="rplidar_ros",
            executable="rplidar_node",
            name="rplidar_node",
            parameters=[os.path.join(
                get_package_share_directory("mybot_bringup"),
                "config",
                "rplidar_a1.yaml"
            )],
            output="screen"
    )
    
    # Start the robot controllers (diff_drive_controller, joint_state_broadcaster)
    controller = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("mybot_controller"),
            "launch",
            "controller.launch.py"
        ),
        launch_arguments={
            "use_simple_controller": "False",
            "use_python": "False",
            "use_sim_time": "False" # Set to False for real robot
        }.items(),
    )
    
    # Start joystick teleoperation
    joystick = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("mybot_controller"),
            "launch",
            "joystick_teleop.launch.py"
        ),
        launch_arguments={"use_sim_time": "False"}.items()
    )

    # Start the IMU driver
    imu_driver_node = Node(
        package="mybot_firmware",
        executable="mpu6050_driver.py"
    )

    # Start the localization stack (AMCL, map_server) and pass the map argument
    localization = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("mybot_localization"),
            "launch",
            "global_localization.launch.py"
        ),
        launch_arguments={"map": LaunchConfiguration("map")}.items()
    )

    # Start the full navigation stack (Nav2)
    navigation = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("mybot_navigation"),
            "launch",
            "navigation.launch.py"
        ),
        launch_arguments={"use_sim_time": "False"}.items()
    )
    
    return LaunchDescription([
        map_arg, # Add the new map argument to the launch description
        hardware_interface,
        laser_driver,
        controller,
        joystick,
        imu_driver_node,
        localization,
        navigation
    ])
