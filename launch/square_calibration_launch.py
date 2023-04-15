from launch_ros.actions import Node

from launch import LaunchDescription


def generate_launch_description():
    square_calibration_node = Node(
        package="square_calib",
        # namespace="square_calibration",
        executable="square_calibration",
        name="square_calib_node",
        output="screen",
        parameters=[
            {"vel_lin": 0.5},
            {"vel_ang": 0.75},
            {"dim_length": 4},
            {"dim_width": 4},
        ],
        remappings=[
            ("/square/cmd_vel", "/nav_vel"),
        ],
    )

    return LaunchDescription(
        [
            square_calibration_node,
        ]
    )
