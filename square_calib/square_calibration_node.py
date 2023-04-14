import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class SquareCalibrationNode(Node):
    """Square calibration node

    Based on https://doi.org/10.1080/00423114.2016.1203961
    """

    def __init__(self):
        super().__init__("square_calib_node")

        # PARAMETERS
        # Velocities
        self.declare_parameter("vel_lin", 0.5)  # 0.5 m/s
        self.vel_linear = self.get_parameter("vel_lin").get_parameter_value().double_value

        self.declare_parameter("vel_ang", 0.75)  # 45 deg/s
        self.vel_angular = self.get_parameter("vel_ang").get_parameter_value().double_value

        # DISTANCES
        # Square Length
        self.declare_parameter("dim_length", 4)  # 4m
        self.dim_length = self.get_parameter("dim_length").get_parameter_value().double_value
        # Square Width
        self.declare_parameter("dim_width", 4)  # 4m
        self.dim_width = self.get_parameter("dim_width").get_parameter_value().double_value

        # TIMES
        self.time_length = self.dim_length / self.vel_linear
        self.time_width = self.dim_width / self.vel_linear

        # PUBLISHERS
        # CMD_VEL
        self.cmd_vel_pub = self.create_publisher(Twist, "/square/cmd_vel", 10)


def main(args=None):
    rclpy.init(args=args)
    square_calibration = SquareCalibrationNode()

    rclpy.spin(square_calibration)

    square_calibration.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
