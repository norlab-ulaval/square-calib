import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Duration
from std_srvs.srv import Trigger


def wrap_to_pi(angle: float) -> float:
    """Wrap angle to pi

    Args:
        angle (float): Wrap angle between [-pi,pi]

    Returns:
        float: Wrapped angle to pi
    """
    return math.atan2(math.sin(angle), math.cos(angle))


class SquareCalibrationNode(Node):
    """Square calibration node

    Based on https://doi.org/10.1080/00423114.2016.1203961
    """

    def __init__(self, publish_freq=20):
        super().__init__("square_calib_node")

        # PARAMETERS
        # Velocities
        self.declare_parameter("vel_lin", 0.5)  # 0.5 m/s
        self.vel_linear = self.get_parameter("vel_lin").get_parameter_value().double_value

        self.declare_parameter("vel_ang", 0.75)  # 45 deg/s
        self.vel_angular = self.get_parameter("vel_ang").get_parameter_value().double_value

        # DISTANCES
        # Square Length
        self.declare_parameter("dim_length", 4.0)  # 4m
        self.dim_length = self.get_parameter("dim_length").get_parameter_value().double_value
        # Square Width
        self.declare_parameter("dim_width", 4.0)  # 4m
        self.dim_width = self.get_parameter("dim_width").get_parameter_value().double_value

        # TIMES
        self.time_length = Duration(seconds=self.dim_length / self.vel_linear)
        self.time_width = Duration(seconds=self.dim_width / self.vel_linear)
        self.time_rotation = Duration(seconds=math.radians(90) / self.vel_angular)

        publish_freq = 10  # Hz
        self.rate = self.create_rate(publish_freq, self.get_clock())

        # PUBLISHERS
        # CMD_VEL
        self.cmd_vel_pub = self.create_publisher(Twist, "/square/cmd_vel", 10)

        # Services
        # Square Path
        self.square_srv = self.create_service(
            Trigger,
            "start_square_path",
            self.square_path_cback,
        )

        # MESSAGES
        # Linear
        self.linear_twist_msg = Twist()
        self.linear_twist_msg.linear.x = self.vel_linear
        # Angular
        self.angular_twist_msg = Twist()
        self.angular_twist_msg.angular.z = self.vel_angular
        # Stop
        self.stop_twist_msg = Twist()

    def linear_drive(self, linear_time: Duration):
        clock = self.get_clock()
        start_time = clock.now()

        while rclpy.ok() and (clock.now() - start_time) < linear_time:
            self.cmd_vel_pub.publish(self.linear_twist_msg)
            self.rate.sleep()
        self.cmd_vel_pub.publish(self.stop_twist_msg)

    def angular_drive(self, angular_time: Duration):
        clock = self.get_clock()
        start_time = clock.now()

        while rclpy.ok() and (clock.now() - start_time) < angular_time:
            self.cmd_vel_pub.publish(self.angular_twist_msg)
            self.rate.sleep()
        self.cmd_vel_pub.publish(self.stop_twist_msg)

    def square_path_cback(self, request: Trigger.Request, response: Trigger.Response):
        dim_a = self.dim_length
        dim_b = self.dim_width

        message = f"Starting square path with dimensions {dim_a} m and {dim_b} m"
        self.get_logger().info(message)

        # Square
        self.linear_drive(self.time_length)
        self.angular_drive(self.time_rotation)
        self.linear_drive(self.time_width)
        self.angular_drive(self.time_rotation)
        self.linear_drive(self.time_length)
        self.angular_drive(self.time_rotation)
        self.linear_drive(self.time_width)
        self.angular_drive(self.time_rotation)

        response.success = True
        response.message = message

        return response


def main(args=None):
    rclpy.init(args=args)
    square_calibration = SquareCalibrationNode()
    executor = MultiThreadedExecutor()

    try:
        rclpy.spin(square_calibration, executor=executor)
    except KeyboardInterrupt:
        pass
    else:
        rclpy.shutdown()

    # rclpy.spin(square_calibration)
    # square_calibration.destroy_node()
    # rclpy.shutdown()


if __name__ == "__main__":
    main()
