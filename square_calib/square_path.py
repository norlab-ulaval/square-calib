import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class SquarePublisher(Node):
    def __init__(self):
        pass


def main(args=None):
    rclpy.init(args=args)

    square_publisher = SquarePublisher()

    rclpy.spin(square_publisher)

    square_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
