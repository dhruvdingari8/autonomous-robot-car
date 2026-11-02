from copy import error

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose


class SquareDriver(Node):

    def __init__(self):
        super().__init__('square_driver')
        self.publisher = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.step = 0
        self.initial_pose = None
        self.square_drawn = False

        # Subscribe to the /turtle1/pose topic to get the turtle's position and orientation
        self.create_subscription(
            Pose,
            'turtle1/pose',
            self.pose_callback,
            10
        )

    def pose_callback(self, msg):
        # Log the turtle's position and orientation
        self.pose = msg
        self.get_logger().info(f'Position: {self.pose.x}, {self.pose.y}, Orientation: {self.pose.theta}', throttle_duration_sec=1.0)
        if self.initial_pose is None:
            self.initial_pose = msg  # Save the initial pose when first received

    def timer_callback(self):
        if self.square_drawn:
            return  # Stop moving after the square is drawn

        if self.initial_pose is None:
            return;  # Wait until the initial pose is received

        msg = Twist()

        # 2 seconds forward, then 1 second turn left
        phase = self.step % 30
        if phase < 20:
            msg.linear.x = 1.0  # Move forward
        else:
            msg.angular.z = 1.5708  # Turn left (90 degrees in radians)
        self.publisher.publish(msg)
        self.step += 1

        if self.step > 120:  # After 4 sides (4 * 30 steps)
            self.square_drawn = True
            self.get_logger().info('Square drawn! Stopping the turtle.')
            # Stop the turtle
            stop_msg = Twist()
            self.publisher.publish(stop_msg)

        # Calculate the error from the initial position
        if self.initial_pose is not None:
            error_pos = ((self.pose.x - self.initial_pose.x) ** 2 + (self.pose.y - self.initial_pose.y) ** 2) ** 0.5
            error_orient = abs(self.pose.theta - self.initial_pose.theta)
            self.get_logger().info(f'Error from initial position: {error_pos:.2f}')
            self.get_logger().info(f'Error from initial orientation: {error_orient:.2f}')


def main(args=None):
    rclpy.init(args=args)

    square_driver = SquareDriver()

    try:
        rclpy.spin(square_driver)
    except KeyboardInterrupt:
        pass
    finally:
        square_driver.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()