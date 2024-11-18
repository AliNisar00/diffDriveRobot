import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import math

class SquareTrajectoryController(Node):
    def __init__(self):
        super().__init__('square_trajectory_controller')
        # Publisher for velocity commands
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info("Square Trajectory Controller Node has been started.")

    def move_straight(self, speed, duration):
        """Move the robot straight with a given speed for a given duration."""
        twist = Twist()
        twist.linear.x = speed
        twist.angular.z = 0.0
        self.cmd_vel_publisher.publish(twist)
        self.get_logger().info("Moving straight.")
        time.sleep(duration)

    def turn(self, angular_speed, duration):
        """Turn the robot with a given angular speed for a given duration."""
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = angular_speed
        self.cmd_vel_publisher.publish(twist)
        self.get_logger().info("Turning.")
        time.sleep(duration)

    def stop(self):
        """Stop the robot."""
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_publisher.publish(twist)
        self.get_logger().info("Stopping.")

    def execute_trajectory(self):
        """Execute a square trajectory."""
        side_length = 2.0  # meters
        speed = 0.2  # meters per second
        angular_speed = 0.5  # radians per second
        turn_duration = (math.pi/2) / angular_speed  # 90 degrees turn

        for i in range(4):  # Four sides of the square
            self.move_straight(speed, side_length / speed)  # Move forward
            self.stop()  # Stop before turning
            time.sleep(1.0)
            self.turn(angular_speed, turn_duration)  # Turn 90 degrees
            self.stop()  # Stop after turning
            time.sleep(1.0)

        self.get_logger().info("Square trajectory completed.")

def main(args=None):
    rclpy.init(args=args)
    controller = SquareTrajectoryController()
    controller.execute_trajectory()
    controller.stop()  # Ensure the robot is stopped
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
