#!/usr/bin/env python3
# Author: Addison Sears-Collins
# Date: March 19, 2021
# ROS Version: ROS 2 Foxy Fitzroy
 
# Python math library
import math
 
# ROS client library for Python
import rclpy
 
# Used to create nodes
from rclpy.node import Node
 
# Twist is linear and angular velocity
from geometry_msgs.msg import Twist 
 
# Position, orientation, linear velocity, angular velocity
from nav_msgs.msg import Odometry
 
# Enable use of std_msgs/Float64MultiArray message
from std_msgs.msg import Float64MultiArray 
 
# Scientific computing library for Python
import numpy as np

# For a path message
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header

from visualization_msgs.msg import Marker 
from geometry_msgs.msg import Point

class Estimator(Node):
  """
  Class constructor to set up the node
  """
  def __init__(self):
 
    ############## INITIALIZE ROS PUBLISHERS AND SUBSCRIBERS ######
    super().__init__('Estimator')
 
    # Create a subscriber
    # This node subscribes to messages of type
    # nav_msgs/Odometry (i.e. position and orientation of the robot)
    self.odom_subscriber = self.create_subscription(
                           Odometry,
                           '/odom',
                           self.odom_callback,
                           10)
 
    # Create a subscriber 
    # This node subscribes to messages of type 
    # geometry_msgs/Twist.msg. We are listening to the velocity commands here.
    # The maximum number of queued messages is 10.
    self.velocity_subscriber = self.create_subscription(
                               Twist,
                               '/cmd_vel',
                               self.velocity_callback,
                               10)
 
    # Create a publisher
    # This node publishes the estimated position (x, y, yaw) 
    # The type of message is std_msgs/Float64MultiArray
    self.publisher_state_est = self.create_publisher(
                               Float64MultiArray, 
                               '/state_est', 
                               10)
    

    # Create a publisher for path
    # This node creates a sequence of poses from odometry and published it as a path.
    # The path will be published to topic '/robot_path'. The name of topic is chosen by us.
    # The type of message is nav_msgs/Path
    self.publisher_path = self.create_publisher(Path,'/robot_path',10)

    # Initialize a path
    self.path = Path()
    # The path is described relative to which frame?
    # Rviz can use this to correctly display it. 
    self.path.header.frame_id = 'odom'

    # Create a publisher to show the desired path in Rviz
    self.publisher_desiredpath = self.create_publisher(Marker,'/desired_path',10)
    # Define a path
    self.waypoints = [
      (0.0,0.0,0.0),
      (2.0,0.0,0.0),
      (2.0,2.0,0.0),
      (0.0,2.0,0.0),
      (0.0,0.0,0.0),
    ]
    self.publish_desiredpath()
 
  def publish_desiredpath(self):
    marker = Marker()
    marker.header = Header(stamp=self.get_clock().now().to_msg(),frame_id='odom')
    marker.ns = 'desired_path'
    marker.id = 0
    marker.type = Marker.LINE_STRIP
    marker.action = Marker.ADD
    marker.scale.x = 0.05 # Line width
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0
    marker.color.a = 1.0 # Transparency
    for waypoint in self.waypoints:
      point = Point()
      point.x = waypoint[0]
      point.y = waypoint[1]
      point.z = waypoint[2]
      marker.points.append(point)
    self.publisher_desiredpath.publish(marker)
  


  def odom_callback(self, msg):
    """
    Receive the odometry information containing the position and orientation
    of the robot in the global reference frame. 
    The position is x, y, z.
    The orientation is a x,y,z,w quaternion. 
    """                    
    # Create a stamped pose message
    current_pose = PoseStamped()
    # Add a time stamp and reference frame to the pose message
    current_pose.header = Header(stamp=self.get_clock().now().to_msg(),frame_id=self.path.header.frame_id)
    # Set the pose to the received odometry
    current_pose.pose = msg.pose.pose
    # Append the pose to the existing path
    self.path.poses.append(current_pose)
    # Time stamp the path to indicate the last update
    self.path.header.stamp = self.get_clock().now().to_msg()
    # Publish the path 
    self.publisher_path.publish(self.path)

    # The following part converts the received odometry into pose we can understand
    # and publish it to 'state_estimate' topic
    roll, pitch, yaw = self.euler_from_quaternion(
      msg.pose.pose.orientation.x,
      msg.pose.pose.orientation.y,
      msg.pose.pose.orientation.z,
      msg.pose.pose.orientation.w)    
    obs_state_vector_x_y_yaw = [msg.pose.pose.position.x,msg.pose.pose.position.y,yaw] 
    # Publish the estimated state (x position, y position, yaw angle)
    self.publish_estimated_state(obs_state_vector_x_y_yaw)
  
     
     
  def publish_estimated_state(self, state_vector_x_y_yaw):
    """
    Publish the estimated pose (position and orientation) of the 
    robot to the '/state_est' topic. 
    :param: state_vector_x_y_yaw [x, y, yaw] 
    x is in meters, y is in meters, yaw is in radians
    """
    msg = Float64MultiArray()
    msg.data = state_vector_x_y_yaw
    self.publisher_state_est.publish(msg)
 
  def euler_from_quaternion(self, x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
 
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
 
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
 
    return roll_x, pitch_y, yaw_z # in radians
 
  def velocity_callback(self, msg):
    """
    Listen to the velocity commands (linear forward velocity 
    in the x direction in the robot's reference frame and 
    angular velocity (yaw rate) around the robot's z-axis.
    [v,yaw_rate]
    [meters/second, radians/second]
    """
    # Forward velocity in the robot's reference frame
    v = msg.linear.x
 
    # Angular velocity around the robot's z axis
    yaw_rate = msg.angular.z
 
def main(args=None):
    """
    Entry point for the program.
    """
    # Initialize rclpy library
    rclpy.init(args=args)
 
    # Create the node
    estimator = Estimator()

    
    # Spin the node so the callback function is called.
    # Pull messages from any topics this node is subscribed to.
    # Publish any pending messages to the topics.
    rclpy.spin(estimator)
 
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    estimator.destroy_node()
     
    # Shutdown the ROS client library for Python
    rclpy.shutdown()
 
if __name__ == '__main__':
    main()