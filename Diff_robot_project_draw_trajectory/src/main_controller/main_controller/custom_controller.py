#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import TwistStamped
import numpy as np


from sensor_msgs.msg import JointState 
from rclpy.constants import S_TO_NS
from rclpy.time import Time
import math
from nav_msgs.msg import Odometry
from tf_transformations import quaternion_from_euler


class SimpleController(Node):

    def __init__(self):
        super().__init__("custom_controller")
        self.declare_parameter("wheel_radius", 0.033)  #"wheel_radius" is the parameter name
        self.declare_parameter("wheel_separation", 0.17)

        self.wheel_radius_ = self.get_parameter("wheel_radius").get_parameter_value().double_value
        self.wheel_separation_ = self.get_parameter("wheel_separation").get_parameter_value().double_value

        self.get_logger().info("Using wheel radius %d" % self.wheel_radius_)
        self.get_logger().info("Using wheel separation %d" % self.wheel_separation_)

        
        self.left_wheel_prev_pos_ = 0.0
        self.right_wheel_prev_pos_ = 0.0
        self.prev_time_ = self.get_clock().now()
        self.x_ = 0.0
        self.y_ = 0.0
        self.theta_ = 0.0

        # Fill the Odometry message with invariant parameters
        self.odom_msg_ = Odometry()
        self.odom_msg_.header.frame_id = "odom"
        self.odom_msg_.child_frame_id = "base_footprint"
        self.odom_msg_.pose.pose.orientation.x = 0.0
        self.odom_msg_.pose.pose.orientation.y = 0.0
        self.odom_msg_.pose.pose.orientation.z = 0.0
        self.odom_msg_.pose.pose.orientation.w = 1.0


        self.wheel_cmd_pub_ = self.create_publisher(Float64MultiArray, "simple_velocity_controller/commands", 10)
        self.vel_sub_ = self.create_subscription(TwistStamped, "main_controller/cmd_vel", self.velCallback, 10) #Another ROS2 node 
        #(like a teleop keyboard) publishes a geometry_msgs/msg/Twist message  which is a robot center speed to 
        # the topic /main_controller/cmd_vel ,so this subscriber can read the same message.
        

        self.joint_sub_ = self.create_subscription(JointState,"joint_states", self.jointCallback, 10) 

      #  Before this point, the robot controller could already compute the robot’s pose (position + orientation) and velocity from wheel data.

      #these values were just printed to the terminal for debugging.

     #The improvement here is to publish them as an Odometry message on a ROS 2 topic.

     #By doing this, other nodes (e.g., navigation, SLAM, or visualization tools like RViz) can subscribe to this odometry data in real time.


        self.odom_pub_ = self.create_publisher(Odometry, "main_controller/odom", 10) #section 9, Creating the Odometry Publisher



        self.speed_conversion_ = np.array([[self.wheel_radius_/2, self.wheel_radius_/2],
                                           [self.wheel_radius_/self.wheel_separation_, -self.wheel_radius_/self.wheel_separation_]])
        self.get_logger().info("The conversion matrix is %s" % self.speed_conversion_)








    def velCallback(self, msg):   #The received Twist message is passed as the argument msg.Inside your callback, you can access:

                #msg.linear.x → robot’s forward velocity  "v"

                  #msg.angular.z → robot’s rotation speed  "w" 
        
        robot_speed = np.array([[msg.twist.linear.x],
                                [msg.twist.angular.z]])
        
        # Given v and w, calculate the velocities of the wheels
        wheel_speed = np.matmul(np.linalg.inv(self.speed_conversion_), robot_speed) 

        wheel_speed_msg = Float64MultiArray()
        wheel_speed_msg.data = [wheel_speed[1, 0], wheel_speed[0, 0]]

        self.wheel_cmd_pub_.publish(wheel_speed_msg) #publish wheel speeds using publisher created above on the topic
        #"simple_velocity_controller/commands"


    def jointCallback(self, msg):
        #Implements the inverse differential kinematic model
        # Given the position of the wheels, calculates their velocities
        # then calculates the velocity of the robot wrt the robot frame
        # and then converts it in the global frame and publishes the TF
        dp_left = msg.position[1] - self.left_wheel_prev_pos_
        dp_right = msg.position[0] - self.right_wheel_prev_pos_
        dt = Time.from_msg(msg.header.stamp) - self.prev_time_

        # Actualize the prev pose for the next itheration
        self.left_wheel_prev_pos_ = msg.position[1]
        self.right_wheel_prev_pos_ = msg.position[0]
        self.prev_time_ = Time.from_msg(msg.header.stamp)

       

        # Calculate the rotational speed of each wheel
        fi_left = dp_left / (dt.nanoseconds / S_TO_NS)
        fi_right = dp_right / (dt.nanoseconds / S_TO_NS)

        # Calculate the linear and angular velocity
        linear = (self.wheel_radius_ * fi_right + self.wheel_radius_ * fi_left) / 2
        angular = (self.wheel_radius_ * fi_right - self.wheel_radius_ * fi_left) / self.wheel_separation_


        # Calculate the position increment
        d_s = (self.wheel_radius_ * dp_right + self.wheel_radius_ * dp_left) / 2
        d_theta = (self.wheel_radius_ * dp_right - self.wheel_radius_ * dp_left) / self.wheel_separation_
        self.theta_ += d_theta
        self.x_ += d_s * math.cos(self.theta_)
        self.y_ += d_s * math.sin(self.theta_)


       # self.get_logger().info("linear: %f, angular: %f" % (linear, angular))
       # self.get_logger().info("world_y: %f, world_x: %f" % (self.x_, self.y_))


        # Compose and publish the odom message
        q = quaternion_from_euler(0, 0, self.theta_)
        
        self.odom_msg_.pose.pose.orientation.x = q[0]
        self.odom_msg_.pose.pose.orientation.y = q[1]
        self.odom_msg_.pose.pose.orientation.z = q[2]
        self.odom_msg_.pose.pose.orientation.w = q[3]

        self.odom_msg_.header.stamp = self.get_clock().now().to_msg()

        self.odom_msg_.pose.pose.position.x = self.x_
        self.odom_msg_.pose.pose.position.y = self.y_
        self.odom_msg_.twist.twist.linear.x = linear
        self.odom_msg_.twist.twist.angular.z = angular
        
        self.odom_pub_.publish(self.odom_msg_)

      
      
        




def main():
    rclpy.init()

    custom_controller = SimpleController()
    rclpy.spin(custom_controller)
    
    custom_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
