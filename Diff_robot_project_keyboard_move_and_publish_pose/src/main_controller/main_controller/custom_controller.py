#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist
import numpy as np


from sensor_msgs.msg import JointState 
from rclpy.constants import S_TO_NS
from rclpy.time import Time
import math


class SimpleController(Node):

    def __init__(self):
        super().__init__("custom_controller")
        self.declare_parameter("wheel_radius", 0.033)  #"wheel_radius" is the parmeter name
        self.declare_parameter("wheel_separation", 0.17)

        self.wheel_radius_ = self.get_parameter("wheel_radius").get_parameter_value().double_value
        self.wheel_separation_ = self.get_parameter("wheel_separation").get_parameter_value().double_value

        self.get_logger().info("Using wheel radius %d" % self.wheel_radius_)
        self.get_logger().info("Using wheel separation %d" % self.wheel_separation_)

        #Initial wheel velocities
        self.left_wheel_prev_pos = 0.0
        self.right_wheel_prev_pos = 0.0
        self.prev_time_ = self.get_clock().now()
        
        #Initial robot center position
        self.x_ = 0.0
        self.y_ = 0.0
        self.theta_ = 0.0
        

    

        self.wheel_cmd_pub_ = self.create_publisher(Float64MultiArray, "simple_velocity_controller/commands", 10)
       
        self.vel_sub_ = self.create_subscription(Twist, "main_controller/cmd_vel", self.velCallback, 10)  #Another ROS2 node 
        #(like a teleop keyboard) publishes a geometry_msgs/msg/Twist message  which is a robot center speed to 
        # the topic /main_controller/cmd_vel ,so this subscriber can read the same message.

        self.joint_sub_ = self.create_subscription(JointState,"joint_states", self.jointCallback, 10) #section 9


        self.speed_conversion_ = np.array([[self.wheel_radius_/2, self.wheel_radius_/2],
                                           [self.wheel_radius_/self.wheel_separation_, -self.wheel_radius_/self.wheel_separation_]])
        self.get_logger().info("The conversion matrix is %s" % self.speed_conversion_)



    def velCallback(self, msg:Twist):   #The received Twist message is passed as the argument msg.Inside your callback, you can access:

                  #msg.linear.x → robot’s forward velocity  "v"
                  #msg.angular.z → robot’s rotation speed  "w" 
        
        robot_speed = np.array([[msg.linear.x],
                                [msg.angular.z]])
        
        # Given v and w, calculate the velocities of the wheels
        wheel_speed = np.matmul(np.linalg.inv(self.speed_conversion_), robot_speed) 

        wheel_speed_msg = Float64MultiArray()
        wheel_speed_msg.data = [wheel_speed[1, 0], wheel_speed[0, 0]]

        self.wheel_cmd_pub_.publish(wheel_speed_msg) #publish wheel speeds using publisher created above on the topic
        #"simple_velocity_controller/commands"


    def jointCallback(self, msg:JointState):
       
        # Given the position of the wheels captured from encoders, calculates their rotational speed
        # then calculates the velocity of the robot center in the robot frame
        # and then converts it in the global frame.
        # finally calculate the new center position in the global frame

        delta_fi_left = msg.position[1] - self.left_wheel_prev_pos
        delta_fi_right = msg.position[0] - self.right_wheel_prev_pos
        dt = Time.from_msg(msg.header.stamp) - self.prev_time_ #Subtracting gives a rclpy.time.Duration object, represents a time difference.

        
        # update the previous positions
        self.left_wheel_prev_pos = msg.position[1]
        self.right_wheel_prev_pos = msg.position[0]
        self.prev_time_ = Time.from_msg(msg.header.stamp)

       
        #The Duration object has an attribute .nanoseconds, which gives the time difference as an integer number of nanoseconds.
        # Calculate the rotational speed of wheels
        omega_fi_left = delta_fi_left / (dt.nanoseconds / S_TO_NS) 
        omega_fi_right = delta_fi_right / (dt.nanoseconds / S_TO_NS) #It converts the time difference dt from nanoseconds → seconds.

        # Calculate the linear and angular velocity from forward kinematics
        linear = (self.wheel_radius_ * omega_fi_right + self.wheel_radius_ * omega_fi_left) / 2
        angular = (self.wheel_radius_ * omega_fi_right - self.wheel_radius_ * omega_fi_left) / self.wheel_separation_


        # Calculate the position increment
        delta_s = (self.wheel_radius_ * delta_fi_right + self.wheel_radius_ * delta_fi_left) / 2
        delta_theta = (self.wheel_radius_ * delta_fi_right - self.wheel_radius_ * delta_fi_left) / self.wheel_separation_
        self.theta_ += delta_theta
        self.x_ += delta_s * math.cos(self.theta_)
        self.y_ += delta_s * math.sin(self.theta_)


        self.get_logger().info("linear: %f, angular: %f" % (linear, angular))
        self.get_logger().info("world_y: %f, world_x: %f" % (self.x_, self.y_))


        
      
      
        




def main():
    rclpy.init()

    custom_controller = SimpleController()
    rclpy.spin(custom_controller)
    
    custom_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

#-----------------------------------------------------------

#note:
#A JointState message (from sensor_msgs/JointState) looks like:

#std_msgs/Header header
#string[] name
#float64[] position
#float64[] velocity
#float64[] effort


#So inside jointCallback, we can access:

#msg.header.stamp → ROS 2 timestamp

#msg.name → list of joint names

#msg.position → list of joint positions (radians for wheels)

#msg.velocity → list of joint velocities (if available)

#msg.effort → list of torques/forces (if available)