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


from nav_msgs.msg import Odometry
from tf_transformations import quaternion_from_euler

from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


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


       #create odometry message object
        self.odom_msg_ = Odometry()
         # Initialize odometry fields
        self.odom_msg_.header.frame_id = "odom"
        self.odom_msg_.child_frame_id = "base_footprint"
        self.odom_msg_.pose.pose.orientation.x = 0.0   #  x y z =(0,0,1)  rotation about z yaw so, q=(w,x,y,z)=(cos(θ​/2),0,0,sin(θ/2​))
        self.odom_msg_.pose.pose.orientation.y = 0.0
        self.odom_msg_.pose.pose.orientation.z = 0.0    # no rotation means θ=0
        self.odom_msg_.pose.pose.orientation.w = 1.0   # (w,x,y,z)  = (1,0,0,0)


        self.broadCaster_ =  TransformBroadcaster(self)
        self.transformStamped_ = TransformStamped()
        self.transformStamped_.header.frame_id = "odom"
        self.transformStamped_.child_frame_id = "base_footprint"

        

    

        self.wheel_cmd_pub_ = self.create_publisher(Float64MultiArray, "simple_velocity_controller/commands", 10)
       
        self.vel_sub_ = self.create_subscription(Twist, "main_controller/cmd_vel", self.velCallback, 10)  #Another ROS2 node 
        #(like a teleop keyboard) publishes a geometry_msgs/msg/Twist message  which is a robot center speed to 
        # the topic /main_controller/cmd_vel ,so this subscriber can read the same message.

        self.joint_sub_ = self.create_subscription(JointState,"joint_states", self.jointCallback, 10) 
        self.odom_pub_ = self.create_publisher(Odometry, "main_controller/odom", 10)


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

        dt_sec = max(dt.nanoseconds / S_TO_NS, 1e-6) #to avoid division by zero error
        #The Duration object has an attribute .nanoseconds, which gives the time difference as an integer number of nanoseconds.
        # Calculate the rotational speed of wheels
        omega_fi_left = delta_fi_left /  dt_sec 
        omega_fi_right = delta_fi_right /  dt_sec #It converts the time difference dt from nanoseconds → seconds.

        # Calculate the linear and angular velocity from forward kinematics
        linear = (self.wheel_radius_ * omega_fi_right + self.wheel_radius_ * omega_fi_left) / 2
        angular = (self.wheel_radius_ * omega_fi_right - self.wheel_radius_ * omega_fi_left) / self.wheel_separation_
        
        # Calculate the position increment
        delta_s = (self.wheel_radius_ * delta_fi_right + self.wheel_radius_ * delta_fi_left) / 2
        delta_theta = (self.wheel_radius_ * delta_fi_right - self.wheel_radius_ * delta_fi_left) / self.wheel_separation_
        self.theta_ += delta_theta
        self.x_ += delta_s * math.cos(self.theta_)
        self.y_ += delta_s * math.sin(self.theta_)


    


        
        q = quaternion_from_euler(0, 0, self.theta_) #converts Euler angles → quaternion representation, Output q is a 4-element array: [x, y, z, w]
        
        #fill the quaternion part in the odom message 
        # nav_msgs/Odometry has a nested message: pose.pose.orientation is the quaternion part

        self.odom_msg_.pose.pose.orientation.x = q[0]
        self.odom_msg_.pose.pose.orientation.y = q[1]
        self.odom_msg_.pose.pose.orientation.z = q[2]
        self.odom_msg_.pose.pose.orientation.w = q[3]

        self.odom_msg_.header.stamp = self.get_clock().now().to_msg()

        #fill the position part in the odom message 
        self.odom_msg_.pose.pose.position.x = self.x_
        self.odom_msg_.pose.pose.position.y = self.y_

        #fill the velocity part in the odom message 
        self.odom_msg_.twist.twist.linear.x = linear
        self.odom_msg_.twist.twist.angular.z = angular
        
        self.odom_pub_.publish(self.odom_msg_)
       #----------------------------------------------------------------------
        self.transformStamped_.header.stamp = self.get_clock().now().to_msg() #Take the current ROS clock time
        self.transformStamped_.transform.translation.x = self.x_
        self.transformStamped_.transform.translation.y = self.y_
        self.transformStamped_.transform.translation.z = 0.0
        self.transformStamped_.transform.rotation.x = q[0]
        self.transformStamped_.transform.rotation.y = q[1]
        self.transformStamped_.transform.rotation.z = q[2]
        self.transformStamped_.transform.rotation.w = q[3]

       # translation.x, y, z → position of the child frame relative to the parent.
       # #rotation.x, y, z, w → orientation of the child frame relative to the parent (as a quaternion).

        self.broadCaster_.sendTransform(self.transformStamped_)

      
      
        




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