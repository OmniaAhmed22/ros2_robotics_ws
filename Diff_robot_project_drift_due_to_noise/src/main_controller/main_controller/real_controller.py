
 #Original robot model: AntoBrandi, "Self-Driving-and-ROS-2-Learn-by-Doing-Odometry-Control"
 #Source: https://github.com/AntoBrandi/Self-Driving-and-ROS-2-Learn-by-Doing-Odometry-Control
 # Modifications by: Omnia A.Youssef
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np


from sensor_msgs.msg import JointState 
from rclpy.constants import S_TO_NS
from rclpy.time import Time
import math
from nav_msgs.msg import Odometry
from tf_transformations import quaternion_from_euler


from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped



class RealController(Node):

    def __init__(self):
        super().__init__("real_controller")
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

       

        
        self.odom_msg_ = Odometry()
        self.odom_msg_.header.frame_id = "odom"
        self.odom_msg_.child_frame_id = "real_base_footprint"
        self.odom_msg_.pose.pose.orientation.x = 0.0
        self.odom_msg_.pose.pose.orientation.y = 0.0
        self.odom_msg_.pose.pose.orientation.z = 0.0
        self.odom_msg_.pose.pose.orientation.w = 1.0

        # Fill TF message
        self.br_ = TransformBroadcaster(self)
        self.transform_stamped_ = TransformStamped()
        self.transform_stamped_.header.frame_id = "odom"
        self.transform_stamped_.child_frame_id = "real_base_footprint"

        

        
        

        self.joint_sub_ = self.create_subscription(JointState,"joint_states", self.jointCallback, 10) 

      #  Before this point, the robot controller could already compute the robot’s pose (position + orientation) and velocity from wheel data.

      #these values were just printed to the terminal for debugging. #The improvement here is to publish them as an Odometry message on a ROS 2 topic.

     #By doing this, other nodes (e.g., navigation, SLAM, or visualization tools like RViz) can subscribe to this odometry data in real time.


        self.odom_pub_ = self.create_publisher(Odometry, "main_controller/odom_real", 10) 



        self.speed_conversion_ = np.array([[self.wheel_radius_/2, self.wheel_radius_/2],
                                           [self.wheel_radius_/self.wheel_separation_, -self.wheel_radius_/self.wheel_separation_]])
        self.get_logger().info("The conversion matrix is %s" % self.speed_conversion_)







    def jointCallback(self, msg:JointState):
       
        # Given the position of the wheels captured from encoders, calculates their rotational speed
        # then calculates the velocity of the robot center in the robot frame
        # and then converts it in the global frame.
        # finally calculate the new center position in the global frame

        left_encoder = msg.position[1] + np.random.normal(0, 0.005)
        right_encoder = msg.position[0] + np.random.normal(0, 0.005)

        delta_fi_left = left_encoder - self.left_wheel_prev_pos_
        delta_fi_right = right_encoder - self.right_wheel_prev_pos_    

        dt = Time.from_msg(msg.header.stamp) - self.prev_time_ #Subtracting gives a rclpy.time.Duration object, represents a time difference.

        
        # update the previous positions
        self.left_wheel_prev_pos_ = msg.position[1]
        self.right_wheel_prev_pos_ = msg.position[0]
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
        self.odom_msg_.header.stamp = self.get_clock().now().to_msg()
        #fill the quaternion part in the odom message 
        # nav_msgs/Odometry has a nested message: pose.pose.orientation is the quaternion part

        self.odom_msg_.pose.pose.orientation.x = q[0]
        self.odom_msg_.pose.pose.orientation.y = q[1]
        self.odom_msg_.pose.pose.orientation.z = q[2]
        self.odom_msg_.pose.pose.orientation.w = q[3]

        

        #fill the position part in the odom message 
        self.odom_msg_.pose.pose.position.x = self.x_
        self.odom_msg_.pose.pose.position.y = self.y_

        #fill the velocity part in the odom message 
        self.odom_msg_.twist.twist.linear.x = linear
        self.odom_msg_.twist.twist.angular.z = angular
        
        self.odom_pub_.publish(self.odom_msg_)
       #----------------------------------------------------------------------
        # TF
        self.transform_stamped_.transform.translation.x = self.x_
        self.transform_stamped_.transform.translation.y = self.y_
        self.transform_stamped_.transform.rotation.x = q[0]
        self.transform_stamped_.transform.rotation.y = q[1]
        self.transform_stamped_.transform.rotation.z = q[2]
        self.transform_stamped_.transform.rotation.w = q[3]
        self.transform_stamped_.header.stamp = self.get_clock().now().to_msg()
        

       # translation.x, y, z → position of the child frame relative to the parent.
       # #rotation.x, y, z, w → orientation of the child frame relative to the parent (as a quaternion).

        self.br_.sendTransform(self.transform_stamped_)

      
      
        




def main():
    rclpy.init()

    real_controller = RealController()
    rclpy.spin(real_controller)
    
    real_controller.destroy_node()
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