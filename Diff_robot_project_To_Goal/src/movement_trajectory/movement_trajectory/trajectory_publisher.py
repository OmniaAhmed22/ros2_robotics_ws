import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


class TrajectoryPublisher(Node):

    def __init__(self):
        super().__init__("trajectory_publisher")
        
        self.odom_sub_ = self.create_subscription(Odometry, "main_controller/odom", self.trajectoryCallback, 10)

        self.pub = self.create_publisher(Path,"main_controller/trajectory", 10)

        self.path = Path()
        self.path.header.frame_id = "odom"
       


    def trajectoryCallback(self,msg: Odometry):
       
        # 1. Create a PoseStamped object
        pose_stamped = PoseStamped()
        
        # 2. Fill the header (time + frame)
        pose_stamped.header.stamp = msg.header.stamp
        pose_stamped.header.frame_id = msg.header.frame_id  # usually "odom"
        
        # 3. Copy the pose from odometry
        pose_stamped.pose = msg.pose.pose
        
        # 4. Append to the Path
        self.path.header = msg.header   # defines the reference frame for the trajectory.
        self.path.poses.append(pose_stamped) #adds each pose (time + position + orientation) into the trajectory
        
        # 5. Publish the Path
        self.pub.publish(self.path)


def main():
    rclpy.init()

    trajectory_publisher = TrajectoryPublisher()
    rclpy.spin(trajectory_publisher)
    
    trajectory_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()