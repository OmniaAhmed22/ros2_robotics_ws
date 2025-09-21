#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

class BoxMarker(Node):
    def __init__(self):
        super().__init__('box_marker')

        # Publisher for visualization markers
        self.marker_pub = self.create_publisher(Marker, 'visualization_marker', 10)

        # Timer: publish every 0.5 sec
        self.timer = self.create_timer(0.5, self.publish_marker)

    def publish_marker(self):
        marker = Marker()
        marker.header.frame_id = "map"    # Or "odom" depending on your TF tree
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = "goal_box"
        marker.id = 0
        marker.type = Marker.CUBE   # options: SPHERE, ARROW, CUBE
        marker.action = Marker.ADD

        # Position (center of the cube)
        marker.pose.position.x = 3.0
        marker.pose.position.y = 6.0
        marker.pose.position.z = 0.25   # half the height, so it rests on the ground
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        # Scale (size of the box)
        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 0.5

        # Color (RGBA)
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0   # must be > 0 to be visible

        self.marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = BoxMarker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
