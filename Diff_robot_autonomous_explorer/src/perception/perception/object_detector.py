#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from cv_bridge import CvBridge

import cv2
import numpy as np
import math
import torch


class SmartExplorer(Node):

    def __init__(self):
        super().__init__("smart_explorer_visual")

        # Subs
        self.create_subscription(Image, "/camera", self.image_callback, 10)
        self.create_subscription(LaserScan, "/scan", self.scan_callback, 10)
        self.create_subscription(Odometry, "/main_controller/odom", self.odom_callback, 10)

        # Pub
        self.cmd_pub = self.create_publisher(Twist, "/main_controller/cmd_vel", 10)

        self.bridge = CvBridge()

        # ===== MIDAS =====
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.midas = torch.hub.load("intel-isl/MiDaS", "MiDaS_small")
        self.midas.to(self.device)
        self.midas.eval()

        transforms = torch.hub.load("intel-isl/MiDaS", "transforms")
        self.transform = transforms.small_transform

        # ===== STATE =====
        self.left_depth = 1.0
        self.right_depth = 1.0
        self.front_distance = 10.0
        self.ranges = None

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0

        self.state = "MOVING"

        # ===== GRID =====
        self.grid_size = 200
        self.resolution = 0.1
        self.origin = self.grid_size // 2
        self.visited = np.zeros((self.grid_size, self.grid_size))

        # ===== STUCK =====
        self.last_x = None
        self.last_y = None
        self.stuck_time = 0.0
        self.last_time = self.get_clock().now()
        self.stuck_threshold = 5.0

        # ===== TURNING =====
        self.is_turning = False
        self.start_yaw = 0.0
        self.target_rotation = 0.0

        self.best_angle = None

        self.get_logger().info("🚀 VISUAL EXPLORER STARTED")

    # ==========================================
    def set_state(self, new_state):
        if self.state != new_state:
            self.state = new_state
            self.get_logger().info(f"STATE → {self.state}")

    # ==========================================
    def world_to_grid(self, x, y):
        gx = int(x / self.resolution) + self.origin
        gy = int(y / self.resolution) + self.origin
        return gx, gy

    # ==========================================
    def is_direction_unvisited(self, angle):

        for d in np.arange(0.2, 1.5, 0.2):
            x = self.current_x + d * math.cos(angle)
            y = self.current_y + d * math.sin(angle)

            gx, gy = self.world_to_grid(x, y)

            if 0 <= gx < self.grid_size and 0 <= gy < self.grid_size:
                if self.visited[gx, gy] == 1:
                    return False

        return True

    # ==========================================
    def is_direction_free(self, angle):

        if self.ranges is None:
            return False

        n = len(self.ranges)
        idx = int((angle + math.pi) / (2 * math.pi) * n)
        idx = np.clip(idx, 0, n - 1)

        window = self.ranges[max(0, idx-5):min(n, idx+5)]
        window = window[np.isfinite(window)]

        if len(window) == 0:
            return False

        return np.min(window) > 0.6

    # ==========================================
    def score_direction(self, angle):

        if not self.is_direction_free(angle):
            return -1

        score = 5.0

        if self.is_direction_unvisited(angle):
            score += 3.0

        score += (self.left_depth + self.right_depth)

        return score

    # ==========================================
    def find_best_direction(self):

        angles = np.linspace(-math.pi/2, math.pi/2, 21)

        best_score = -999
        best_angle = None

        for angle in angles:
            global_angle = self.current_yaw + angle

            s = self.score_direction(global_angle)

            if s > best_score:
                best_score = s
                best_angle = angle

        self.best_angle = best_angle
        return best_angle

    # ==========================================
    def odom_callback(self, msg):

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        self.current_x = x
        self.current_y = y

        q = msg.pose.pose.orientation
        siny = 2 * (q.w * q.z + q.x * q.y)
        cosy = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny, cosy)

        gx, gy = self.world_to_grid(x, y)
        if 0 <= gx < self.grid_size and 0 <= gy < self.grid_size:
            self.visited[gx, gy] = 1

        # STUCK
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9

        if self.last_x is not None:
            dist = np.hypot(x - self.last_x, y - self.last_y)

            if dist < 0.02:
                self.stuck_time += dt
            else:
                self.stuck_time = 0.0

            if self.stuck_time > self.stuck_threshold and not self.is_turning:

                self.set_state("STUCK")

                best_angle = self.find_best_direction()

                if best_angle is not None:
                    self.target_rotation = best_angle
                else:
                    self.target_rotation = math.radians(90)

                self.start_yaw = self.current_yaw
                self.is_turning = True

        self.last_x = x
        self.last_y = y
        self.last_time = now

    # ==========================================
    def image_callback(self, msg):

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        input_batch = self.transform(img).to(self.device)

        with torch.no_grad():
            prediction = self.midas(input_batch)
            prediction = torch.nn.functional.interpolate(
                prediction.unsqueeze(1),
                size=frame.shape[:2],
                mode="bicubic",
                align_corners=False,
            ).squeeze()

        depth = prediction.cpu().numpy()
        depth = cv2.normalize(depth, None, 0, 1, cv2.NORM_MINMAX)

        h, w = depth.shape
        roi = depth[int(h * 0.6):h, :]

        self.left_depth = np.mean(roi[:, :w//2])
        self.right_depth = np.mean(roi[:, w//2:])

        depth_vis = (depth * 255).astype(np.uint8)
        depth_vis = cv2.applyColorMap(depth_vis, cv2.COLORMAP_MAGMA)

        cv2.putText(depth_vis, f"State: {self.state}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)

        cv2.imshow("Depth Map", depth_vis)
        cv2.waitKey(1)

    # ==========================================
    def scan_callback(self, msg):

        ranges = np.array(msg.ranges)
        ranges[~np.isfinite(ranges)] = np.nan
        self.ranges = ranges

        center = len(ranges)//2
        front = ranges[center-20:center+20]
        front = front[np.isfinite(front)]

        if len(front) > 0:
            self.front_distance = np.min(front)

        self.compute_cmd()

    # ==========================================
    def show_map(self):

        img = (self.visited * 255).astype(np.uint8)
        img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

        # robot position
        gx, gy = self.world_to_grid(self.current_x, self.current_y)
        if 0 <= gx < self.grid_size and 0 <= gy < self.grid_size:
            cv2.circle(img, (gy, gx), 3, (0,0,255), -1)

        # arrows
        angles = np.linspace(-math.pi/2, math.pi/2, 21)

        for angle in angles:
            a = self.current_yaw + angle
            dx = int(10 * math.cos(a))
            dy = int(10 * math.sin(a))

            cv2.arrowedLine(img,
                (gy, gx),
                (gy + dy, gx + dx),
                (255,0,0), 1)

        # best arrow
        if self.best_angle is not None:
            a = self.current_yaw + self.best_angle
            dx = int(15 * math.cos(a))
            dy = int(15 * math.sin(a))

            cv2.arrowedLine(img,
                (gy, gx),
                (gy + dy, gx + dx),
                (0,255,0), 2)

        img = cv2.resize(img, (400,400))
        cv2.imshow("Explorer Map", img)
        cv2.waitKey(1)

    # ==========================================
    def compute_cmd(self):

        cmd = Twist()

        if self.is_turning:
            self.set_state("TURNING")

            diff = self.current_yaw - self.start_yaw

            if abs(diff) < abs(self.target_rotation):
                cmd.angular.z = 0.6 if self.target_rotation > 0 else -0.6
            else:
                self.is_turning = False
                self.stuck_time = 0.0
                self.set_state("MOVING")

            self.cmd_pub.publish(cmd)
            return

        if self.front_distance < 0.5:
            self.set_state("OBSTACLE")

            cmd.linear.x = 0.0
            cmd.angular.z = 0.5

            self.cmd_pub.publish(cmd)
            return

        self.set_state("MOVING")

        cmd.linear.x = 0.4
        cmd.angular.z = (self.left_depth - self.right_depth) * 0.8

        self.cmd_pub.publish(cmd)

        self.show_map()


def main(args=None):
    rclpy.init(args=args)
    node = SmartExplorer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()