#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
import math
import time


class PID:
    def __init__(self, kp, ki, kd, limit=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.limit = limit

        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_time = None

    def compute(self, error):
        now = time.time()
        if self.prev_time is None:
            dt = 0.1
        else:
            dt = now - self.prev_time
        self.prev_time = now

        # PID terms
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
        self.prev_error = error

        output = self.kp * error + self.ki * self.integral + self.kd * derivative

        if self.limit is not None:
            output = max(-self.limit, min(self.limit, output))

        return output


class GoToGoal(Node):
    def __init__(self):
        super().__init__('go_to_goal')

        # Publishers/Subscribers
        self.cmd_pub = self.create_publisher(Twist, 'main_controller/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, 'main_controller/odom', self.odom_cb, 10)

        # Parameters
        self.declare_parameter('goal_x', 6.0)
        self.declare_parameter('goal_y', -3.0)
        self.declare_parameter('goal_tolerance', 0.5)

        self.goal_x = self.get_parameter('goal_x').value
        self.goal_y = self.get_parameter('goal_y').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value

        # PID controllers
        self.lin_pid = PID(kp=0.35, ki=-0.05, kd=0.06, limit=0.35)

          # angular PID (heading)
        self.ang_pid = PID(kp=0.9, ki=-0.02, kd=0.08, limit=0.9)


        # Robot state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.have_odom = False

        # Control loop
        self.timer = self.create_timer(0.1, self.control_loop)

    def odom_cb(self, msg: Odometry):
        self.have_odom = True
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        # Quaternion -> yaw
        q = msg.pose.pose.orientation
        quat = [q.x, q.y, q.z, q.w]
        _, _, yaw = euler_from_quaternion(quat)
        self.theta = yaw

    def control_loop(self):
        if not self.have_odom:
            return

        dx = self.goal_x - self.x
        dy = self.goal_y - self.y
        rho = math.hypot(dx, dy)  # distance to goal
        angle_to_goal = math.atan2(dy, dx)
        heading_error = math.atan2(math.sin(angle_to_goal - self.theta),
                                   math.cos(angle_to_goal - self.theta))

        cmd = Twist()

        if rho <= self.goal_tolerance:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.cmd_pub.publish(cmd)
            self.get_logger().info("Goal reached ✅")
            return

        # PID control
        v = self.lin_pid.compute(rho)
        w = self.ang_pid.compute(heading_error)

        cmd.linear.x = v
        cmd.angular.z = w

        self.cmd_pub.publish(cmd)

        self.get_logger().info(
            f"rho={rho:.2f}, heading_error={heading_error:.2f}, v={v:.2f}, w={w:.2f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = GoToGoal()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
