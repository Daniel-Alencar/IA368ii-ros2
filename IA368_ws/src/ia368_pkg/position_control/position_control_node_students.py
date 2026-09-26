#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose2D
from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation as R


class SiegwartController(Node):
    def __init__(self):
        super().__init__("siegwart_position_controller")

        # Parameters (declared so they can be set via launch or CLI)
        # Stability (Siegwart): Krho > 0, Kbeta < 0, Kalpha - Krho > 0
        self.declare_parameter("Krho", 0.3)
        self.declare_parameter("Kalpha", 0.8)
        self.declare_parameter("Kbeta", -0.15)
        self.declare_parameter("constantSpeed", 0.1)
        self.declare_parameter("backwardAllowed", False)
        self.declare_parameter("useconstantSpeed", False)
        self.declare_parameter("angle_threshold", 0.1)
        self.declare_parameter("dist_threshold", 0.1)

        self.Krho = self.get_parameter("Krho").value
        self.Kalpha = self.get_parameter("Kalpha").value
        self.Kbeta = self.get_parameter("Kbeta").value
        self.constantSpeed = self.get_parameter("constantSpeed").value
        self.backwardAllowed = self.get_parameter("backwardAllowed").value
        self.useconstantSpeed = self.get_parameter("useconstantSpeed").value
        self.angle_threshold = self.get_parameter("angle_threshold").value
        self.dist_threshold = self.get_parameter("dist_threshold").value

        # State
        self.robot_pose = Pose2D()
        self.goal_pose = Pose2D()
        self.goal_received = False

        # Subscribers
        self.create_subscription(Odometry, "myRobot/odom", self.odom_callback, 10)
        self.create_subscription(Pose2D, "myRobot/goal", self.goal_callback, 10)

        # Publisher
        self.cmd_pub = self.create_publisher(Twist, "myRobot/cmd_vel", 10)

        # Control loop timer
        self.timer = self.create_timer(0.05, self.control_loop)  # 20 Hz

    def normalize_angle(self, angle):
        """Wrap angle to [-pi, pi]."""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def odom_callback(self, msg: Odometry):
        # Extract robot pose (x,y,theta) from Odometry
        self.robot_pose.x = msg.pose.pose.position.x
        self.robot_pose.y = msg.pose.pose.position.y

        # Convert quaternion -> yaw
        q = msg.pose.pose.orientation
        r = R.from_quat([q.x, q.y, q.z, q.w])  # x, y, z, w order
        euler = r.as_euler('xyz')               # returns [roll, pitch, yaw] in radians
        yaw = euler[2]
        self.robot_pose.theta = yaw

    def goal_callback(self, msg: Pose2D):
        self.goal_pose = msg
        self.goal_received = True

    def control_law(self):
        dx = self.goal_pose.x - self.robot_pose.x
        dy = self.goal_pose.y - self.robot_pose.y

        # distance to goal
        rho = math.hypot(dx, dy)  
        theta_goal = math.atan2(dy, dx)
        alpha = self.normalize_angle(theta_goal - self.robot_pose.theta)
        # beta measured w.r.t. the goal orientation (Siegwart assumes goal theta = 0)
        beta = self.normalize_angle(self.goal_pose.theta - self.robot_pose.theta - alpha)

        twist = Twist()
        # Implement the Siegwart controller here.
        # the following paramerters should be used:
        # Task 1:
        # parameters: Kalpha, Kbeta, Krho: controller tuning parameters
        # Task 2:
        # constantSpeed: The speed used when constant speed option is on
        # backwardAllowed: This boolean variable should switch the between the two controllers
        # useConstantSpeed: Turn on constant speed option
        
        # Goal behind the robot (alpha outside (-pi/2, pi/2]): drive backward by
        # redefining the robot heading as theta + pi, so alpha and beta shift by pi
        direction = 1.0
        if self.backwardAllowed and not (-math.pi / 2 < alpha <= math.pi / 2):
            direction = -1.0
            alpha = self.normalize_angle(alpha + math.pi)
            beta = self.normalize_angle(beta + math.pi)

        # Task 1: Siegwart control law
        vu = direction * self.Krho * rho  # [m/s]
        omega = self.Kalpha * alpha + self.Kbeta * beta  # [rad/s]

        # Task 2: constant speed, scaling omega by the same factor as v keeps the
        # path curvature (omega / v) and therefore the same trajectory
        if self.useconstantSpeed and rho > self.dist_threshold:
            scale = self.constantSpeed / (self.Krho * rho)
            vu = direction * self.constantSpeed
            omega *= scale

        twist.linear.x = float(vu)
        twist.angular.z = float(omega)
        return twist, rho

    def control_loop(self):
        if self.goal_received:
            cmd, rho = self.control_law()
            # Stop if close enough to goal
            if rho < self.dist_threshold and abs(self.normalize_angle(self.robot_pose.theta - self.goal_pose.theta)) < self.angle_threshold:
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
            self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = SiegwartController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
