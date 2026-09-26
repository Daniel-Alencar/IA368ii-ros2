import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Wrench
# Coppelia ZeroMQ Remote API
from coppeliasim_zmqremoteapi_client import RemoteAPIClient


class BumperAndVelocityBridge(Node):
    def __init__(self):
        super().__init__('bumper_and_velocity_bridge')

        # Wheel joints: empty path = search the /myRobot tree for joints named *left*/*right*
        self.declare_parameter('left_motor', '')
        self.declare_parameter('right_motor', '')
        # Wheel base: 0.0 = measure the distance between the two wheel joints in the scene
        self.declare_parameter('wheel_base', 0.0)
        self.declare_parameter('wheel_radius', 0.195 / 2)

        self.bumper_publisher = self.create_publisher(Wrench, '/myRobot/bumper', 10)
        self.subscription = self.create_subscription(Twist, '/myRobot/cmd_vel', self.cmd_vel_callback, 10)

        # Connect to CoppeliaSim
        try:
            self.client = RemoteAPIClient()
            self.sim = self.client.getObject('sim')
            self.robotHandle = self.sim.getObject('/myRobot')
            self.forceSensorHandle = self.sim.getObject('/myRobot/forceSensor')
            self.leftMotor, self.rightMotor = self.find_wheel_joints()

            self.get_logger().info('Connected to CoppeliaSim successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to connect to CoppeliaSim: {e}')
            return

        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_base = self.get_parameter('wheel_base').value
        if self.wheel_base <= 0.0:
            # Position of the left joint in the right joint frame -> axle length
            self.wheel_base = math.dist(self.sim.getObjectPosition(self.leftMotor, self.rightMotor), (0, 0, 0))
        self.get_logger().info(
            f'Wheel joints: {self.sim.getObjectAlias(self.leftMotor, 1)} / '
            f'{self.sim.getObjectAlias(self.rightMotor, 1)}, '
            f'wheel_base={self.wheel_base:.3f} m, wheel_radius={self.wheel_radius:.4f} m')

        if self.sim.getSimulationState() == self.sim.simulation_stopped:
            self.sim.startSimulation()

        # Timer to publish bumper data every 20Hz
        self.timer = self.create_timer(0.05, self.publish_bumper)

        self.get_logger().info('ROS2 ↔ CoppeliaSim Bumper and velocity bridge started.')

    def find_wheel_joints(self):
        left_path = self.get_parameter('left_motor').value
        right_path = self.get_parameter('right_motor').value
        if left_path and right_path:
            return self.sim.getObject(left_path), self.sim.getObject(right_path)

        joints = self.sim.getObjectsInTree(self.robotHandle, self.sim.object_joint_type, 0)
        left = [j for j in joints if 'left' in self.sim.getObjectAlias(j).lower()]
        right = [j for j in joints if 'right' in self.sim.getObjectAlias(j).lower()]
        if len(left) != 1 or len(right) != 1:
            aliases = [self.sim.getObjectAlias(j, 1) for j in joints]
            raise RuntimeError(
                f'could not identify the wheel joints among {aliases}; '
                'set the left_motor and right_motor parameters')
        return left[0], right[0]

    def cmd_vel_callback(self, msg):
        linVel = msg.linear.x
        rotVel = msg.angular.z
        # Inverse kinematics for differential wheeled robot [rad/s]
        rightVel = (linVel + self.wheel_base / 2 * rotVel) / self.wheel_radius
        leftVel = (linVel - self.wheel_base / 2 * rotVel) / self.wheel_radius
        try:
            self.sim.setJointTargetVelocity(self.leftMotor, leftVel)
            self.sim.setJointTargetVelocity(self.rightMotor, rightVel)
        except Exception as e:
            self.get_logger().error(f'Error setting wheel velocities: {e}')

    def publish_bumper(self):
        try:
            result, forceVector, torqueVector = self.sim.readForceSensor(self.forceSensorHandle)
            msg = Wrench()
            msg.force.x, msg.force.y, msg.force.z = (float(f) for f in forceVector)
            msg.torque.x, msg.torque.y, msg.torque.z = (float(t) for t in torqueVector)
            self.bumper_publisher.publish(msg)
        except Exception as e:
            self.get_logger().error(f'Error reading bumper: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = BumperAndVelocityBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
