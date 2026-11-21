import math
import rclpy
from .interface import SensorState
from .interface import Arbiter
from .interface import HAZARD_TOPIC
from .interface import ODOM_TOPIC
from .interface import TELEOP_TOPIC
from .interface import VelocityCommand
from .behaviors import DriveForward
from .behaviors import Undock
from .behaviors import RandomTurn
from .behaviors import AvoidBehavior, EscapeBehavior
from .halt_keyboard import HaltBehavior, KeyboardBehavior
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped
from irobot_create_msgs.msg import HazardDetection, HazardDetectionVector
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

LIDAR_MOUNT_YAW = 1*(math.pi / 2)    # Lidar is mounted faced left, this is the offset to get it facing forward


def yaw_from_quaternion(q):
    "Return heading in radians from an orientation quaternion."
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class RobotController(Node):

    def __init__(self):
        super().__init__('robot_controller')
        
        self.cmd_pub = self.create_publisher(
            TwistStamped,
            '/cmd_vel',
            10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            qos_profile_sensor_data
        )

        self.timer = self.create_timer(
            0.1,
            self.move_forward
        )

        self.get_logger().info("Controller running")

        self.state = SensorState()
        
        self.state.start_time = self.state.now = self.get_clock().now().nanoseconds * 1e-9

        self.arbiter = Arbiter([
            RandomTurn(),
            DriveForward()
        ])

        self.create_subscription(Odometry, ODOM_TOPIC, self.pose_callback, 10)
        self.create_subscription(
            HazardDetectionVector, HAZARD_TOPIC, self.hazard_callback, 10)
        self.create_subscription(Twist, TELEOP_TOPIC, self.teleop_callback, 10)

        self.arbiter.behaviors.insert(0, Undock())
        self.arbiter.behaviors.insert(1, HaltBehavior())
        self.arbiter.behaviors.insert(2, KeyboardBehavior())
        self.arbiter.behaviors.insert(3, EscapeBehavior())
        self.arbiter.behaviors.insert(4, AvoidBehavior())


    def move_forward(self):
        if (self.state.now - float(self.state.start_time) < 2):
            return
        name, command = self.arbiter.select(self.state)
        
        self.get_logger().info(
            f"Active behavior: {name}"
        )
        
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x = command.linear
        msg.twist.angular.z = command.angular

        self.cmd_pub.publish(msg)

    def odom_callback(self, msg):

        self.state.x = msg.pose.pose.position.x
        self.state.y = msg.pose.pose.position.y

    def lidar_callback(self, msg):
        self.state.ranges = list(msg.ranges)
        self.state.angle_min = msg.angle_min
        self.state.angle_increment = msg.angle_increment

        self.state.angle_min = msg.angle_min + LIDAR_MOUNT_YAW

    def pose_callback(self, msg):
        "Add heading and time to the shared state."
        self.state.yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        self.state.now = self.get_clock().now().nanoseconds * 1e-9

    def hazard_callback(self, msg):
        "Set bumped when bumped"
        bumped = any(d.type == HazardDetection.BUMP for d in msg.detections)
        if bumped != self.state.bumped:
            self.get_logger().info(f'bumped: {bumped}')
        self.state.bumped = bumped

    def teleop_callback(self, msg):
        "Store the latest keyboard command and the time it arrived."
        self.state.teleop_cmd = VelocityCommand(
            linear=msg.linear.x, angular=msg.angular.z)
        self.state.teleop_stamp = self.get_clock().now().nanoseconds * 1e-9
        

def main(args=None):
    rclpy.init(args=args)

    node = RobotController()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
