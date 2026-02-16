import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import math
from odometry.odometry import DifferentialOdometry
from odometry.DualMotorController import DualMotorController

class OdometryNode(Node):
    def __init__(self):
        super().__init__('wheel_odometry')

        # -------- PARAMÈTRES --------
        self.declare_parameter('wheel_radius', 0.28)
        self.declare_parameter('wheel_base', 1.2)

        self.declare_parameter('gear_ratio', 1.0)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')

        R = self.get_parameter('wheel_radius').value
        L = self.get_parameter('wheel_base').value
        gear = self.get_parameter('gear_ratio').value

        # ODOMÉTRIE
        self.odom = DifferentialOdometry(
            motors=DualMotorController(), 
            wheel_radius=R,
            wheel_base=L,
            gear_ratio=gear
        )

        # -------- ROS INTERFACES --------
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.timer = self.create_timer(0.02, self.update)  # 50 Hz

        self.get_logger().info("Wheel odometry node started")

    def update(self):
        self.odom.update()
        pose = self.odom.get_pose()

        now = self.get_clock().now().to_msg()

        # ---------- ODOM MSG ----------
        msg = Odometry()
        msg.header.stamp = now
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_footprint'

        msg.pose.pose.position.x = pose["x"]
        msg.pose.pose.position.y = pose["y"]

        qz = math.sin(pose["theta"] / 2.0)
        qw = math.cos(pose["theta"] / 2.0)

        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw

        msg.twist.twist.linear.x = pose["v_linear"]
        msg.twist.twist.angular.z = pose["v_angular"]

        self.odom_pub.publish(msg)

        # ---------- TF ----------
        tf = TransformStamped()
        tf.header.stamp = now
        tf.header.frame_id = 'odom'
        tf.child_frame_id = 'base_footprint'
        tf.transform.translation.x = pose["x"]
        tf.transform.translation.y = pose["y"]
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(tf)

def main():
    rclpy.init()
    node = OdometryNode()
    rclpy.spin(node)
    rclpy.shutdown()
