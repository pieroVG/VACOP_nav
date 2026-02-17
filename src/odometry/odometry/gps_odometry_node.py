import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import math
import time

R_EARTH = 6378137.0  # mètres


class GPSOdometry(Node):

    def __init__(self):
        super().__init__('gps_odometry')

        self.sub = self.create_subscription(
            NavSatFix,
            '/vacop/gnss/fix', # (sensor_msgs/NavSatFix)
            self.gps_callback,
            10
        )

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.lat0 = None
        self.lon0 = None
        self.last_yaw = 0.0


        self.prev_x = None
        self.prev_y = None
        self.prev_time = None

        self.get_logger().info("GPS Odometry started")

    def gps_callback(self, msg):

        if msg.status.status < 0:
            return  # pas de fix valide

        lat = math.radians(msg.latitude)
        lon = math.radians(msg.longitude)

        if self.lat0 is None:
            self.lat0 = lat
            self.lon0 = lon
            self.get_logger().info("GPS origin initialized")
            return

        # ---- conversion ENU locale ----
        x = (lon - self.lon0) * math.cos(self.lat0) * R_EARTH
        y = (lat - self.lat0) * R_EARTH

        now = self.get_clock().now()
        now_sec = now.nanoseconds * 1e-9

        if self.prev_time is None:
            self.prev_x = x
            self.prev_y = y
            self.prev_time = now_sec
            return

        dt = now_sec - self.prev_time
        if dt <= 0:
            return

        dx = x - self.prev_x
        dy = y - self.prev_y

        v = math.sqrt(dx**2 + dy**2) / dt
        if math.sqrt(dx**2 + dy**2) > 0.05:
            yaw = math.atan2(dy, dx)
        else:
            yaw = self.last_yaw


        self.publish_odometry(x, y, yaw, v, now)

        self.prev_x = x
        self.prev_y = y
        self.prev_time = now_sec

    def publish_odometry(self, x, y, yaw, v, now):

        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)

        msg = Odometry()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_footprint"

        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw

        msg.twist.twist.linear.x = v
        msg.twist.twist.angular.z = 0.0

        self.odom_pub.publish(msg)

        # TF
        tf = TransformStamped()
        tf.header.stamp = now.to_msg()
        tf.header.frame_id = "odom"
        tf.child_frame_id = "base_footprint"
        tf.transform.translation.x = x
        tf.transform.translation.y = y
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(tf)


def main():
    rclpy.init()
    node = GPSOdometry()
    rclpy.spin(node)
    rclpy.shutdown()
