import rclpy
from rclpy.node import Node
import rclpy
from geometry_msgs.msg import Twist

from irobot_create_msgs.msg import HazardDetectionVector, HazardDetection
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from visualization_msgs.msg import Marker
from rclpy import qos

from math import cos, sin, pi
from random import uniform

TIMER_INTERVAL = 0.5
BACKUP_TIME = 1.0


class LaserFollow(Node):

    def __init__(self):
        super().__init__('laserfollow')

        self.hazard_subscription = self.create_subscription(
            HazardDetectionVector,
            'zelda/hazard_detection',
            self.hazard_callback,
            qos.qos_profile_sensor_data)
        self.hazard_subscription  # prevent unused variable warning

        self.laser_subscription = self.create_subscription(
            LaserScan,
            'zelda/scan',
            self.laser_callback,
            qos.qos_profile_sensor_data)

        self.laser_subscription  # prevent unused variable warning

        self.marker_publisher = self.create_publisher(
            Marker,
            'zelda/marker',
            qos.qos_profile_sensor_data)

    def hazard_callback(self, haz: HazardDetectionVector):
        print(f"Received hazard: {haz}")

    def laser_callback(self, scan: LaserScan):
        print(f"Received laser: {scan.ranges}")

        for i, dist in enumerate(scan.ranges):
            if dist > scan.range_min and dist < scan.range_max:
                print(f"Distance: {dist}")

        marker = Marker()

        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = "laser"
        marker.id = 0

        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0

        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1

        self.marker_publisher.publish(marker)


def main(args=None):
    rclpy.init(args=args)

    laser_follow = LaserFollow()

    rclpy.spin(laser_follow)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    laser_follow.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
