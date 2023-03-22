import rclpy
from rclpy.node import Node
import rclpy
from geometry_msgs.msg import Twist

from irobot_create_msgs.msg import HazardDetectionVector, HazardDetection
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
from rclpy import qos

from math import cos, sin, pi
from random import uniform

FORWARD_STOP_RANGE = 0.5


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
        # print(f"Received hazard: {haz}")
        pass

    def filter_points(self, scan: LaserScan, angle_ranges, range_max=float('inf')):
        polar_points = []
        for i, dist in enumerate(scan.ranges):
            angle = i * scan.angle_increment

            for r in angle_ranges:
                if angle > r[0] and angle < r[1] and dist > scan.range_min and dist < scan.range_max and dist < range_max:
                    polar_points.append((dist, angle))
                    break

        return polar_points

    def laser_callback(self, scan: LaserScan):
        # print(f"Received laser: {scan.ranges}")

        front_points = self.filter_points(
            scan, [(0, pi / 4), (7 * pi / 4, 2 * pi)], FORWARD_STOP_RANGE)
        left_points = self.filter_points(scan, [(pi / 4, 3 * pi / 4)])
        right_points = self.filter_points(scan, [(5 * pi / 4, 7 * pi / 4)])

        self.get_logger().info(
            f"front: {len(front_points)}, left: {len(left_points)}, right: {len(right_points)}")

        front_points = self.to_cartesian(front_points)
        left_points = self.to_cartesian(left_points)
        right_points = self.to_cartesian(right_points)

        self.show_points(front_points, 0, ColorRGBA(
            r=1.0, g=0.0, b=0.0, a=1.0))
        self.show_points(left_points, 1, ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0))
        self.show_points(right_points, 2, ColorRGBA(
            r=1.0, g=0.0, b=1.0, a=1.0))

    def to_cartesian(self, polar_points):
        return [(r * cos(a), r * sin(a)) for r, a in polar_points]

    def show_points(self, points, id=0, color=ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)):
        points = points  # only show every few points

        marker = Marker()
        marker.id = id
        marker.type = Marker.POINTS
        marker.action = Marker.ADD

        marker.color = color

        point_scale = 0.01
        marker.scale.x = point_scale
        marker.scale.y = point_scale

        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.points = [Point(x=x, y=y, z=0.0) for x, y in points]

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
