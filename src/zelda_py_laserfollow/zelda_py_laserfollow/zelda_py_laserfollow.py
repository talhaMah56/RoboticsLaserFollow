import rclpy
from rclpy.node import Node
import rclpy
from geometry_msgs.msg import Twist

from irobot_create_msgs.msg import HazardDetectionVector, HazardDetection
from std_msgs.msg import ColorRGBA
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from visualization_msgs.msg import Marker
from rclpy import qos

from math import cos, sin
from sklearn import linear_model

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
        # print(f"Received hazard: {haz}")
        pass

    def laser_callback(self, scan: LaserScan):
        # print(f"Received laser: {scan.ranges}")

        points = []

        for i, dist in enumerate(scan.ranges):

            if dist > scan.range_min and dist < scan.range_max:
                x = dist * cos(i * scan.angle_increment)
                y = dist * sin(i * scan.angle_increment)
                points.append((x, y))

                # print(f"x: {x}")
                # print(f"y: {y}")

        self.show_points(points)

        model = linear_model.RANSACRegressor()
        X = [[x] for x, _ in points]
        Y = [y for _, y in points]

        model.fit(X, Y)

    def show_points(self, points):
        points = points[::3]  # only show every few points

        marker = Marker()

        marker.id = 0
        marker.type = Marker.POINTS
        marker.action = Marker.ADD

        marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)

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
