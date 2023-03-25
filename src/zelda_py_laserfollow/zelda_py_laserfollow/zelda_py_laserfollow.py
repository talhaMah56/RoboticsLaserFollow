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

from math import cos, sin, pi, degrees
from random import uniform

# TODO: These parameters follow the wall tightly, and have a wavy motion.
# We should try and test out a stop_range of 0.5 and wall_follow_dist of 0.4
FORWARD_STOP_RANGE = 0.25
WALL_FOLLOW_DIST = 0.3
TIMER_INTERVAL = 0.1
BACKUP_TIMER = 1.0
SLIGHT_TURN_POWER = 7.0
SLIGHT_TURN_MAX = 0.8
MOVE_SPEED_MS = 0.05
FOLLOW_SIDE = "left"
BACKUP_TIME = 1.0
SPIN_TIMER = 0.5



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

        self.backup_timer = None
        self.spin_timer = None
        self.move_state = "forward"
        self.slight_turn = 0.0
        self.sign = 1 if FOLLOW_SIDE == "right" else -1

        self.move_publisher = self.create_publisher(Twist, 'zelda/cmd_vel', 10)
        self.move_timer = self.create_timer(
            TIMER_INTERVAL,
            self.move_timer_callback
        )

    def move_timer_callback(self):
        twist = Twist()

        self.get_logger().info(f"moving {self.move_state}")

        if self.move_state == "forward":
            twist.linear.x = MOVE_SPEED_MS
        elif self.move_state == "follow":
            twist.linear.x = MOVE_SPEED_MS
            twist.angular.z = self.slight_turn
        elif self.move_state == "align":
            twist.angular.z = self.sign * 1.0  # rad/s
        elif self.move_state == "turn_left":
            twist.angular.z = 1.0  # rad/s
        elif self.move_state == "turn_right":
            twist.angular.z = -1.0  # rad/s
        elif self.move_state == "backward":  # backward
            twist.linear.x = -0.01  # m/s
        else:
            self.get_logger().info("stopping")
            twist.linear.x = 0.0

        self.move_publisher.publish(twist)

    def backup_timer_callback(self):
        self.move_state = "align"
        self.backup_timer.destroy()

        self.spin_timer = self.create_timer(
            SPIN_TIMER,
            self.spin_timer_callback
        )

    

    def spin_timer_callback(self):
        self.move_state = "follow"
        self.spin_timer.destroy()

    def hazard_callback(self, haz: HazardDetectionVector):
        for hazard in haz.detections:
            if hazard.type == HazardDetection.BUMP:
                self.move_state = "stop"

                # stop the robot
                twist = Twist()
                self.move_publisher.publish(twist)

                # start backing up
                self.move_state = "backward"
                self.backup_timer = self.create_timer(
                    BACKUP_TIME, self.backup_timer_callback)

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
        left_points = self.filter_points(scan, [(pi / 4, pi / 2)])
        # right_points = self.filter_points(scan, [(5 * pi / 4, 7 * pi / 4)])
        right_points = self.filter_points(scan, [(3 * pi / 2, 7 * pi / 4)])

        selection_points = right_points if FOLLOW_SIDE == "right" else left_points
        min_dist = min(
            [dist for dist, angle in selection_points], default=scan.range_max)

        if self.move_state == "forward":
            if len(front_points) > 0:
                self.move_state = "align"
                self.get_logger().info("aligning")
        elif self.move_state == "align":
            if len(front_points) == 0:
                self.move_state = "follow"
                self.get_logger().info("following")
        elif self.move_state == "follow":
            difference = self.sign * (WALL_FOLLOW_DIST - min_dist)
            self.slight_turn = min(
                max(difference * SLIGHT_TURN_POWER, -SLIGHT_TURN_MAX), SLIGHT_TURN_MAX)

            self.get_logger().info(
                f"slight turn: {self.slight_turn}, difference: {difference}"
            )

            if len(front_points) > 0:
                self.move_state = "turn_left" if FOLLOW_SIDE == "right" else "turn_right"
                self.get_logger().info("turning")
        elif self.move_state == "turn_left" or self.move_state == "turn_right":
            if len(front_points) == 0:
                self.move_state = "follow"
                self.get_logger().info("following again")

        front_points = self.to_cartesian(front_points)
        selection_points = self.to_cartesian(selection_points)

        self.show_points(front_points, 0, ColorRGBA(
            r=1.0, g=0.0, b=0.0, a=1.0))
        self.show_points(selection_points, 1, ColorRGBA(
            r=0.0, g=1.0, b=0.0, a=1.0))

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
