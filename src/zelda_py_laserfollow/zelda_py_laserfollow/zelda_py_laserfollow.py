import rclpy
from rclpy.node import Node
import rclpy
from geometry_msgs.msg import Twist

from irobot_create_msgs.msg import HazardDetectionVector, HazardDetection
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from visualization_msgs.msg import Marker
from rclpy import qos

from math import cos, sin, pi, degrees
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
        
        self.move_publisher = self.create_publisher(Twist, 'zelda/cmd_vel', 10)
        self.move_timer = self.create_timer(
            TIMER_INTERVAL,
            self.move_timer_callback
        )
    
    def move_timer_callback(self):
        twist = Twist()

        self.get_logger().info(f"moving {self.move_state}")

        if self.move_state == "forward":
            twist.linear.x = 0.1  # m/s
        elif self.move_state == "spin":
            twist.angular.z = 1.0  # rad/s
        else:  # backward
            twist.linear.x = -0.01  # m/s

        self.move_publisher.publish(twist)

    def backup_timer_callback(self):
        self.move_state = "spin"
        self.backup_timer.destroy()

        #min_spin_time = radians(120)
        #max_spin_time = radians(240)

        #spin_time = uniform(min_spin_time, max_spin_time)

        self.spin_timer = self.create_timer(
            spin_time,
            self.spin_timer_callback
        )
    

    def hazard_callback(self, haz: HazardDetectionVector):
        #print(f"Received hazard: {haz}")
        pass

    def laser_callback(self, scan: LaserScan):
        #print(f"Received laser: {scan.ranges}")

        for i, dist in enumerate(scan.ranges):
            
            if dist > scan.range_min and dist < .5:
                x = dist * cos(i * scan.angle_increment)
                y = dist * sin(i * scan.angle_increment)
                #print(f"x: {x}")
                #print(f"y: {y}")

            
            if i/len(scan.ranges) < .25 and i/len(scan.ranges) > 0:
                print(f"angle: {degrees(i*scan.angle_increment)}")

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
