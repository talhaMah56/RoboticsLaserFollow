import rclpy
from rclpy.node import Node
import rclpy
from geometry_msgs.msg import Twist

from irobot_create_msgs.msg import HazardDetectionVector, HazardDetection
from geometry_msgs.msg import Twist
from rclpy import qos

from math import radians
from random import uniform

TIMER_INTERVAL = 0.5
BACKUP_TIME = 1.0


class Wanderer(Node):

    def __init__(self):
        super().__init__('wanderer')

        self.hazard_subscription = self.create_subscription(
            HazardDetectionVector,
            'zelda/hazard_detection',
            self.hazard_callback,
            qos.qos_profile_sensor_data)
        self.hazard_subscription  # prevent unused variable warning

        self.publisher = self.create_publisher(Twist, 'zelda/cmd_vel', 10)
        self.move_timer = self.create_timer(
            TIMER_INTERVAL,
            self.move_timer_callback
        )
        self.backup_timer = None
        self.spin_timer = None

        self.move_state = "forward"  # forward, spin, backward

    def move_timer_callback(self):
        twist = Twist()

        self.get_logger().info(f"moving {self.move_state}")

        if self.move_state == "forward":
            twist.linear.x = 0.1  # m/s
        elif self.move_state == "spin":
            twist.angular.z = 1.0  # rad/s
        else:  # backward
            twist.linear.x = -0.01  # m/s

        self.publisher.publish(twist)

    def backup_timer_callback(self):
        self.move_state = "spin"
        self.backup_timer.destroy()

        min_spin_time = radians(120)
        max_spin_time = radians(240)

        spin_time = uniform(min_spin_time, max_spin_time)

        self.spin_timer = self.create_timer(
            spin_time,
            self.spin_timer_callback
        )

    def spin_timer_callback(self):
        self.move_state = "forward"
        self.spin_timer.destroy()

    def hazard_callback(self, haz: HazardDetectionVector):
        if self.move_state != "forward":
            return

        # self.get_logger().info('hazard!: "%i"' % len(haz.detections))

        for det in haz.detections:
            if det.type == HazardDetection.BUMP:
                self.get_logger().info(f"bumped")

                # stop the robot
                twist = Twist()
                self.publisher.publish(twist)

                # start backing up
                self.move_state = "backward"
                self.backup_timer = self.create_timer(
                    BACKUP_TIME, self.backup_timer_callback)


def main(args=None):
    rclpy.init(args=args)

    wanderer = Wanderer()

    rclpy.spin(wanderer)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    wanderer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
