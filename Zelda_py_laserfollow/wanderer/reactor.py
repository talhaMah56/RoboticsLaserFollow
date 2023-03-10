import rclpy
from rclpy.node import Node
import rclpy
from geometry_msgs.msg import Twist

from irobot_create_msgs.msg import IrIntensityVector
from geometry_msgs.msg import Twist
from rclpy import qos

from math import radians
from random import uniform


FORWARD_DETECTION_THRESHOLD = 2.0
TURNAROUND_THRESHOLD = 100.0
TURNING_POWER = 0.05
TIMER_INTERVAL = 0.5
MAX_TURN_SPEED = 0.5


class Reactor(Node):

    def __init__(self):
        super().__init__('reactor')
        self.ir_subscription = self.create_subscription(
            IrIntensityVector,
            'zelda/ir_intensity',
            self.ir_callback,
            qos.qos_profile_sensor_data)

        self.publisher = self.create_publisher(Twist, 'zelda/cmd_vel', 10)
        self.move_timer = self.create_timer(
            TIMER_INTERVAL,
            self.move_timer_callback
        )
        self.twist = Twist()

        self.spin_timer = None

    def move_timer_callback(self):
        if self.spin_timer is not None:
            self.twist = Twist()
            self.twist.angular.z = 1.0

        self.publisher.publish(self.twist)

    def spin_timer_callback(self):
        self.twist = Twist()
        self.spin_timer.destroy()
        self.spin_timer = None

    def ir_callback(self, ir: IrIntensityVector):
        if self.spin_timer is not None:
            return

        # self.get_logger().info(f"ir_callback: {ir.readings}")

        readings = [float(r.value) for r in ir.readings]

        left_average = sum(readings[:3]) / 3
        middle_average = sum(readings[2:5]) / 3
        right_average = sum(readings[4:]) / 3

        self.get_logger().info(
            f"left: {left_average}, middle: {middle_average}, right: {right_average}")

        difference = right_average - left_average

        if middle_average < FORWARD_DETECTION_THRESHOLD:
            difference = 0
        elif middle_average > TURNAROUND_THRESHOLD:
            # turn around
            self.get_logger().info("turning around")
            self.spin_timer = self.create_timer(
                radians(180),
                self.spin_timer_callback
            )
            return

        self.twist.angular.z = max(
            min(TURNING_POWER * difference, MAX_TURN_SPEED), -MAX_TURN_SPEED)
        self.twist.linear.x = 0.1


def main(args=None):
    rclpy.init(args=args)

    reactor = Reactor()

    rclpy.spin(reactor)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    reactor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
