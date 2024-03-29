import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy


class MotorControlMultiplexer(Node):
    # When set to true it means that the joystick has taken over and only messages received from the joystick
    # will be sent to the motor controller
    joystick_override = True

    def __init__(self):
        super().__init__('motor_control_multiplexer')

        self.get_logger().info("Starting multiplexer!!!")

        # Listen for cmd_vel commands
        self.cmd_vel_subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.process_cmd_vel_msg,
            1)

        self.joy_cmd_subscription = self.create_subscription(
            Joy,
            'joy',
            self.process_joystick_msg,
            1)

        self.motor_commands = self.create_publisher(Twist, 'cmd_motor', 1)

        # self.WHEEL_RADIUS = 0.095  # radius of wheels (meters)
        # self.WHEEL_SEPARATION = 0.69  # width of the robot (meters)
        self.joystick_override = False

    def process_joystick_msg(self, twist_msg):
        # The joystick has moved, let's move to override mode
        self.joystick_override = True
        self.get_logger().info("joy: %s" % twist_msg)

    def process_cmd_vel_msg(self, twist_msg):
        self.get_logger().info("cmd_vel: %s" % twist_msg)
        # If the joystick has taken command, don't process cmd_vel messages
        # if self.joystick_override:
        #     return
        self.process_twist_msg(twist_msg)

    def process_twist_msg(self, twist_msg):
        self.motor_commands.publish(twist_msg)
        # read linear and angular velocities from twist message
        # lin_speed = twist_msg.linear.x
        # ang_speed = twist_msg.angular.z
        #
        # # convert linear and angular inputs to left and right wheel velocities
        # motor1 = ((2 * lin_speed) - (ang_speed * self.WHEEL_SEPARATION)) / (
        #         2.0 * self.WHEEL_RADIUS
        # )
        #
        # motor2 = ((2 * lin_speed) + (ang_speed * self.WHEEL_SEPARATION)) / (
        #         2.0 * self.WHEEL_RADIUS
        # )
        #
        # # map values obtained above between [-70 , 70] out of [-100,100]
        # point_msg = Point()
        # point_msg.x = map_val(motor1, -14.736, 14.736, -100.0, 100.0)
        # point_msg.y = map_val(motor2, -14.736, 14.736, -100.0, 100.0)
        # self.sabertooth_commands.publish(point_msg)


def main(args=None):
    rclpy.init(args=args)

    motor_multiplexer = MotorControlMultiplexer()

    rclpy.spin(motor_multiplexer)

    motor_multiplexer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
