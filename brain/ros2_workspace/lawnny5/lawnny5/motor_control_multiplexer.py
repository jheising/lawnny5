import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Point

class MotorControlMultiplexer(Node):

    # When set to true it means that the joystick has taken over and only messages received from the joystick
    # will be sent to the motor controller
    joystick_override = True

    def __init__(self):
        super().__init__('motor_control_multiplexer')

        # Listen for joystick commands
        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.joystick_listener_callback,
            10)

        self.sabertooth_mixed_commands_ = self.create_publisher(Point, 'cmd_sabertooth_mixed', 10)

    def joystick_listener_callback(self, msg):
        self.joystick_override = True
        point_message = Point()
        point_message.x = msg.axes[0]
        point_message.y = msg.axes[1]
        self.sabertooth_mixed_commands_.publish(point_message)

def main(args=None):
    rclpy.init(args=args)

    motor_multiplexer = MotorControlMultiplexer()

    rclpy.spin(motor_multiplexer)

    motor_multiplexer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()