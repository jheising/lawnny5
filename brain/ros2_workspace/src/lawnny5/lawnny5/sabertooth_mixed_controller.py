import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Bool
from pysabertooth import Sabertooth
import time

FORWARD_MIXED = 0x0A # 0x08
REVERSE_MIXED = 0x0B # 0x09
RIGHT_MIXED = 0x09 # 0x0A
LEFT_MIXED = 0x08 # 0x0B
SERIAL_TIMEOUT = 0x0E
RAMPING = 0x10

class SabertoothMixedController(Node):

    def __init__(self):
        super().__init__('sabertooth_mixed_controller')

        self.e_stop = False
        self.sabertooth_controller = None
        self.failsafe_timer = None

        self.declare_parameter('sabertooth_serial_device', '/dev/ttyS0')
        self.declare_parameter('sabertooth_controller_address', 128)

        self.subscription = self.create_subscription(
            Bool,
            'cmd_estop',
            self.estop_command_callback,
            10)

        self.subscription = self.create_subscription(
            Point,
            'cmd_sabertooth_mixed',
            self.motor_command_callback,
            10)

    def start(self):
        self.initialize_sabertooth(
            self.get_parameter('sabertooth_serial_device').get_parameter_value().string_value,
            self.get_parameter('sabertooth_controller_address').get_parameter_value().integer_value
        )

        self.update_motor_failsafe()
        self.failsafe_timer = self.create_timer(5, self.update_motor_failsafe)

    def stop(self):
        if self.sabertooth_controller:
            self.sabertooth_controller.stop()
            self.sabertooth_controller.close()
            self.sabertooth_controller = None

        if self.failsafe_timer:
            self.failsafe_timer.destroy()
            self.failsafe_timer = None

    def estop_command_callback(self, msg):
        if msg.data == True:
            self.get_logger().warn("EMERGENCY STOP")
            self.e_stop = True

    # This function is called every few seconds to make sure the motor has a serial timeout value.
    # The serial timeout value will shut down the motor controller if a command hasn't been received
    # in a certain amount of time.
    def update_motor_failsafe(self):
        if self.e_stop or not self.sabertooth_controller:
            return

        self.sabertooth_controller.sendCommand(SERIAL_TIMEOUT, 2)  # 2 = 200ms
        self.sabertooth_controller.sendCommand(RAMPING, 27)

    def motor_command_callback(self, msg):

        if not self.sabertooth_controller:
            return

        # Don't allow any motor commands if e_stop = True
        if self.e_stop:
            self.get_logger().info('Cannot process motor command because of EMERGENCY STOP')
            return

        # Convert our percentage values to 0 - 127
        x_value = round(msg.x * 127)
        y_value = round(msg.y * 127)

        # self.get_logger().info('x:%d - y:%d' % (x_value, y_value))

        if x_value > 0:
            self.sabertooth_controller.sendCommand(RIGHT_MIXED, abs(x_value))
        else:
            self.sabertooth_controller.sendCommand(LEFT_MIXED, abs(x_value))

        if y_value > 0:
            self.sabertooth_controller.sendCommand(FORWARD_MIXED, abs(y_value))
        else:
            self.sabertooth_controller.sendCommand(REVERSE_MIXED, abs(y_value))

    def initialize_sabertooth(self, device, address):
        self.get_logger().info("Initializing Sabertooth motor controller:\ndevice: %s\naddress: %d" % (device, address))

        self.sabertooth_controller = Sabertooth(
            device,
            9600,
            address,
            0.1,
        )

def main(args=None):
    rclpy.init(args=args)

    sabertooth_controller = SabertoothMixedController()

    sabertooth_controller.start()
    rclpy.spin(sabertooth_controller)
    sabertooth_controller.stop()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    sabertooth_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()