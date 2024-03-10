import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Bool
from pysabertooth import Sabertooth

class SabertoothMixedController(Node):

    def __init__(self):
        super().__init__('sabertooth_mixed_controller')

        self.e_stop = False
        self.sabertooth_controller = None

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

        self.initialize_sabertooth(
            self.get_parameter('sabertooth_serial_device').get_parameter_value().string_value,
            self.get_parameter('sabertooth_controller_address').get_parameter_value().integer_value
        )

    def estop_command_callback(self, msg):
        if msg.data == True:
            self.get_logger().warn("EMERGENCY STOP")
            self.e_stop = True

    def motor_command_callback(self, msg):

        # Don't allow any motor commands if e_stop = True
        if self.e_stop:
            self.get_logger().info('Cannot process motor command because of EMERGENCY STOP')
            return

        self.get_logger().info('I heard: "%s"' % msg)

    def initialize_sabertooth(self, device, address):
        # try:
            self.get_logger().info("Initializing Sabertooth motor controller:\ndevice: %s\naddress: %d" % (device, address))

            self.sabertooth_controller = Sabertooth(
                device,
                9600,
                address,
                0.1,
            )
        # except Exception as e:
            # self._loop_rate = self.create_rate(loop_rate, self.get_clock())
            # self.get_logger().error("The was an error")

def main(args=None):
    rclpy.init(args=args)

    sabertooth_controller = SabertoothMixedController()

    rclpy.spin(sabertooth_controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    sabertooth_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()