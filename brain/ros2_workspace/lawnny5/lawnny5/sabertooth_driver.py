import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Bool
from pysabertooth import Sabertooth
from geometry_msgs.msg import Twist

SERIAL_TIMEOUT = 0x0E
RAMPING = 0x10
MOTOR_1_FWD = 0x00
MOTOR_1_REV = 0x01
MOTOR_2_FWD = 0x04
MOTOR_2_REV = 0x05

WHEEL_RADIUS = 0.095  # radius of wheels (meters)
WHEEL_SEPARATION = 0.69  # width of the robot (meters)

def map_val(x, in_min, in_max, out_min, out_max):
    return min(out_max, max(out_min, (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min))

class SabertoothController(Node):

    def __init__(self):
        super().__init__('sabertooth_controller')

        self.e_stop = False
        self.sabertooth_controller = None
        self.failsafe_timer = None

        self.declare_parameter('sabertooth_serial_device', '/dev/ttyS0')
        self.declare_parameter('sabertooth_controller_address', 128)

        self.create_subscription(
            Bool,
            'cmd_estop',
            self.estop_command_callback,
            1)

        self.create_subscription(
            Twist,
            'cmd_motor',
            self.motor_command_callback,
            1)

    def start(self):
        self.initialize_sabertooth(
            self.get_parameter('sabertooth_serial_device').get_parameter_value().string_value,
            self.get_parameter('sabertooth_controller_address').get_parameter_value().integer_value
        )

        self.update_motor_failsafe()
        self.failsafe_timer = self.create_timer(5, self.update_motor_failsafe)

    def stop(self):
        if self.sabertooth_controller:
            self.sabertooth_controller.stop_movement()
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

        self.sabertooth_controller.sendCommand(SERIAL_TIMEOUT, 3)  # 3 = 300ms
        self.sabertooth_controller.sendCommand(RAMPING, 27)

    def motor_command_callback(self, twist_msg):

        # Don't allow any motor commands if e_stop = True
        if self.e_stop:
            self.get_logger().info('Cannot process motor command because of EMERGENCY STOP')
            return

        # read linear and angular velocities from twist message
        lin_speed = twist_msg.linear.x
        ang_speed = twist_msg.angular.z

        # convert linear and angular inputs to left and right wheel velocities
        motor2 = ((2 * lin_speed) - (ang_speed * WHEEL_SEPARATION)) / (
                2.0 * WHEEL_RADIUS
        )

        motor1 = ((2 * lin_speed) + (ang_speed * WHEEL_SEPARATION)) / (
                2.0 * WHEEL_RADIUS
        )

        motor1 = map_val(motor1, -14.736, 14.736, -100.0, 100.0)
        motor2 = map_val(motor2, -14.736, 14.736, -100.0, 100.0)

        # self.get_logger().info("%f,%f : %f,%f" % (lin_speed, ang_speed, motor1, motor2))

        if self.sabertooth_controller:
            self.sabertooth_controller.drive(1, -motor1)
            self.sabertooth_controller.drive(2, motor2)

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

    node = SabertoothController()

    try:
        node.start()
        rclpy.spin(node)
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()