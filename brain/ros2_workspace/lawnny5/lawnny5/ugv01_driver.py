import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
import serial
import json

WHEEL_RADIUS = 0.095  # radius of wheels (meters)
WHEEL_SEPARATION = 0.69  # width of the robot (meters)
FAILSAFE_TIME_IN_NS = 3e+8  # 300 NS


def map_val(x, in_min, in_max, out_min, out_max):
    return min(out_max, max(out_min, (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min))


class UGV01Driver(Node):

    def __init__(self):
        super().__init__('ugv01_driver')

        self.device = None
        self.e_stop = False
        self.failsafe_timer = None
        self.last_motor_command_time = None

        # /dev/ttyS0 - rpi 4
        # /dev/ttyAMA0 - rpi 5
        self.declare_parameter('ugv01_serial_device', '/dev/ttyS0')

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

    def process_failsafe(self):

        if self.last_motor_command_time is None:
            return

        now = self.get_clock().now().nanoseconds

        if now - self.last_motor_command_time >= FAILSAFE_TIME_IN_NS:
            self.stop_motor()
            self.last_motor_command_time = None

    def start(self):
        self.initialize_serial(self.get_parameter('ugv01_serial_device').get_parameter_value().string_value)
        self.failsafe_timer = self.create_timer(0.3, self.process_failsafe)

    def stop(self):

        if self.failsafe_timer:
            self.destroy_timer(self.failsafe_timer)
            self.failsafe_timer = None

        self.stop_motor()

        if self.device:
            self.device.close()
            self.device = None

    def stop_motor(self):
        self.send_motor_command(0.0, 0.0)

    def estop_command_callback(self, msg):
        if msg.data == True:
            self.stop()
            self.get_logger().warn("EMERGENCY STOP")
            self.e_stop = True

    def send_motor_command(self, motor1, motor2):
        if self.device:
            command = {"T": 1, "L": motor1, "R": motor2}
            self.device.write(json.dumps(command).encode() + b'\n')

    def motor_command_callback(self, twist_msg):

        # Don't allow any motor commands if e_stop = True
        if self.e_stop:
            self.get_logger().info('Cannot process motor command because of EMERGENCY STOP')
            return

        self.last_motor_command_time = self.get_clock().now().nanoseconds

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

        motor1 = map_val(motor1, -14.736, 14.736, -0.5, 0.5)
        motor2 = map_val(motor2, -14.736, 14.736, -0.5, 0.5)

        self.send_motor_command(motor1, motor2)

    def initialize_serial(self, device):
        self.get_logger().info("Initializing UGV-01 motor controller:\ndevice: %s" % (device))
        self.device = serial.Serial(device, baudrate=115200, dsrdtr=None)
        self.device.setRTS(False)
        self.device.setDTR(False)


def main(args=None):
    rclpy.init(args=args)

    node = UGV01Driver()

    try:
        node.start()
        rclpy.spin(node)
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
