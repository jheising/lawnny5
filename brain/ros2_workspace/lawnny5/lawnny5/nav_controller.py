import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition
from rclpy.callback_groups import ReentrantCallbackGroup
from diagnostic_msgs.msg import KeyValue
from lawnny5_interfaces.srv import SetGlobalSetting
from std_msgs.msg import String
from enum import Enum


class NAV_MODE(str, Enum):
    JOYSTICK = 'JOYSTICK'
    FOLLOW_ME = 'FOLLOW_ME'
    DIRECT = 'DIRECT'


class NavController(Node):

    def __init__(self):
        super().__init__('nav_controller')

        self.global_settings_cb_group = ReentrantCallbackGroup()

        # Listen for cmd_vel commands
        self.create_subscription(
            Twist,
            'cmd_vel',
            self.process_cmd_vel_msg,
            1)

        self.create_subscription(
            Joy,
            'joy',
            self.process_joystick_msg,
            1)

        self.create_subscription(
            String,
            'nav_mode',
            self.process_nav_mode_message,
            1)

        self.global_setting_subscription = self.create_subscription(
            KeyValue,
            'global_setting_updated',
            self.process_global_setting_change,
            1, callback_group=self.global_settings_cb_group)

        self.set_global_setting_client = self.create_client(SetGlobalSetting, 'set_setting')

        self.motor_commands = self.create_publisher(Twist, 'cmd_motor', 1)
        self.joy_twist = Twist()

        self.nav_mode = None
        self.nav_mode_client = None
        self.set_nav_mode(NAV_MODE.JOYSTICK)

    def process_nav_mode_message(self, msg):
        self.set_nav_mode(msg.data)

    def process_global_setting_change(self, msg):
        setting_name = msg.key
        setting_value = msg.value

        if setting_name == "nav-mode":
            self.set_nav_mode(setting_value)

    def transition_nav_service_state(self, state):
        req = ChangeState.Request()
        req.transition = Transition(label=state)
        future = self.nav_mode_client.call_async(req)
        # rclpy.spin_until_future_complete(self, future, timeout_sec=10)
        # future.wait(10)
        # self.get_logger().info("finishing 2")
        # return future.result() and future.result().success
        return True

    def shutdown_current_nav_mode(self):
        if self.nav_mode_client:
            self.get_logger().info('Shutting down previous nav mode...')
            self.transition_nav_service_state('cleanup')
            self.destroy_client(self.nav_mode_client)
            self.nav_mode_client = None

    def set_nav_mode(self, nav_mode):

        if nav_mode == self.nav_mode:
            return

        self.nav_mode = nav_mode

        self.shutdown_current_nav_mode()

        self.get_logger().info('Transitioning to Nav Mode: %s' % nav_mode)

        if nav_mode == NAV_MODE.JOYSTICK:
            pass
        elif nav_mode == NAV_MODE.FOLLOW_ME:
            self.nav_mode_client = self.create_client(ChangeState, '/follow_me_tracker/change_state')
        elif nav_mode == NAV_MODE.DIRECT:
            pass
        else:
            self.get_logger().error('No Nav Mode: %s' % nav_mode)

        if self.nav_mode != NAV_MODE.JOYSTICK and self.nav_mode_client:
            result = self.transition_nav_service_state('configure')
            if not result and self.nav_mode:
                self.get_logger().error('Unable to transition to Nav Mode: %s' % nav_mode)
                self.set_nav_mode(NAV_MODE.JOYSTICK)
            else:
                self.get_logger().info('Successfully transitioned to Nav Mode: %s' % nav_mode)

        self.set_global_setting("nav-mode", self.nav_mode, True)

    def set_global_setting(self, name, value, temporary=False):
        req = SetGlobalSetting.Request()
        req.name = name
        req.value = value
        req.temporary = temporary
        self.set_global_setting_client.call_async(req)

    def process_joystick_msg(self, joy_msg):
        # The joystick has moved, let's transition to joystick mode
        self.set_nav_mode(NAV_MODE.JOYSTICK)

        # Turn our joystick position into a twist message
        self.joy_twist.linear.x = joy_msg.axes[1] * 1.4
        self.joy_twist.angular.z = joy_msg.axes[0] * 4.09

        self.publish_cmd_motor(self.joy_twist)

    def process_cmd_vel_msg(self, twist_msg):
        # If we're in joystick nav mode, ignore all cmd_vel messages and only allow controls from the joystick
        if self.nav_mode == NAV_MODE.JOYSTICK:
            return

        self.publish_cmd_motor(twist_msg)

    def publish_cmd_motor(self, twist_msg):
        self.motor_commands.publish(twist_msg)


def main(args=None):
    rclpy.init(args=args)

    node = NavController()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
