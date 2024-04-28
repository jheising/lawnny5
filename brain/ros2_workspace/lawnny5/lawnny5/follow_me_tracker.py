import rclpy
from rclpy.lifecycle import Node
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn
from .lib.depthai_hand_tracker.HandController import HandController
from .lib.pid import PID
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import pytweening

HAND_SIZE_IN_M = 0.1778
PERCEIVED_FOCAL_LENGTH = 1303.0
FOLLOW_DISTANCE_IN_M = 1.0
MAX_MOVE_TIMEOUT_NS = 5e+8  # half a second

MOVEMENT_RAMP_TIME_IN_NS = 1.5e+9  # 1.5 seconds

LINEAR_PID_PARAMS = [1.5,  # P
                     0.0,  # I
                     0.0,  # D
                     0.0,  # MIN
                     0.8]  # MAX

ANGULAR_PID_PARAMS = [4.0,  # P
                      1.0,  # I
                      0.0,  # D
                      -1.5,  # MIN
                      1.5]  # MAX


def convert_hand_size_to_y_distance_in_m(rect_width, rect_height):
    pixel_size = max(rect_width, rect_height)
    return (HAND_SIZE_IN_M * PERCEIVED_FOCAL_LENGTH) / pixel_size


def convert_hand_position_to_x_distance_in_m(y_distance_in_m, frame_size_in_px, center_x_in_px):
    center_x = center_x_in_px - (frame_size_in_px / 2.0)
    return ((y_distance_in_m * center_x) / PERCEIVED_FOCAL_LENGTH) * 1.5


def clamp(value, min_val, max_val):
    return min(max_val, max(min_val, value))


class FollowMeTracker(Node):

    def __init__(self):
        super().__init__('follow_me_tracker')

        self.hand_controller = None
        self.cmd_vel_publisher = None
        self.timer = None
        self.last_recognition_event_time = None
        self.last_recognition_start_time = None
        self.twist_cmd = Twist()

        self.linear_pid = PID(LINEAR_PID_PARAMS[0], LINEAR_PID_PARAMS[1], LINEAR_PID_PARAMS[2], 5.0, -5.0)
        self.angular_pid = PID(ANGULAR_PID_PARAMS[0], ANGULAR_PID_PARAMS[1], ANGULAR_PID_PARAMS[2], 5.0, -5.0)

        self.create_subscription(
            String,
            '/follow_me_tracker/pid/linear',
            self.set_linear_pid,
            1)

        self.create_subscription(
            String,
            '/follow_me_tracker/pid/angular',
            self.set_angular_pid,
            1)

    def set_linear_pid(self, msg):
        self.get_logger().info("Setting new Linear PID values: %s" % msg.data)
        components = msg.data.split(",")
        self.linear_pid.set_gains(float(components[0]), float(components[1]), float(components[2]), float(components[3]), float(components[4]))
        self.linear_pid.reset()

    def set_angular_pid(self, msg):
        self.get_logger().info("Setting new Angular PID values: %s" % msg.data)
        components = msg.data.split(",")
        self.angular_pid.set_gains(float(components[0]), float(components[1]), float(components[2]), float(components[3]), float(components[4]))
        self.angular_pid.reset()

    def reset_movement(self):
        self.publish_twist(0.0, 0.0)
        self.linear_pid.reset()
        self.angular_pid.reset()
        self.last_recognition_start_time = None

    def tick(self):

        if self.hand_controller:
            self.hand_controller.tick()

        now = self.get_clock().now().nanoseconds

        # If it's been a while since our last hand recognition, stop the robot and reset everything
        if self.last_recognition_event_time and (now - self.last_recognition_event_time) >= MAX_MOVE_TIMEOUT_NS and (self.twist_cmd.angular.z != 0.0 or self.twist_cmd.linear.x != 0.0):
            self.reset_movement()

        return

    def publish_twist(self, linear_velocity, angular_velocity):
        self.twist_cmd.angular.z = angular_velocity
        self.twist_cmd.linear.x = linear_velocity

        if self.cmd_vel_publisher:
            self.cmd_vel_publisher.publish(self.twist_cmd)

    def process_movement(self, event):

        now = self.get_clock().now().nanoseconds
        self.last_recognition_event_time = now

        if self.last_recognition_start_time is None:
            self.last_recognition_start_time = now

        hand_y_coord_in_meters = convert_hand_size_to_y_distance_in_m(event.hand.rect_w_a, event.hand.rect_h_a)
        hand_x_coord_in_meters = convert_hand_position_to_x_distance_in_m(hand_y_coord_in_meters, self.hand_controller.tracker.frame_size, event.hand.rect_x_center_a)

        linear_error = -(hand_y_coord_in_meters - FOLLOW_DISTANCE_IN_M)
        angular_error = -hand_x_coord_in_meters

        linear_velocity = clamp(self.linear_pid.update_PID(linear_error), LINEAR_PID_PARAMS[3], LINEAR_PID_PARAMS[4])
        angular_velocity = clamp(self.angular_pid.update_PID(angular_error), ANGULAR_PID_PARAMS[3], ANGULAR_PID_PARAMS[4])

        # Ramp our movement if there is a large amount of error to start with— this prevents jerking.
        ramp_time_percentage = (now - self.last_recognition_start_time) / MOVEMENT_RAMP_TIME_IN_NS
        if ramp_time_percentage <= 1.0:
            ramp_multiplier = pytweening.easeInOutQuad(ramp_time_percentage)
            if abs(linear_velocity) >= LINEAR_PID_PARAMS[4] * 0.25:
                self.get_logger().info("Ramping linear: %s" % ramp_multiplier)
                linear_velocity = linear_velocity * ramp_multiplier
            if abs(angular_velocity) >= ANGULAR_PID_PARAMS[4] * 0.25:
                self.get_logger().info("Ramping angular: %s" % ramp_multiplier)
                angular_velocity = angular_velocity * ramp_multiplier

        self.get_logger().info("Dist: %s, AV: %s, LV: %s" % (-(hand_y_coord_in_meters - FOLLOW_DISTANCE_IN_M), angular_velocity, linear_velocity))

        self.publish_twist(linear_velocity, angular_velocity)

    def hand_recognized(self, event):
        if event.hand.gesture == "ONE":
            self.process_movement(event)

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Starting follow_me_tracker...")

        config = {
            'renderer': {'enable': False},
            'pose_actions': [
                {'name': 'ONE', 'pose': 'ONE', 'trigger': 'periodic', 'callback': self.hand_recognized},
                # {'name': 'FIVE', 'pose': 'FIVE', 'trigger': 'periodic', 'callback': self.reset_movement},
            ]
        }

        try:
            self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 1)
            self.reset_movement()
            self.hand_controller = HandController(config)
            self.timer = self.create_timer(0.05, self.tick)
        except Exception as e:
            self.get_logger().error('Unable to create HandController: %s' % e)
            return TransitionCallbackReturn.FAILURE

        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Cleaning up follow_me_tracker...')

        self.reset_movement()

        if self.timer:
            self.destroy_timer(self.timer)
            self.timer = None

        if self.hand_controller:
            del self.hand_controller
            self.hand_controller = None

        if self.cmd_vel_publisher:
            self.destroy_publisher(self.cmd_vel_publisher)
            self.cmd_vel_publisher = None

        return TransitionCallbackReturn.SUCCESS


def main(args=None):
    rclpy.init(args=args)

    node = FollowMeTracker()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
