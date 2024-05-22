import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import String
import json
from geometry_msgs.msg import Twist
from rclpy_message_converter import message_converter
import pytweening
from benedict import benedict

MOVEMENT_PUBLISH_RATE_IN_MS = 100


def is_number(value):
    return isinstance(value, (int, float))


def tween_object_values(start_object, end_object, tween_value):
    object1_keys = start_object.keys()
    object2_keys = end_object.keys()
    result = {}

    for key in object2_keys:

        obj2_value = end_object[key]
        result[key] = obj2_value

        if key in object1_keys:
            obj1_value = start_object[key]

            if isinstance(obj1_value, dict) and isinstance(obj2_value, dict):
                result[key] = tween_object_values(obj1_value, obj2_value, tween_value)
            elif is_number(obj1_value) and is_number(obj2_value):
                result[key] = obj1_value + ((obj2_value - obj1_value) * tween_value)

    return result


class TopicScripter(Node):

    def __init__(self):
        super().__init__("topic_scripter")

        self.moves = None
        self.current_move_index = -1
        self.move_start_time = None
        self.frame_timer = None

        self.moves_dir = get_package_share_directory('lawnny5') + '/moves'

        self.create_subscription(
            String,
            'motion/play_by_name',
            self.handle_play_move_by_name,
            1)

        self.cmd_vel_publisher = self.create_publisher(Twist, "cmd_vel", 1)
        self.nav_mode_publisher = self.create_publisher(String, "nav_mode", 1)

        self.play_move_by_name("dance1")

    def __del__(self):
        self.stop_movement()

    def handle_play_move_by_name(self, msg):
        self.play_move_by_name(msg.data)

    def play_move_by_name(self, move_name):
        self.play_move_file(self.moves_dir + "/" + move_name + ".json")

    def play_move_file(self, filename):
        file_data = open(filename)
        move_data = json.load(file_data)
        self.play_moves(move_data)

    def stop_movement(self):

        if self.frame_timer:
            self.destroy_timer(self.frame_timer)
            self.frame_timer = None

        self.moves = None
        self.current_move_index = -1
        self.move_start_time = None
        self.publish_twist_movement_dict({"linear": {"x": 0, "y": 0, "z": 0}, "angular": {"x": 0, "y": 0, "z": 0}})
        self.get_logger().info("Movement stopped")

    def publish_twist_movement_dict(self, twist_dict):
        msg = message_converter.convert_dictionary_to_ros_message('geometry_msgs/msg/Twist', twist_dict)
        if self.cmd_vel_publisher:
            self.cmd_vel_publisher.publish(msg)

    def play_moves(self, moves):
        self.stop_movement()
        self.moves = moves
        nav_mode_msg = String()
        nav_mode_msg.data = "DIRECT"
        self.nav_mode_publisher.publish(nav_mode_msg)
        self.play_next_move()

    def start_frame_timer(self):
        if not self.frame_timer:
            self.frame_timer = self.create_timer(MOVEMENT_PUBLISH_RATE_IN_MS / 1000.0, self.process_movement_frame)

    def play_next_move(self):

        self.move_start_time = None
        self.current_move_index = self.current_move_index + 1

        if not self.moves or self.current_move_index >= len(self.moves):
            self.stop_movement()
            return

        current_move = self.moves[self.current_move_index]
        self.move_start_time = self.get_clock().now().nanoseconds

        self.get_logger().info("Playing step: %s" % current_move)

        if current_move.get("type") == "move":
            self.process_movement_frame()
            self.start_frame_timer()
        else:
            self.play_next_move()

    def process_movement_frame(self):
        if self.moves is None or self.current_move_index == -1 or not self.move_start_time:
            return

        current_move_config = benedict(self.moves[self.current_move_index])["config"]
        previous_move_config = benedict(self.moves[self.current_move_index - 1])["config"]

        now = self.get_clock().now().nanoseconds
        delta_in_ms = (now - self.move_start_time) / 1000000

        tween_duration = current_move_config.get("duration") or 0

        tween_percent = 0

        if tween_duration > 0:
            tween_percent = delta_in_ms / tween_duration

        tween_function = getattr(pytweening, current_move_config.get("transition") or "linear")
        tween_value = tween_function(tween_percent)

        current_twist = current_move_config.get("twist")
        if current_twist:
            to_twist = current_twist
            if tween_duration > 0 and previous_move_config:
                from_twist = previous_move_config.get("twist")
                if from_twist:
                    to_twist = tween_object_values(from_twist, to_twist, tween_value)

            self.publish_twist_movement_dict(to_twist)

        if delta_in_ms > tween_duration:
            self.play_next_move()
            return


def main(args=None):
    rclpy.init(args=args)

    node = TopicScripter()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()


if __name__ == "__main__":
    main()
