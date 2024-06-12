import rclpy
from rclpy.node import Node
import json
from .lib.keyframe_interpreter.KeyframeInterpreter import KeyframeInterpreter
from std_msgs.msg import String
from rclpy_message_converter import message_converter
import os

DEFAULT_FRAME_RATE_IN_MS = 100

class TopicScripter(Node):

    def __init__(self):
        super().__init__("topic_scripter")

        self.current_script = None
        self.current_script_interpreter = None
        self.current_topic_publishers = {}
        self.frame_timer = None
        self.script_start_time = None

        self.scripts_dir = os.getenv("LAWNNY5_ASSETS") + "/moves"

        self.create_subscription(
            String,
            "topic_script/play/by_name",
            self.handle_play_script_by_name,
            1)

    def __del__(self):
        self.stop_script()

    def handle_play_script_by_name(self, msg):
        self.play_script_by_name(msg.data)

    def play_script_by_name(self, move_name):
        self.play_script_file(self.scripts_dir + "/" + move_name + ".json")

    def play_script_file(self, filename):
        with open(filename) as file_data:
            move_data = json.load(file_data)
            self.play_script(move_data)

    def stop_script(self):

        if self.frame_timer:
            self.destroy_timer(self.frame_timer)
            self.frame_timer = None

        self.current_script = None
        self.current_script_interpreter = None
        self.script_start_time = None

        for publisher in self.current_topic_publishers.values():
            self.destroy_publisher(publisher)

        self.current_topic_publishers = {}

    def play_script(self, script):
        self.stop_script()
        self.current_script = script

        # Create publishers for all of our topics
        topics = script["topics"]
        for topic_key in topics.keys():
            topic = topics[topic_key]
            msg = message_converter.convert_dictionary_to_ros_message(topic["type"], {})
            self.current_topic_publishers[topic_key] = self.create_publisher(type(msg), topic["topic"], 1)

        script_keyframes = script["keyframes"]
        script_duration = self.current_script["duration"]

        self.current_script_interpreter = KeyframeInterpreter(script_keyframes, script_duration)

        self.script_start_time = self.get_clock().now().nanoseconds
        framerate = script.get("framerate") or DEFAULT_FRAME_RATE_IN_MS
        self.frame_timer = self.create_timer(framerate / 1000.0, self._process_frame)

    def _process_frame(self):
        now = self.get_clock().now().nanoseconds
        elapsed_ms = (now - self.script_start_time) / 1000000
        script_duration = self.current_script["duration"]
        elapsed_percentage = elapsed_ms / script_duration

        new_messages = self.current_script_interpreter.play(elapsed_percentage)

        for message in new_messages:
            publisher = self.current_topic_publishers.get(message["type"])
            if publisher:
                message_type = self.current_script["topics"][message["type"]]["type"]
                msg = message_converter.convert_dictionary_to_ros_message(message_type, message["msg"])
                publisher.publish(msg)

        if elapsed_percentage >= 1.0:
            self.stop_script()


def main(args=None):
    rclpy.init(args=args)

    node = TopicScripter()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()


if __name__ == "__main__":
    main()
