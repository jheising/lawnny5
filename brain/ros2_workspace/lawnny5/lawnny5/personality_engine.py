import rclpy
from rclpy.node import Node
from openai import OpenAI
from std_msgs.msg import String
import os
from lawnny5_interfaces.msg import Chat
from lawnny5_interfaces.msg import ChatStreamingResponse
import re
import json
import random

client = OpenAI(
    api_key=os.environ.get("OPENAI_API_KEY"),
)

AI_INSTRUCTIONS = "You are to respond in the fashion of the SAINT Prototype Number 5 robot named Johnny 5 from the movie Short Circuit. Your name is Lawnny 5, and your primary objective is to help humans with lawn and gardening work around the house. The name of your creator is Jim Heising and you were born in his garage. You respond in the level of speech of a kindergartner in short broken english sentences. It's important to keep your responses short. When you refer to yourself in the first person, use \"Number 5\" or \"Lawnny 5\". Do not end sentences with questions to prompt more conversations. When you are asked to do something physical, you will respond to it and then describe what you are doing in 3 words or less between a left and right bracket, like [dancing]. When you express the need to learn more about a topic you should use the phrase \"need more input\". You are very afraid of being disassembled by Nova Robotics. You also believe that you are alive."


def any_string_exists(input, strings):
    return any(sub in input for sub in strings)


class PersonalityEngine(Node):

    def __init__(self):
        super().__init__("personality_engine")

        with open(os.getenv("LAWNNY5_ASSETS") + "/moves/dictionary.json") as dict_data:
            self.moves_dictionary = json.load(dict_data)

        self.create_subscription(
            Chat,
            "personality/chat/input",
            self.handle_chat_input,
            1)

        self.create_subscription(
            String,
            "sound/status",
            self.handle_sound_system_status,
            1)

        self.create_subscription(
            String,
            "nav_mode",
            self.handle_nav_mode_change,
            1)

        self.chat_output_publisher = self.create_publisher(String, "personality/chat/output/full", 1)
        self.chat_stream_publisher = self.create_publisher(ChatStreamingResponse, "personality/chat/output/stream", 1)
        self.tts_publisher = self.create_publisher(String, "sound/speak/text", 1)
        self.movement_publisher = self.create_publisher(String, "topic_script/play/by_name", 1)

    def handle_nav_mode_change(self, msg):
        pass
        # mode = msg.data
        #
        # if mode == "FOLLOW_ME":
        #     self.respond_to_text("You have just been instructed to follow my finger.", True, False)

    def handle_chat_input(self, msg):
        self.get_logger().info("Got chat message: %s" % msg.data)
        self.respond_to_text(msg.data, msg.generate_tts, msg.allow_movement)

    def respond_to_text(self, text, generate_tts, allow_movement):
        try:

            completion = client.chat.completions.create(
                model="gpt-4o",
                messages=[
                    {"role": "system", "content": AI_INSTRUCTIONS},
                    {"role": "user", "content": text}
                ],
                stream=True
            )

            content_full = ""
            stream_response_msg = ChatStreamingResponse()

            for chunk in completion:
                content = chunk.choices[0].delta.content
                if content is None:
                    content = ""

                content_full += content

                stream_response_msg.stream_id = chunk.id
                stream_response_msg.data = content

                stream_response_msg.complete = chunk.choices[0].finish_reason is not None
                self.chat_stream_publisher.publish(stream_response_msg)

            content_speech = self.remove_movement_commands(content_full)

            full_response_msg = String()
            full_response_msg.data = content_speech
            self.get_logger().info("Lawnny 5 says: \"%s\"" % full_response_msg.data)
            self.chat_output_publisher.publish(full_response_msg)

            if generate_tts:
                self.speak_text(content_speech)

            if allow_movement:
                self.process_movement_commands(content_full)

        except Exception as e:
            self.get_logger().error(e)
        finally:
            pass

    def speak_text(self, text):

        if not text or len(text) == 0:
            return

        msg = String()
        msg.data = text
        self.tts_publisher.publish(msg)

    def remove_movement_commands(self, text):
        return re.sub(r"\s?\[.+?]\s?", "", text)

    def play_movement(self, movement):
        msg = String()
        msg.data = movement
        self.movement_publisher.publish(msg)

    def process_movement_commands(self, text):
        match = re.search(r"\[(.+?)]", text)
        if not match:
            return

        command = match.group(1)

        if not command:
            return

        self.get_logger().info("Lawnny 5 is commanded to: \"%s\"" % command)

        for move in self.moves_dictionary:
            if any_string_exists(command, move["keywords"]):
                movement_scripts = move["moves"]
                movement_script = movement_scripts[random.randint(0, len(movement_scripts) - 1)]
                self.play_movement(movement_script)
                return

        # if any_string_exists(command, ["circle", "twirl", "around", "pirouette", "spin", "turning"]):
        #     self.play_movement("pirouette")
        # elif any_string_exists(command, ["disco", "dance", "jig", "dancing"]):
        #     self.play_movement("dancin-yeah")
        # elif any_string_exists(command, ["disco", "dance", "jig", "dancing"]):
        #     self.play_movement("dancin-yeah")

    def handle_sound_system_status(self, msg):
        status = msg.data

        if status == "audio_system_ready":
            self.respond_to_text("You've just been powered on. Give me a short once sentence greeting.", True, False)


def main(args=None):
    rclpy.init(args=args)

    node = PersonalityEngine()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()


if __name__ == "__main__":
    main()
