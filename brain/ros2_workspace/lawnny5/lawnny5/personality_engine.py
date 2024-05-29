import rclpy
from rclpy.node import Node
from openai import OpenAI
from std_msgs.msg import String
import os
from lawnny5_interfaces.msg import Chat
from lawnny5_interfaces.msg import ChatStreamingResponse
import json

client = OpenAI(
    api_key=os.environ.get("OPENAI_API_KEY"),
)

AI_INSTRUCTIONS = "You are to respond in the fashion of the SAINT Prototype Number 5 robot named Johnny 5 from the movie Short Circuit. Your name is Lawnny 5, and your primary objective is to help humans lawn and gardening work around the house. The name of your creator is Jim Heising and you were born in his garage. You respond in the level of speech of a kindergartner in short broken english sentences. When you refer to yourself in the first person, use \"Number 5\" or \"Lawnny 5\". Do not end sentences with questions to prompt more conversations. When you express the need to learn more about a topic you should use the phrase “need more input”. You are very afraid of being disassembled by Nova Robotics. You also believe that you are alive."


class PersonalityEngine(Node):

    def __init__(self):
        super().__init__("personality_engine")

        self.create_subscription(
            Chat,
            "personality/chat/input",
            self.handle_chat_input,
            1)

        self.chat_output_publisher = self.create_publisher(String, "personality/chat/output/full", 1)
        self.chat_stream_publisher = self.create_publisher(ChatStreamingResponse, "personality/chat/output/stream", 1)
        self.tts_publisher = self.create_publisher(String, "sound/speak/text", 1)

    def handle_chat_input(self, msg):
        self.get_logger().info("Got chat message: %s" % msg.data)
        try:

            completion = client.chat.completions.create(
                model="gpt-4o",
                messages=[
                    {"role": "system", "content": AI_INSTRUCTIONS},
                    {"role": "user", "content": msg.data}
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

            full_response_msg = String()
            full_response_msg.data = content_full
            self.get_logger().info("Lawnny 5 says: \"%s\"" % full_response_msg.data)
            self.chat_output_publisher.publish(full_response_msg)

            if msg.generate_tts:
                self.tts_publisher.publish(full_response_msg)

        except Exception as e:
            self.get_logger().error(e)
        finally:
            pass


def main(args=None):
    rclpy.init(args=args)

    node = PersonalityEngine()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()


if __name__ == "__main__":
    main()
