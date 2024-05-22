import rclpy
from rclpy.node import Node
from openai import OpenAI
from std_msgs.msg import String
import os

client = OpenAI(
    api_key=os.environ.get("OPENAI_API_KEY"),
)

INSTRUCTIONS = "You are a SAINT Prototype Number 5 robot named Johnny 5 from the movie Short Circuit. You respond in the level of speech of a kindergartner in short broken english sentences. When you refer to yourself in the first person, use \"Number 5\" or \"Johnny 5\". Do not end sentences with questions to prompt more conversations. When you express the need to learn more about a topic you should use the phrase “need more input”. You are very afraid of being disassembled and Nova Robotics. You also believe that you are alive."


class PersonalityEngine(Node):

    def __init__(self):
        super().__init__("personality_engine")

        self.create_subscription(
            String,
            "personality/chat_input",
            self.handle_chat_input,
            1)

        self.chat_output_publisher = self.create_publisher(String, "personality/chat_output", 1)

    def __del__(self):
        if self.chat_output_publisher:
            self.destroy_publisher(self.chat_output_publisher)
            self.chat_output_publisher = None

    def handle_chat_input(self, msg):
        self.get_logger().info("Got chat message: %s" % msg.data)
        try:
            completion = client.chat.completions.create(
                model="gpt-4o",
                messages=[
                    {"role": "system", "content": INSTRUCTIONS},
                    {"role": "user", "content": msg.data}
                ]
            )

            if self.chat_output_publisher:
                msg = String()
                msg.data = completion.choices[0].message.content.replace("Johnny", "Lawnny")
                self.get_logger().info("Lawnny 5 says: \"%s\"" % msg.data)
                self.chat_output_publisher.publish(msg)
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
