import rclpy
from rclpy.node import Node
from .lib.depthai_hand_tracker.HandController import HandController
from lawnny5_interfaces.msg import ObjectTrack
from lawnny5_interfaces.srv import SetString
from std_msgs.msg import String
import depthai as dai
from sensor_msgs.msg import CompressedImage
import numpy as np


CAMERA_FPS = 24

class DepthAICamera(Node):

    def __init__(self):
        super().__init__('depth_ai_camera')

        self.mode = None
        self.hand_controller = None
        self.hand_tracking_publisher = None
        self.timer = None
        self.image_subscriber_count = 0
        self.camera_started = False
        self.frame_timer = None
        self.camera_device = None
        self.camera_output_queue = None
        self.has_camera_subscriber = False

        self.mode_service = self.create_service(SetString, 'camera_mode', self.camera_mode_callback)
        self.compressed_image_publisher = self.create_publisher(CompressedImage, 'camera/color/bgr', 1)
        self.subscriber_timer = self.create_timer(1.0, self.check_image_subscriber_count)

    def __del__(self):
        self.cleanup()

    def check_image_subscriber_count(self):
        self.image_subscriber_count = self.compressed_image_publisher.get_subscription_count()

        if self.image_subscriber_count > 0:
            self.has_camera_subscriber = True
        else:
            self.has_camera_subscriber = False

    def camera_mode_callback(self, request, response):
        mode = request.data
        self.set_mode(mode)
        response.success = True
        return response

    def cleanup(self):
        self.stop_camera()

        if self.camera_output_queue:
            self.camera_output_queue = None

        if self.hand_controller:
            del self.hand_controller
            self.hand_controller = None

        if self.hand_tracking_publisher:
            self.destroy_publisher(self.hand_tracking_publisher)
            self.hand_tracking_publisher = None

        if self.timer:
            self.timer.cancel()
            self.destroy_timer(self.timer)
            self.timer = None

    def hand_recognized(self, event):
        if event.hand.gesture == "ONE":
            ot = ObjectTrack()
            ot.name = event.hand.gesture
            ot.x = event.hand.rect_x_center_a
            ot.y = event.hand.rect_y_center_a
            ot.width = event.hand.rect_w_a
            ot.height = event.hand.rect_h_a
            ot.bound_width = float(self.hand_controller.tracker.frame_size)
            ot.bound_height = ot.bound_width

            if self.hand_tracking_publisher:
                self.hand_tracking_publisher.publish(ot)

    def hand_recognizer_tick(self):
        if self.hand_controller:
            self.hand_controller.tick()

    def set_mode(self, mode):

        if mode == self.mode:
            return

        self.mode = mode
        self.cleanup()

        self.get_logger().info("Transitioning to mode %s..." % mode)

        if mode == "hand_tracker":

            config = {
                'renderer': {'enable': False},
                'pose_actions': [
                    {'name': 'ONE', 'pose': 'ONE', 'trigger': 'periodic', 'callback': self.hand_recognized},
                ]
            }

            self.hand_tracking_publisher = self.create_publisher(ObjectTrack, 'hand_tracking', 1)
            self.hand_controller = HandController(config)
            self.camera_output_queue = self.hand_controller.tracker.q_video
            self.timer = self.create_timer(0.1, self.hand_recognizer_tick)
            self.start_camera()

        self.get_logger().info("Successfully transitioned to mode %s" % mode)

    def stop_camera(self):
        if not self.camera_started:
            return

        self.camera_started = False

        if self.frame_timer:
            self.frame_timer.cancel()
            self.destroy_timer(self.frame_timer)
            self.frame_timer = None

        self.get_logger().info("Stopped DepthAI Camera Feed...")

    def start_camera(self):
        if self.camera_started:
            return

        if self.camera_output_queue:
            self.camera_started = True
            self.get_logger().info("Starting DepthAI Camera Feed...")
            self.frame_timer = self.create_timer(1.0 / CAMERA_FPS, self.process_camera_frame)

    def process_camera_frame(self):

        if not self.camera_output_queue:
            return

        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.format = "jpeg"
        msg.data = np.array(self.camera_output_queue.get().getData()).tobytes()
        self.compressed_image_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    node = DepthAICamera()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
        #rclpy.spin(node)
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()