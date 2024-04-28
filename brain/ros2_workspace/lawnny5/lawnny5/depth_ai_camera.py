import rclpy
from rclpy.node import Node
import depthai as dai
from sensor_msgs.msg import CompressedImage
import numpy as np

CAMERA_FPS = 24

class DepthAICamera(Node):

    def __init__(self):
        super().__init__('depth_ai_camera')
        self.compressed_image_publisher = self.create_publisher(CompressedImage, 'camera/color/bgr', 10)

        self.create_timer(1, self.check_subscribers)

        self.subscriber_count = 0
        self.frame_timer = None
        self.camera_device = None
        self.camera_output_queue = None
        self.camera_started = False

    def check_subscribers(self):
        self.subscriber_count = self.compressed_image_publisher.get_subscription_count()

        # self.get_logger().info("Subscriber count %d" % self.subscriber_count)

        if not self.camera_started and self.subscriber_count > 0:
            self.start_camera()
        elif self.camera_started and self.subscriber_count == 0:
            self.stop_camera()

    def stop_camera(self):
        self.get_logger().info("Stopping DepthAI Camera Feed...")

        if self.frame_timer:
            self.frame_timer.destroy()
            self.frame_timer = None

        if self.camera_device:
            self.camera_device.close()
            self.camera_device = None

        if self.camera_output_queue:
            self.camera_output_queue.close()
            self.camera_output_queue = None

        self.camera_started = False

    def start_camera(self):
        if self.camera_started:
            return

        self.camera_started = True
        self.get_logger().info("Starting DepthAI Camera Feed...")

        # Create pipeline
        pipeline = dai.Pipeline()

        camRgb = pipeline.create(dai.node.ColorCamera)

        videoEnc = pipeline.create(dai.node.VideoEncoder)
        videoEnc.setDefaultProfilePreset(CAMERA_FPS, dai.VideoEncoderProperties.Profile.MJPEG)
        camRgb.video.link(videoEnc.input)

        xoutVideo = pipeline.create(dai.node.XLinkOut)
        xoutVideo.setStreamName("mjpeg")
        videoEnc.bitstream.link(xoutVideo.input)

        self.camera_device = dai.Device(pipeline)
        self.camera_output_queue = self.camera_device.getOutputQueue(name="mjpeg", maxSize=1, blocking=False)
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

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()