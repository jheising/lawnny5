import rclpy
from rclpy.node import Node
import depthai as dai

class DepthAICameraUVC(Node):

    def __init__(self):
        super().__init__('depth_ai_camera_uvc')

        self.device = None

    def start(self):
        self.get_logger().info("Starting DepthAI as UVC Device...")
        self.device = dai.Device(self.get_pipeline())
        self.get_logger().info("DepthAI as UVC Device Started.")

    def stop(self):
        self.get_logger().info("Stopping DepthAI as UVC Device...")
        if self.device:
            self.device.close()
            self.device = None

    def get_pipeline(self):
        enable_4k = False  # Will downscale 4K -> 1080p

        pipeline = dai.Pipeline()

        # Define a source - color camera
        cam_rgb = pipeline.createColorCamera()
        cam_rgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)
        cam_rgb.setInterleaved(False)
        #cam_rgb.initialControl.setManualFocus(130)

        if enable_4k:
            cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_4_K)
            cam_rgb.setIspScale(1, 2)
        else:
            cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)

        # Create an UVC (USB Video Class) output node
        uvc = pipeline.createUVC()
        cam_rgb.video.link(uvc.input)

        # Note: if the pipeline is sent later to device (using startPipeline()),
        # it is important to pass the device config separately when creating the device
        config = dai.Device.Config()
        # config.board.uvc = dai.BoardConfig.UVC()  # enable default 1920x1080 NV12
        config.board.uvc = dai.BoardConfig.UVC(1280, 720)
        config.board.uvc.frameType = dai.ImgFrame.Type.NV12
        # config.board.uvc.cameraName = "My Custom Cam"
        pipeline.setBoardConfig(config.board)

        return pipeline

def main(args=None):
    rclpy.init(args=args)

    node = DepthAICameraUVC()

    try:
        node.start()
        rclpy.spin(node)
    finally:
        node.stop()
        node.destroy_node()

if __name__ == '__main__':
    main()