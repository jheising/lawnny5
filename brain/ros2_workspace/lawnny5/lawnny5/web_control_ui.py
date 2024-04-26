import rclpy
from rclpy.node import Node
import http.server
import socketserver
import os

DEFAULT_SERVER_PORT = 8000

class Handler(http.server.SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=(os.getenv('LAWNNY5_ROOT') + "/www"), **kwargs)

class WebControlUI(Node):

    def __init__(self):
        super().__init__('web_control_ui')

        self.get_logger().info('Starting web controller UI on port %d' % DEFAULT_SERVER_PORT)
        httpd = socketserver.TCPServer(("", DEFAULT_SERVER_PORT), Handler)
        try:
            httpd.serve_forever()
        except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
            httpd.shutdown()


def main(args=None):
    rclpy.init(args=args)

    ros_node = WebControlUI()
    rclpy.spin(ros_node)
    ros_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()