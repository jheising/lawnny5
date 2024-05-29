import rclpy
from rclpy.node import Node
import http.server
import socketserver
import os

DEFAULT_SERVER_PORT = 8000

class Handler(http.server.SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=(os.getenv('LAWNNY5_ASSETS') + "/www"), **kwargs)

class WebControlUI(Node):

    def __init__(self):
        super().__init__('web_control_ui')

        self.get_logger().info('Starting web controller UI on port %d' % DEFAULT_SERVER_PORT)
        self.httpd = socketserver.TCPServer(("", DEFAULT_SERVER_PORT), Handler)
        self.httpd.serve_forever()

    def __del__(self):
        self.httpd.shutdown()

def main(args=None):
    rclpy.init(args=args)

    ros_node = WebControlUI()

    try:
        rclpy.spin(ros_node)
    finally:
        ros_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()