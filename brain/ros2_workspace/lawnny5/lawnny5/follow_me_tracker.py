import rclpy
from rclpy.node import Node
import sys
sys.path.append("../../lib/depthai_hand_tracker/examples/remote_control")
from HandController import HandController

class FollowMeTracker(Node):

    def __init__(self):
        super().__init__('follow_me_tracker')

    def start(self):
        pass

def main(args=None):
    rclpy.init(args=args)

    node = FollowMeTracker()
    node.start()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()