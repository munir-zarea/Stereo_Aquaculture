import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class SBSStream(Node):
    def __init__(self):
        super().__init__('sbs_cam')
        self.publisher_ = self.create_publisher(Image, '/stereo/image_raw', 10)
        self.bridge = CvBridge()
        self.timer = self.create_timer(0.03, self.timer_callback)  # ~30 Hz
        self.cap = cv2.VideoCapture('/dev/video4')  # Change as needed

        if not self.cap.isOpened():
            self.get_logger().error("Cannot open camera device")

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("Failed to grab frame")
            return

        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SBSStream()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
