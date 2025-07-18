import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class WebcamPublisher(Node):
    def __init__(self):
        super().__init__('webcam_publisher')
        self.publisher_ = self.create_publisher(Image, 'camera/image_raw', 10)
        self.timer_period = 0.033  # seconds (approx 30 frames per second)
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.cap = cv2.VideoCapture(0)  
        self.bridge = CvBridge()
        self.get_logger().info('Webcam Publisher Node Started')

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            flipped_frame = cv2.flip(frame, 1) 
            ros_image_message = self.bridge.cv2_to_imgmsg(flipped_frame, "bgr8")
            self.publisher_.publish(ros_image_message)
            self.get_logger().info('Publishing video frame')
        else:
            self.get_logger().error('Failed to capture image from webcam')

def main(args=None):
    rclpy.init(args=args)
    webcam_publisher = WebcamPublisher()
    rclpy.spin(webcam_publisher)
    webcam_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()