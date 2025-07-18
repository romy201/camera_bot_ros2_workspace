import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import math 
from my_robot_interfaces.msg import DetectionInfo 

# Import Twist message for robot commands
from geometry_msgs.msg import Twist

HAARCASCADE_PATH = '/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml' # CHECK THIS PATH/FILE NAME!



# Control gains - tune these for your robot's speed and responsiveness
# K_ANGULAR: How fast the robot turns based on horizontal deviation
K_ANGULAR = 0.00005 # Increase for faster turns, decrease for slower


# Thresholds for stopping angular movement
ANGULAR_STOP_THRESHOLD_PX = 10 # If object center is within +/- this many pixels of screen center, stop turning

# Max angular speed (to prevent aggressive turning)
MAX_ANGULAR_SPEED = 0.25 # radians/second (adjust for your robot)
# --- END ROBOT CONTROL PARAMETERS ---


class HumanDetector(Node):
    def __init__(self):
        super().__init__('human_detector')
        self.subscription = self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_callback,
            10)
        self.subscription  

        self.robot_action_publisher_ = self.create_publisher(Twist, '/robotAction', 10)
        self.detection_info_publisher_ = self.create_publisher(DetectionInfo, '/detection_info', 10)


        if not os.path.exists(HAARCASCADE_PATH):
            self.get_logger().error(f"Haar cascade file NOT found at: {HAARCASCADE_PATH}")
            rclpy.shutdown() # Exit the node if the cascade file isn't found
            return

       
        self.face_cascade = cv2.CascadeClassifier(HAARCASCADE_PATH)

        if self.face_cascade.empty():
            self.get_logger().error("Failed to load Haar cascade classifier. Is the XML file corrupt?")
            rclpy.shutdown()
            return
        self.bridge = CvBridge()
        self.get_logger().info('Human Detector Node Started, waiting for images...')



    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            (img_h, img_w) = cv_image.shape[:2]
            screen_center_x = img_w / 2.0 
            
            gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            
           # sclae is like how hard to look for a face
           # min is how many cond to consider smth as face (roughly)

            objects = self.face_cascade.detectMultiScale(
                gray_image,
                scaleFactor=1.2,    
                minNeighbors=3,       
                minSize=(30 ,30),   
                maxSize=(350, 350)    
            )

            cmd_vel_msg = Twist()
            cmd_vel_msg.linear.x = 0.0
            cmd_vel_msg.angular.z = 0.0 
            detection_info_msg = DetectionInfo()
            detection_info_msg.detected = False 


            largest_object = None
            max_object_area = 0

            for (x, y, w, h) in objects:
                area = w * h
                if area > max_object_area:
                    max_object_area = area
                    largest_object = (x, y, w, h)

            if largest_object is not None:
                x, y, w, h = largest_object 
                cv2.rectangle(cv_image, (x, y), (x+w, y+h), (0, 255, 0), 2) 

                center_x = x + w // 2
                center_y = y + h // 2

                cv2.circle(cv_image, (center_x, center_y), 5, (0, 0, 255), -1) 

                self.get_logger().info(f'Object detected: BBox=({x}, {y}, {w}, {h}), Center=({center_x}, {center_y})')

                detection_info_msg.detected = True
                detection_info_msg.x = float(x)
                detection_info_msg.y = float(y)
                detection_info_msg.width = float(w)
                detection_info_msg.height = float(h)
                detection_info_msg.center_x = float(center_x)
                detection_info_msg.center_y = float(center_y)

                deviation_x = center_x - screen_center_x 

                if math.fabs(deviation_x) > ANGULAR_STOP_THRESHOLD_PX:

                    cmd_vel_msg.angular.z = K_ANGULAR * deviation_x 

                    cmd_vel_msg.angular.z = max(min(cmd_vel_msg.angular.z, MAX_ANGULAR_SPEED), -MAX_ANGULAR_SPEED)
                else:
                    cmd_vel_msg.angular.z = 0.0 

                self.get_logger().info(f"Angular Action (Twist): angular.z={cmd_vel_msg.angular.z:.2f}")

            else: 
                self.get_logger().info("No object detected. Robot stopping angular movement.")
                cmd_vel_msg.angular.z = 0.0 


            self.robot_action_publisher_.publish(cmd_vel_msg)

            self.detection_info_publisher_.publish(detection_info_msg)

            cv2.imshow("Human Detection Output", cv_image)
            cv2.waitKey(10) 

        except Exception as e:
            self.get_logger().error(f'Error processing image or detecting: {e}')


def main(args=None):
    rclpy.init(args=args)
    human_detector = HumanDetector()
    rclpy.spin(human_detector)
    cv2.destroyAllWindows() 
    human_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()