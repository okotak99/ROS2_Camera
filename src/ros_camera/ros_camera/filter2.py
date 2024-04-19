import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np


class Filter2(Node):
    def __init__(self):
        super().__init__('filter2')
        self.subscriber = self.create_subscription(Image, '/camera', self.image_callback, 10)
        self.pub_gray = self.create_publisher(Image, '/gray', 10)
        self.pub_canny = self.create_publisher(Image, 'canny', 10)
        self.pub_cartoon = self.create_publisher(Image, '/cartoon', 10)

        self.cv_bridge = CvBridge()

    def image_callback(self, msg):
        img = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # publish gray image
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        gray_img = self.cv_bridge.cv2_to_imgmsg(gray, "mono8")
        self.pub_gray.publish(gray_img)

        # publish canny image
        canny = cv2.Canny(gray, 1000, 2000, apertureSize=5)
        canny_img = self.cv_bridge.cv2_to_imgmsg(canny, "mono8")
        self.pub_canny.publish(canny_img)

        # publish cartoon image
        h, w = img.shape[:2]
        resized_img = cv2.resize(img, (w//2, h//2))
        blr = cv2.bilateralFilter(resized_img, -1, 20, 7)
        edge = 255 - cv2.Canny(resized_img, 80, 120)
        edge = cv2.cvtColor(edge, cv2.COLOR_GRAY2BGR)
        merged_img = cv2.bitwise_and(blr, edge)
        resized_img = cv2.resize(merged_img, (w, h), interpolation=cv2.INTER_NEAREST)
        cartoon_img = self.cv_bridge.cv2_to_imgmsg(resized_img, "bgr8")
        self.pub_cartoon.publish(cartoon_img)


def main():
    rclpy.init()
    node = Filter2()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__=='__main__':
    main()
