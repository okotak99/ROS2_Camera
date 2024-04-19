import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np

class Filter1(Node):
    def __init__(self):
        super().__init__('filter1')
        self.subscriber = self.create_subscription(Image, '/camera', self.image_callback, 10)
        self.pub_concave = self.create_publisher(Image, '/concave', 10)
        self.cv_bridge = CvBridge()


    def image_callback(self, msg):
        # ros2 datatype to cv2 datatype
        img = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # publish concave image
        rows, cols = img.shape[:2]
        exp = 0.5
        scale = 1
        mapy, mapx = np.indices((rows, cols),dtype=np.float32)
        mapx = 2*mapx/(cols-1)-1
        mapy = 2*mapy/(rows-1)-1
        r, theta = cv2.cartToPolar(mapx, mapy)
        r[r< scale] = r[r<scale] **exp
        mapx, mapy = cv2.polarToCart(r, theta)
        mapx = ((mapx + 1)*cols-1)/2
        mapy = ((mapy + 1)*rows-1)/2
        frame = cv2.remap(img,mapx,mapy,cv2.INTER_LINEAR)
        concave_img = self.cv_bridge.cv2_to_imgmsg(frame, "bgr8")
        self.pub_concave.publish(concave_img)


def main():
    rclpy.init()
    node = Filter1()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__=='__main__':
    main()
