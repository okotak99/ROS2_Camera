import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ros_camera_msgs.srv import SaveImage
from ros_camera_msgs.srv import SaveVideo
from ros_camera_msgs.srv import ChangeTopic
import cv2
import time
from datetime import datetime


class SaveImageServer(Node):
    def __init__(self):
        super().__init__('camera_server')

        # service
        self.srv_img = self.create_service(SaveImage, 'save_image', self.handle_save_image)
        self.srv_vid = self.create_service(SaveVideo, 'save_video', self.handle_save_video)
        self.srv_change_topic = self.create_service(ChangeTopic, 'change_topic', self.handle_change_topic)

        # subscriber
        self.subscription = self.create_subscription(Image, '/camera', self.image_callback, 10)

        # ro2 <-> opencv datatype converter
        self.cv_bridge = CvBridge()

        self.is_image_received = False  
        self.recording = False
        self.video_writer = None

        # file path setting
        self.declare_parameter('image_path', "/home/addinedu/dev_ws/ROSCamera/src/ros_camera/resource/image/")
        self.image_path = self.get_parameter('image_path').value
        self.declare_parameter('video_path', "/home/addinedu/dev_ws/ROSCamera/src/ros_camera/resource/video/")
        self.video_path = self.get_parameter('video_path').value

    def image_callback(self, msg):
        self.process_image(msg)

    # image callback methods
    def process_image(self, msg):
        if self.subscription is None:
            self.is_image_received = False
            return

        self.cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self.is_image_received = True 
        if self.recording:  
            self.video_writer.write(self.cv_image)  

            if time.time() > self.end_time:
                self.recording = False
                self.video_writer.release()
                self.get_logger().info("Video recording completed.")


    # save_image callback method
    def handle_save_image(self, request, response):
        if self.is_image_received: 
            now = datetime.now()
            timestamp = now.strftime("%Y%m%d_%H%M%S")
            file_path = self.image_path + f'{timestamp}.jpg'
            cv2.imwrite(file_path, self.cv_image)
            
            response = SaveImage.Response()
            response.success = True
            response.message = "Image saved successfully"
            self.get_logger().info("Image saved to: %s" % file_path)

        else:
            response = SaveImage.Response()
            response.success = False
            response.message = "No image received to save"
            self.get_logger().warn("No image received to save")

        return response


    # save_video callback method
    def handle_save_video(self, request, response):
        self.duration_seconds = request.duration - 2
        self.start_time = time.time()  
        self.end_time = self.start_time + self.duration_seconds  
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        file_path = self.video_path + f'{timestamp}.avi'
        self.video_writer = cv2.VideoWriter(file_path, cv2.VideoWriter_fourcc(*"XVID"), 20, (self.cv_image.shape[1], self.cv_image.shape[0]))
        self.recording = True
        response.success = True
        response.message = "Video saved successfully"

        return response
    
    # change topic
    def change_subscription(self, topic = '/camera'):
        new_subscription = self.create_subscription(Image, topic, self.image_callback, 10)
        return new_subscription

    
    # change_topic callback method 
    def handle_change_topic(self, request, response):
        # get the new topic from the request
        new_topic = '/' + request.topic  
        if self.subscription is not None:
            self.destroy_subscription(self.subscription)
        self.subscription = self.change_subscription(new_topic)

        response.success = True
        response.message = f"Changed topic to {new_topic}"

        return response


def main(args=None):
    rclpy.init(args=args)
    server = SaveImageServer()
    rclpy.spin(server)
    server.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
