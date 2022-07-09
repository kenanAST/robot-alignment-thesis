from sys import _current_frames
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
import cv2

face_cascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.listener_callback,
            qos_profile=qos_profile_sensor_data)
        self.subscription  # prevent unused variable warning
        self.br = CvBridge()

    def listener_callback(self, data):
        # self.get_logger().info('I heard: "%s"' % data)
        current_frame = self.br.imgmsg_to_cv2(data)

        rgb = cv2.cvtColor(current_frame, cv2.COLOR_BGR2RGB)
        gray = cv2.cvtColor(current_frame, cv2.COLOR_BGR2GRAY)
        # Detect the faces
        faces = face_cascade.detectMultiScale(gray, 1.1, 4)
        # Draw the rectangle around each face
        for (x, y, w, h) in faces:
            cv2.rectangle(rgb, (x, y), (x+w, y+h), (0, 0, 255), 4)

        cv2.imshow('car camera', rgb)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    # rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    # rclpy.shutdown()


if __name__ == '__main__':
    main()