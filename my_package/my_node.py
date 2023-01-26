import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import cv2
import cv_bridge

class ArucoMarkerDetector(Node):

    def __init__(self):
        super().__init__('aruco_marker_detector')
        self.sub = self.create_subscription(Image, '/image_raw', self.image_callback, 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.bridge = cv_bridge.CvBridge()

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        marker_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        corners, ids, _ = cv2.aruco.detectMarkers(cv_image, marker_dict)

        if ids is not None:
            for i in range(len(ids)):
                if corners[i][0][0][1] < cv_image.shape[0]/2:
                    twist = Twist()
                    twist.linear.x = 0.5
                    self.pub.publish(twist)
                else:
                    twist = Twist()
                    twist.linear.x = -0.5
                    self.pub.publish(twist)

        cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)

        cv2.imshow("Aruco Markers", cv_image)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    aruco_detector = ArucoMarkerDetector()
    rclpy.spin(aruco_detector)
    aruco_detector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


