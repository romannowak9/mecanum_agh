import rclpy
from rclpy.node import Node
import numpy as np
import cv2

from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy


class CameraSubscriber(Node):

    def __init__(self):
        super().__init__('camera_node')
        
        print("Strting camera node...")

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.img_sub_ = self.create_subscription(Image, "/camera", self.camera_cb, qos_profile=qos_profile)

        self.set_point_pub_ = self.create_publisher(Point, "/img_set_point", qos_profile=qos_profile)

        self.__curr_frame = None
    
    def camera_cb(self, msg: Image):
        input_data = np.frombuffer(msg.data, dtype=np.uint8)
        current_frame_rgb = input_data.reshape([msg.height, msg.width, 3])
        self.__curr_frame = cv2.cvtColor(current_frame_rgb, cv2.COLOR_RGB2BGR)
        # self.__curr_frame = cv2.cvtColor(current_frame_rgb, cv2.COLOR_RGB2GRAY)
 
        # w HSV [10, 150, 100] i [30, 255, 255]
        lower = [0, 110,  180]
        upper = [5, 128, 210]
        mask = cv2.inRange(self.__curr_frame, np.array(lower, dtype="uint8"), np.array(upper, dtype="uint8"))
        output = cv2.bitwise_and(self.__curr_frame, self.__curr_frame, mask=mask)
        cv2.imshow('mask', output)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        largest_contour = None
        max_area = 0

        for contour in contours:
            area = cv2.contourArea(contour)
            if area > max_area:
                max_area = area
                largest_contour = contour

        if largest_contour is not None:
            M = cv2.moments(largest_contour)
            if M['m00'] != 0:
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])

                cv2.circle(self.__curr_frame, (cx, cy), 6, (0, 255, 0), -1)  # Okrąg w kolorze czerwonym
                
                set_point_msg = Point()
                set_point_msg.x = float(cx)
                set_point_msg.y = float(cy)
                set_point_msg.z = 0.0

                self.set_point_pub_.publish(set_point_msg)

        cv2.imshow('camera', self.__curr_frame)
        cv2.waitKey(1)
            


def main(args=None):
    
    rclpy.init(args=args)
    cam_subscriber = CameraSubscriber()
    rclpy.spin(cam_subscriber)
    cam_subscriber.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()