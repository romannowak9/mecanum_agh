import rclpy
from rclpy.node import Node
import numpy as np
import cv2

from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy


class CameraSubscriber(Node):

    def __init__(self):
        super().__init__('camera_publisher')
        
        print("Strting camera node...")

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.img_sub_ = self.create_subscription(Image, "/camera", self.camera_cb, qos_profile=qos_profile)
        self.__curr_frame = None
    
    def camera_cb(self, msg: Image):
        input_data = np.frombuffer(msg.data, dtype=np.uint8)
        current_frame_rgb = input_data.reshape([msg.height, msg.width, 3])
        self.__curr_frame = cv2.cvtColor(current_frame_rgb, cv2.COLOR_RGB2BGR)
        # self.__curr_frame = cv2.cvtColor(current_frame_rgb, cv2.COLOR_RGB2GRAY)

        # Tu trzeba usunąć to wyświetlanie i robić coś z tym obrazem
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