import rclpy
from rclpy.node import Node
import numpy as np
import cv2

from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from std_msgs.msg import Bool
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy


# Camera intrisic parameters
K = np.array([[277.1913528442383, 0.0, 160.0], [0.0, 277.1913528442383, 120.0], [0.0, 0.0, 1.0]])
# Yellow stripes dims
STRIPE_LEN = 0.6  # m
STRIPE_WIDTH = 0.3  # m

IMG_W = 320
IMG_H = 240

MIN_STRIPE_AREA = 30


class CameraSubscriber(Node):

    def __init__(self):
        super().__init__('camera_node')
        
        print("Strting camera node...")

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.see_setpoint_pub_ = self.create_publisher(
            Bool,
            '/see_setpoint',
            10
        )

        self.img_sub_ = self.create_subscription(Image, "/camera", self.camera_cb, qos_profile=qos_profile)

        self.set_point_pub_ = self.create_publisher(Point, "/img_set_point", qos_profile=qos_profile)

        self.__curr_frame = None

    @staticmethod
    def put_text(img, midpoint, text):
        x, y = midpoint.astype(int)
        cv2.putText(img, text, (x+5, y-5), cv2.FONT_HERSHEY_SIMPLEX,
                    0.6, (0, 255, 255), 2, cv2.LINE_AA)
    
    def camera_cb(self, msg: Image):
        input_data = np.frombuffer(msg.data, dtype=np.uint8)
        current_frame_rgb = input_data.reshape([msg.height, msg.width, 3])
        self.__curr_frame = cv2.cvtColor(current_frame_rgb, cv2.COLOR_RGB2BGR)

        # Yellow stripes color bounds
        # w HSV [10, 150, 100] i [30, 255, 255]
        # lower = [0, 110,  180] # BGR
        # upper = [5, 128, 210] # BGR
        color = [0, 118, 197]
        color_hsv = cv2.cvtColor(np.array([[color]], dtype="uint8"), cv2.COLOR_BGR2HSV)[0][0]
        h_tol = 5
        s_tol = 10
        v_tol = 60
        hsv_lower = [np.clip(color_hsv[0] - h_tol, 0, 255),
                     np.clip(color_hsv[1] - s_tol, 0, 255),
                     np.clip(color_hsv[2] - v_tol, 0, 255)]
        hsv_upper = [np.clip(color_hsv[0] + h_tol, 0, 255),
                     np.clip(color_hsv[1] + s_tol, 0, 255),
                     np.clip(color_hsv[2] + v_tol, 0, 255)]
        
        mask = cv2.inRange(cv2.cvtColor(self.__curr_frame, cv2.COLOR_BGR2HSV),
                           np.array(hsv_lower, dtype="uint8"),
                           np.array(hsv_upper, dtype="uint8"))
        output = cv2.bitwise_and(self.__curr_frame, self.__curr_frame, mask=mask)

        kernel = np.ones((3,3),np.uint8)
        output = cv2.morphologyEx(output, cv2.MORPH_CLOSE, kernel)
        cv2.imshow('mask', output)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        largest_contour = None

        for contour in contours[1:]:
            # Czy kontur jest najwiekszy i czy nie dotyka krawędzi pobrazu
            if contour[:, 0, 1].max() < IMG_H - 1 and \
               contour[:, 0, 1].min() > 0 and \
               contour[:, 0, 0].max() < IMG_W - 1 and \
               contour[:, 0, 0].min() > 0:
                largest_contour = contour
                break

        if largest_contour is not None and cv2.contourArea(largest_contour) >= MIN_STRIPE_AREA:
            # --- Wyznaczanie wymiarów pasa na obrazie w pikselach ---
            # Aproksymacja do 4 punktów
            epsilon = 0.02 * cv2.arcLength(largest_contour, True)
            approx = cv2.approxPolyDP(largest_contour, epsilon, True)

            if len(approx) != 4:
                print("WARNING: Największy kontur w zadanych kolorze nie jest czworokątem!")
                return
            
            # Konwersja do prostszej tablicy
            pts = approx.reshape(4, 2)

            # Sortowanie punktów: najpierw po y (góra/dół), potem po x (lewo/prawo)
            pts = pts[np.argsort(pts[:, 1])]  # sort po Y
            top = pts[:2]
            bottom = pts[2:]

            top = top[np.argsort(top[:, 0])]        # góra: lewo, prawo
            bottom = bottom[np.argsort(bottom[:, 0])]  # dół: lewo, prawo

            tl, tr = top
            bl, br = bottom

            # Szerokość w środku pasa
            mid_left = ((tl + bl) / 2).astype(int)
            mid_right = ((tr + br) / 2).astype(int)

            mid_width = np.linalg.norm(mid_left - mid_right)
            set_point_2d = np.mean([mid_left, mid_right], axis=0).astype(int) # set point on img in pixels

            cv2.circle(self.__curr_frame, tuple(set_point_2d), 6, (0, 255, 0), -1)

            # Wizualizacja
            # Szerokość w środku pasa (biała)
            cv2.line(self.__curr_frame, tuple(mid_left), tuple(mid_right), (255, 255, 255), 2)

            self.put_text(self.__curr_frame, set_point_2d, f"{mid_width:.1f}px")

            # --- Położenie punktu w przstrzeni 3D względem kamery ---
            f = K[0,0]  # focal length

            # Odległość do pasa (Z w układzie kamery)
            Z = f * STRIPE_WIDTH / mid_width  # meters

            # Promień z normalizacją pikseli
            uv1 = np.array([set_point_2d[0], set_point_2d[1], 1.0], dtype=float)
            cam_ray = np.linalg.inv(K) @ uv1

            # Skalowanie do Z
            P_cam = cam_ray * Z
            X, Y, Z = P_cam

            # print(f"3D pos wrt camera: X={X:.3f} m, Y={Y:.3f} m, Z={Z:.3f} m")
            cv2.putText(self.__curr_frame, f"Set point position: X:{X:.3f} m, Y:{Y:.3f} m, Z:{Z:.3f} m", (0, IMG_H - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 255, 255), 1, cv2.LINE_AA)

            # Publikacja 3D punktu
            set_point_3d = Point()
            set_point_3d.x = float(X)
            set_point_3d.y = float(Y)
            set_point_3d.z = float(Z)
            self.set_point_pub_.publish(set_point_3d)

            see_setpoint_msg = Bool()
            see_setpoint_msg.data = True
            self.see_setpoint_pub_.publish(see_setpoint_msg)
        else:
            see_setpoint_msg = Bool()
            see_setpoint_msg.data = False
            self.see_setpoint_pub_.publish(see_setpoint_msg)
            

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