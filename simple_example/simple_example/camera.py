import rclpy
from rclpy.node import Node
import numpy as np
import cv2

from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy


# Camera intrisic parameters
K = np.array([[277.1913528442383, 0.0, 160.0], [0.0, 277.1913528442383, 120.0], [0.0, 0.0, 1.0]])
# Yellow stripes dims
STRIPE_LEN = 0.6  # m
STRIPE_WIDTH = 0.3  # m

IMG_W = 320
IMG_H = 240


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

    @staticmethod
    def put_text(img, midpoint, text):
        x, y = midpoint.astype(int)
        cv2.putText(img, text, (x+5, y-5), cv2.FONT_HERSHEY_SIMPLEX,
                    0.6, (0, 255, 255), 2, cv2.LINE_AA)
    
    def camera_cb(self, msg: Image):
        input_data = np.frombuffer(msg.data, dtype=np.uint8)
        current_frame_rgb = input_data.reshape([msg.height, msg.width, 3])
        self.__curr_frame = cv2.cvtColor(current_frame_rgb, cv2.COLOR_RGB2BGR)
        # self.__curr_frame = cv2.cvtColor(current_frame_rgb, cv2.COLOR_RGB2GRAY)

        # Yellow stripes color bounds
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
            # Czy kontur jest najwiekszy i czy nie dotyka krawędzi pobrazu
            if area > max_area and contour[:, 0, 1].max() < IMG_H - 1:
                # and contour[:, 0, 1].min() > 0 and contour[:, 0, 0].max() < IMG_W - 1 and contour[:, 0, 0].min() > 0:
                max_area = area
                largest_contour = contour

        if largest_contour is not None:
            M = cv2.moments(largest_contour)
            if M['m00'] != 0:
                # --- Środek największego konturu pasa - zadany punkt ---
                # cx = int(M['m10'] / M['m00'])
                # cy = int(M['m01'] / M['m00'])

                # set_point_msg = Point()
                # set_point_msg.x = float(cx)
                # set_point_msg.y = float(cy)
                # set_point_msg.z = 0.0

                # self.set_point_pub_.publish(set_point_msg)

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

                # Szerokości (dalsza i bliższa) pasa widocznego na ekranie w piksela
                # top_width = np.linalg.norm(tl - tr)
                # bottom_width = np.linalg.norm(bl - br)

                # Szerokość w środku pasa
                mid_left = ((tl + bl) / 2).astype(int)
                mid_right = ((tr + br) / 2).astype(int)

                mid_width = np.linalg.norm(mid_left - mid_right)
                set_point_2d = np.mean([mid_left, mid_right], axis=0).astype(int) # set point on img in pixels

                cv2.circle(self.__curr_frame, tuple(set_point_2d), 6, (0, 255, 0), -1)

                # Długość pasa widocznego na ekranie w pikselach
                # mid_top = np.mean([tl, tr], axis=0).astype(int)
                # mid_bottom = np.mean([bl, br], axis=0).astype(int)

                # stripe_len = np.linalg.norm(mid_top - mid_bottom).astype(int)

                # Wizualizacja
                # górna krawędź (zielona)
                # cv2.line(self.__curr_frame, tuple(tl), tuple(tr), (0, 255, 0), 2)
                # # dolna krawędź (niebieska)
                # cv2.line(self.__curr_frame, tuple(bl), tuple(br), (255, 0, 0), 2)
                # # długość (czerwona)
                # cv2.line(self.__curr_frame, tuple(mid_top), tuple(mid_bottom), (0, 0, 255), 2)
                # Szerokość w środku pasa (biała)
                cv2.line(self.__curr_frame, tuple(mid_left), tuple(mid_right), (255, 255, 255), 2)

                # self.put_text(self.__curr_frame, mid_top, f"{top_width:.1f}px")
                # self.put_text(self.__curr_frame, mid_bottom, f"{bottom_width:.1f}px")
                # self.put_text(self.__curr_frame, np.mean([mid_bottom, mid_top], axis=0).astype(int), f"{stripe_len:.1f}px")
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