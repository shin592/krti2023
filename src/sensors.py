import cv2 as cv
from sensor_msgs.msg import Image, LaserScan
import rospy
import threading
from cv_bridge import CvBridge, CvBridgeError


# VERBOSE = rospy.get_param("/sensor/verbose")
class Camera:
    def __init__(self, sim=False):
        Fcamera_index = rospy.get_param("/sensors/front_camera_index")
        Dcamera_index = rospy.get_param("/sensors/down_camera_index")

        # create publisher for camera
        self.front_pub = rospy.Publisher("/sensors/front_camera", Image, queue_size=10)
        self.down_pub = rospy.Publisher("/sensors/down_camera", Image, queue_size=10)
        self.bridge = CvBridge()
        if sim:
            # subscribe to get image from gazebo
            front_sub = rospy.Subscriber(
                "/front_facing_camera/image_raw",
                Image,
                self.front_callback,
            )
            down_sub = rospy.Subscriber(
                "/down_facing_camera/image_raw",
                Image,
                self.down_callback,
            )

        else:
            self.front_cap = cv.VideoCapture(Fcamera_index)
            self.down_cap = cv.VideoCapture(Dcamera_index)
            self.front_cap.set(cv.CAP_PROP_FRAME_WIDTH, 360)
            self.front_cap.set(cv.CAP_PROP_FRAME_HEIGHT, 360)
            self.down_cap.set(cv.CAP_PROP_FRAME_WIDTH, 360)
            self.down_cap.set(cv.CAP_PROP_FRAME_HEIGHT, 360)

            self.thread1 = threading.Thread(
                target=self.publish_image, args=(self.front_cap, self.front_pub)
            )
            self.thread2 = threading.Thread(
                target=self.publish_image, args=(self.down_cap, self.down_pub)
            )

    def start_thread(self):
        self.thread1.start()
        self.thread2.start()

    def publish_image(self, cap, pub):
        ret, img = cap.read()
        try:
            img_sensor = self.bridge.cv2_to_imgmsg(img, "bgr8")
        except CvBridgeError as e:
            print(e)

        if ret:
            pub.publish(img_sensor)

    def down_callback(self, data):
        self.down_pub.publish(data)

    def front_callback(self, data):
        self.front_pub.publish(data)


class RangeFinder:
    def __init__(self, sim=False):
        self.lidar_pub = rospy.Publisher(
            "/sensors/range_finder/front", LaserScan, queue_size=10
        )
        if sim:
            self.lidar = rospy.Subscriber(
                "/spur/laser/scan", LaserScan, self.lidar_callback
            )
        else:
            pass

    def lidar_callback(self, data):
        self.lidar_pub.publish(data)

    def publish_lidar(self):
        pass


if __name__ == "__main__":
    rospy.init_node("sensors")
    r = rospy.Rate(50)
    # sim = rospy.get_param("/use_sim")
    sim = True
    cam = Camera(sim=sim)
    rf = RangeFinder(sim=sim)
    while not rospy.is_shutdown():
        if not sim:
            cam.start_thread()
            rf.publish_lidar()

        r.sleep()
