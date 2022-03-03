import rospy

# ROS Image message
from sensor_msgs.msg import Image

# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError

# OpenCV2 for saving an image
import cv2 as cv
import numpy as np

from std_msgs.msg import Bool, Float32, Int32
from krti2022.srv import activate, activateResponse
from krti2022.msg import QRResult
from krti2022.msg import DResult

#   vision.py
#   This node is responsible for detecting QR codes, detect target and detect ELP
#   and publishing the QRResult to the '/qr/result' topic.
#   this node can be activated using service under '/qr/activate' topic.
#   service can be called from the command line with:
#   rosservice call /qr/activate "{req: true}" msgs type->(activate.srv)
#   or create ServiceProxy() object and call it inside python/cpp code.

#   LIST OF Service TOPIC
#   - qr/activate
#   - qr/verbose
#
#   LIST OF Publisher TOPIC
#   - qr/result
#
#   LIST OF Subscriber TOPIC
#   - front_facing_camera/image_raw
#

# TODO:
# -   add a service to request qr and target dx dy in m based on lidar data and fov
#        https://jamboard.google.com/d/1Iu5qJZLyZbIiGC8b8oDcwbF_GfKyBcttvh0o2xGcWHI/viewer?f=6


# Instantiate CvBridge
bridge = CvBridge()
# Instantiate VERBOSE variable globally for verbose mode
VERBOSE = False


class Vision:
    qr = False
    target = False
    elp = False
    front_img = None
    down_img = None
    elp_lower_hsv = [0, 100, 100]
    elp_upper_hsv = [0, 255, 255]
    name = "vision"

    def __init__(self, sim=False):
        # Define your image topic
        rospy.init_node(self.name)
        self.sim = sim
        if sim == True:
            self.front_image_topic = "/front_facing_camera/image_raw"
            self.down_image_topic = "/down_facing_camera/image_raw"
            self.lidar_topic = "/spur/laser/scan"
        else:
            self.lidar_topic = "/sensors/lidar"  # get lidar data (strength and range)
            # change the value to the appropriate camera indexes
            self.front_cam = cv.VideoCapture(0)
            self.down_cam = cv.VideoCapture(1)
            self.front_cam.set(cv.CAP_PROP_FRAME_WIDTH, 640)
            self.front_cam.set(cv.CAP_PROP_FRAME_HEIGHT, 640)
            self.front_cam.set(cv.CAP_PROP_FPS, 30)
            self.down_cam.set(cv.CAP_PROP_FRAME_WIDTH, 640)
            self.down_cam.set(cv.CAP_PROP_FRAME_WIDTH, 640)
            self.down_cam.set(cv.CAP_PROP_FPS, 30)

        self.timestamp = rospy.Time.now()
        # to start subscribing to the image_topic and starting the QR code detection
        self.activator = rospy.Service(
            self.name + "/activate", activate, self.activate_feature
        )
        self.verbose_srv = rospy.Service(
            self.name + "/verbose", activate, self.activate_verbose
        )
        # to activate verbose mode

    def activate_verbose(self, req):
        global VERBOSE
        VERBOSE = req.req
        return activateResponse(True)

    def activate_feature(self, req):
        # if activating detect and read QR
        if req.qr:
            # if using sim, so subscribe to the sim camera
            if self.sim:
                # subscribe to image_topic from SIM camera
                self.img_sub = rospy.Subscriber(
                    self.front_image_topic, Image, self.callback, callback_args="front"
                )

                # create publisher for QRResult
                self.qr_result_pub = rospy.Publisher(
                    self.name + "/qr/result", QRResult, queue_size=10
                )
                if VERBOSE:
                    print("Subscribed to {}".format(self.front_image_topic))
                self.qr = True
            # if using real robot
            else:
                self.front_img = self.front_cam.read()

        else:
            self.img_sub.unregister()
            self.qr_result_pub.unregister()
            self.qr = False

        if req.elp:
            # if using sim, so subscribe to the sim camera
            if self.sim:
                # subscribe to image_topic from SIM camera
                self.img_sub2 = rospy.Subscriber(
                    self.down_image_topic, Image, self.callback, callback_args="down"
                )
                # QRResult.msg
                # int32 dx
                # int32 dy
                # string decoded_text

                # create publisher for QRResult
                self.elp_result_pub = rospy.Publisher(
                    self.name + "/elp/result", DResult, queue_size=10
                )
                if VERBOSE:
                    print("Subscribed to {}".format(self.down_image_topic))
                self.elp = True
            # if using real robot
            else:
                self.front_img = self.down_cam.read()

        else:
            self.img_sub2.unregister()
            self.elp_result_pub.unregister()
            self.elp = False

        if req.target:
            # if using sim, so subscribe to the sim camera
            if self.sim:
                # subscribe to image_topic from SIM camera
                self.img_sub3 = rospy.Subscriber(
                    self.front_image_topic, Image, self.callback, callback_args="front"
                )
                # QRResult.msg
                # int32 dx
                # int32 dy
                # string decoded_text

                # create publisher for QRResult
                self.target_result_pub = rospy.Publisher(
                    self.name + "/target/result", QRResult, queue_size=10
                )
                if VERBOSE:
                    print("Subscribed to {}".format(self.front_image_topic))
                self.target = True
            # if using real robot
            else:
                self.front_img = self.front_cam.read()

        else:
            self.img_sub3.unregister()
            self.target_result_pub.unregister()
            self.target = False

        self.activateResponse(True)

    def activate_QR(self, req):
        if req.req:

            # subscribe to image_topic from SIM camera
            self.img_sub = rospy.Subscriber(
                self.image_topic, Image, self.callback, callback_args="front"
            )
            # QRResult.msg
            # int32 dx
            # int32 dy
            # string decoded_text

            # create publisher for QRResult
            self.qr_result_pub = rospy.Publisher("qr/result", QRResult, queue_size=10)
            if VERBOSE:
                print("Subscribed to {}".format(self.image_topic))
            self.qr = True
        else:
            # to unpublish the publisher from ros topic
            self.img_sub.unregister()
            self.qr_result_pub.unregister()
            self.qr = False
        return activateResponse(True)

    def callback(self, msg, args):
        if VERBOSE:
            print("Received an image!")
        try:
            # Convert your ROS Image message to OpenCV2
            if args == "front":
                self.front_img = bridge.imgmsg_to_cv2(msg, "bgr8")
            elif args == "down":
                self.down_img = bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(Warning("Conversion failed: {}".format(e)))

    def read_qr(self):
        decoder = cv.QRCodeDetector()
        # https://docs.opencv.org/4.5.5/de/dc3/classcv_1_1QRCodeDetector.html#a7290bd6a5d59b14a37979c3a14fbf394
        # cv.QRCodeDetector.detectAndDecode(img[, points[, straight_qrcode]]) -> retval, points, straight_qrcode
        self.decoded_text, self.qr_points, _ = decoder.detectAndDecode(self.front_img)
        if self.qr_points is not None:
            # """
            # dx and dy means difference between the center of the qr code and the center of the image
            # _____________
            # | -,+ | +,+ |  (x,y)
            # |_____|_____|
            # | -,- | +,- |
            # |_____|_____|
            # """
            # get bounding rect around the given points from detectAndDecode()
            x, y, w, h = cv.boundingRect(self.qr_points)
            dx = int(w / 2 + x - self.front_img.shape[1] // 2)
            dy = int(h / 2 + y - self.front_img.shape[0] // 2)
            self.qr_result_pub.publish(QRResult(True, dx, dy, self.decoded_text))
            if VERBOSE:
                print("QR Code detected: {}".format(self.decoded_text))
                print("points: x:{} y:{} w:{} h:{}".format(x, y, w, h))
                print("dx: {} dy: {}".format(dx, dy))
                # uncommand below code to show the QR code with the bounding box
                # cv.rectangle(self.front_img, (x, y), (x + w, y + h), (255, 0, 0), 2)
                # cv.polylines(self.front_img, np.int32(points), False, (0, 255, 0), 2)
                # cv.imshow("QR code", self.front_img)
                # cv.waitKey(1)
        else:
            self.qr_result_pub.publish(QRResult(False, 0, 0, ""))

    def detect_target(self):
        pass

    def detect_elp(self):
        MORPH_SIZE = 3
        img = self.down_cam
        hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
        mask = cv.inRange(hsv, self.elp_lower_hsv, self.elp_upper_hsv)
        # FILTER
        # element1 = cv.getStructuringElement(cv.MORPH_RECT, (3, 3))
        # mask2 = cv.erode(mask, element1, iterations=1)
        # element2 = cv.getStructuringElement(cv.MORPH_RECT, (3, 3))
        # mask = cv.dilate(mask, element2, iterations=1)
        # element3 = cv.getStructuringElement(cv.MORPH_RECT, (3, 3))
        # mask = cv.erode(mask, element3)
        res = cv.bitwise_and(img, img, mask=mask)
        gray = cv.cvtColor(res, cv.COLOR_BGR2GRAY)
        element4 = cv.getStructuringElement(
            cv.MORPH_RECT, (2 * MORPH_SIZE, 2 * MORPH_SIZE), (MORPH_SIZE, MORPH_SIZE)
        )
        output_morph_opening = cv.morphologyEx(
            gray, cv.MORPH_CLOSE, element4, anchor=(-1, 1), iterations=2
        )
        contours, hierarchy = cv.findContours(
            output_morph_opening, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE
        )
        area_list = map(cv.contourArea, contours)
        max_area = max(area_list)
        index = area_list.index(max_area)
        cnt = contours[index]
        cv.drawContours(img, cnt, -1, (255, 125, 60), 3)
        peri = cv.arcLength(cnt, True)
        approx = cv.approxPolyDP(cnt, 0.02 * peri, True)
        x, y, w, h = cv.boundingRect(approx)
        cv.rectangle(res, (x, y), (x + w, y + h), (0, 255, 0), 2)
        self.elp_result_pub.publish(DResult(True, x + w / 2, y + h / 2, "ELP"))
        # if show_Img:
        cv.imshow("gray", gray)
        cv.imshow("obj", res)
        cv.waitKey(1)

    def main(self):
        last = rospy.Time.now()
        while not rospy.is_shutdown():
            if self.qr:
                self.read_qr()
            else:
                pass
                # printed every 5s so we can see the node still running & doesn't spam the console
                # if VERBOSE and rospy.Time.now() - last > rospy.Duration(5):
                #     last = rospy.Time.now()
                #     print("QR is not active")
            if self.elp:
                self.detect_elp()
            else:
                pass

            if self.target:
                self.detect_target()
            else:
                pass

            rospy.sleep(0.1)


if __name__ == "__main__":
    try:
        img_feature = Vision()
        img_feature.main()
    except rospy.ROSInterruptException:
        pass
