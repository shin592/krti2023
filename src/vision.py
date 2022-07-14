import rospy

# ROS Image message
from sensor_msgs.msg import Image

# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError

# OpenCV2 for saving an image
import cv2 as cv
import numpy as np

# custom srv and msg
from krti2022.srv import Activate, ActivateResponse
from krti2022.msg import QRResult
from krti2022.msg import DResult

# get range from qr/target/elp
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from math import tan, radians
import random as rng

#   LIST OF Service TOPIC
#   - vision/activate/qr
#   - vision/activate/target
#   - vision/activate/elp
#   - vision/verbose
#
#   LIST OF Publisher TOPIC
#   - vision/result
#
#   LIST OF Subscriber TOPIC
#   - front_facing_camera/image_raw
#   - down_facing_camera/image_raw
#

# TODO:
# -   add a service to request qr and target dx dy in m based on lidar data and fov
#        https://jamboard.google.com/d/1Iu5qJZLyZbIiGC8b8oDcwbF_GfKyBcttvh0o2xGcWHI/viewer?f=6


# Instantiate CvBridge for converting ROS Image messages to OpenCV2
bridge = CvBridge()
# Instantiate VERBOSE variable globally for verbose mode
VERBOSE = False


class Vision:
    """
    vision.py
    This Node is responsible for detecting QR codes, detect target and detect ELP
    and publishing the QRResult to the '/vision/qr/result' topic.
    to activate QR code, target, and elp detection we should activate
    using service under '/vision/activate' topic.
    """

    qr = False
    target = False
    elp = False

    front_img = np.array([None for _ in range(10)])
    down_img = np.array([None for _ in range(10)])

    lidar_range = -1
    alt = -99
    max_val = 0
    max_index = -1

    def __init__(self):
        # initialize node
        rospy.init_node("vision")

        # get parameter from launch file
        VERBOSE = rospy.get_param("/vision/verbose")
        self.sim = rospy.get_param("/use_sim")
        self.elp_lower_hsv = np.array(rospy.get_param("/vision/elp_lower_hsv"))
        self.elp_upper_hsv = np.array(rospy.get_param("/vision/elp_upper_hsv"))
        self.target_lower_hsv = np.array(rospy.get_param("/vision/target_lower_hsv"))
        self.target_upper_hsv = np.array(rospy.get_param("/vision/target_upper_hsv"))
        self.show_image = rospy.get_param("/vision/show_img")
        self.front_fov = {
            "x": rospy.get_param("/vision/front_fov_x"),
            "y": rospy.get_param("/vision/front_fov_y"),
        }
        self.down_fov = {
            "x": rospy.get_param("/vision/down_fov_x"),
            "y": rospy.get_param("/vision/down_fov_y"),
        }
        if VERBOSE:
            print("use sim : {}".format(self.sim))
            print("elp lower hsv : {}".format(self.elp_lower_hsv))
            print("elp upper hsv : {}".format(self.elp_upper_hsv))
            print("target lower hsv : {}".format(self.target_lower_hsv))
            print("target upper hsv : {}".format(self.target_upper_hsv))
            print("show image : {}".format(self.show_image))
            print("front fov : {}".format(self.front_fov))
            print("down fov : {}".format(self.down_fov))

        # subscribe to sensors/camera to get image
        self.front_image_topic = "/sensors/front_camera"
        self.down_image_topic = "/sensors/down_camera"

        self.timestamp = rospy.Time.now()
        # to start subscribing to the image_topic and starting the QR code detection
        self.activate_qr = rospy.Service(
            "/vision/activate/qr", Activate, self.activate_QR
        )
        self.activate_target = rospy.Service(
            "vision/activate/target", Activate, self.activate_target
        )
        self.activate_elp = rospy.Service(
            "vision/activate/elp", Activate, self.activate_elp
        )
        self.verbose_srv = rospy.Service(
            "/vision/verbose", Activate, self.activate_verbose
        )
        self.lidar_sub = rospy.Subscriber(
            "/sensors/range_finder/front", LaserScan, self.lidar_cb
        )

        # PUBLISHER
        # create publisher for returnin the QR detection and reading result
        self.qr_result_pub = rospy.Publisher(
            "vision/qr/result", QRResult, queue_size=10
        )
        # create publisher for elp detection
        self.elp_result_pub = rospy.Publisher(
            "/vision/elp/result", DResult, queue_size=10
        )

        # create publisher for target detection
        self.target_result_pub = rospy.Publisher(
            "/vision/target/result", DResult, queue_size=10
        )

    def activate_verbose(self, req):
        """
        This function called when the service '/vision/verbose' is called.
        It activates the verbose mode.
        """
        global VERBOSE
        VERBOSE = req.req
        return ActivateResponse(True)

    def activate_QR(self, data):
        """
        This function called when the service '/vision/activate/qr' is called.
        It activates the QR code detection.
        """
        rospy.loginfo("QR code detect activated")
        if data.data:
            # if so subscribe to the camera

            self.img_sub = rospy.Subscriber(
                self.front_image_topic, Image, self.callback, callback_args="front"
            )
            if VERBOSE:
                print("Subscribed to {}".format(self.front_image_topic))

            self.qr = True

        else:
            # to unpublish the publisher from ros topic
            self.img_sub.unregister()
            self.qr = False
            if self.show_image:
                cv.destroyAllWindows()

        return ActivateResponse(True)

    def activate_elp(self, data):
        """
        This function called when the service '/vision/activate/elp' is called.
        It activates the ELP detection.
        """
        rospy.loginfo("ELP detect activated")
        if data.data:
            # subscribe to image_topic from sensors/camera
            self.img_sub2 = rospy.Subscriber(
                self.down_image_topic, Image, self.callback, callback_args="down"
            )
            self.pose_sub = rospy.Subscriber(
                "/mavros/local_position/odom", Odometry, self.pose_cb
            )
            if VERBOSE:
                print("Subscribed to {}".format(self.down_image_topic))

            self.elp = True

        else:
            self.img_sub2.unregister()
            self.elp = False
            if self.show_image:
                cv.destroyAllWindows()
        return ActivateResponse(True)

    def activate_target(self, data):
        """
        This function called when the service '/vision/activate/target' is called.
        It activates the target detection.
        """
        rospy.loginfo("Target detect activated")
        if data.data:

            # subscribe to image_topic from camera
            self.img_sub3 = rospy.Subscriber(
                self.front_image_topic, Image, self.callback, callback_args="front"
            )
            if VERBOSE:
                print("Subscribed to {}".format(self.front_image_topic))
            # if using real robot
            self.target = True

        else:
            self.img_sub3.unregister()
            self.target = False
            if self.show_image:
                cv.destroyAllWindows()
        return ActivateResponse(True)

    def callback(self, msg, args):
        """
        This function is called when the image_topic is published.
        It gets the image from the topic and convert from ROS Image msgs to OpenCV2 Image.
        """
        if VERBOSE:
            print("Received an image!", args)
        try:
            # Convert your ROS Image message to OpenCV2
            if args == "front":
                self.front_img = bridge.imgmsg_to_cv2(msg, "bgr8")
            elif args == "down":
                self.down_img = bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(Warning("Conversion failed: {}".format(e)))

    def lidar_cb(self, msg):
        """
        This function is called when the lidar_topic is published.
        """
        # save current lidar range value
        self.lidar_range = msg.range_max

    def pose_cb(self, msg: Odometry):
        """
        Gets the raw pose of the drone and processes it for use in control.

        Args:
                msg (nav_msgs/Odometry): Raw pose of the drone.
        """
        # save current alt
        self.alt = msg.pose.pose.position.z

    def read_qr(self):
        """
        This function is called when the QR code detection is activated.
        this function called from the main
        It reads the QR code from the image and publishes the result.
        https://docs.opencv.org/4.5.5/de/dc3/classcv_1_1QRCodeDetector.html#a7290bd6a5d59b14a37979c3a14fbf394
        It will publish the QR code result to 'vision/qr/result'.
        the result is a QRResult msg.
        """
        img = self.front_img  # maybe we could crop to scan QR faster
        decoder = cv.QRCodeDetector()
        # cv.QRCodeDetector.detectAndDecode(img[, points[, straight_qrcode]]) -> retval, points, straight_qrcode
        self.decoded_text, qr_points, _ = decoder.detectAndDecode(img)
        if qr_points is not None:
            # """
            # dx and dy means difference between the center of the qr code and the center of the image
            # _____________
            # | -,+ | +,+ |  (x,y)
            # |_____|_____|
            # | -,- | +,- |
            # |_____|_____|
            # """
            # get bounding rect around the given points from detectAndDecode()
            x, y, w, h = cv.boundingRect(qr_points)
            # get image size
            Fwidth = img.shape[1]
            Fheight = img.shape[0]

            # get the difference between the center of the qr code
            # and the center of the image frame in pixels
            dx = int(w / 2 + x - Fwidth // 2)
            dy = int(h / 2 + y - Fheight // 2)

            # get the difference in meters, currently not working as expected
            x_m, y_m = self.calculate_meter_from_pixel(dx, dy, Fwidth, Fheight)
            # publish the result
            if self.decoded_text is not None and self.decoded_text != "":
                self.qr_result_pub.publish(
                    QRResult(
                        True,
                        dx,
                        dy,
                        x_m,
                        y_m,
                        self.decoded_text,
                    )
                )
            else:
                self.qr_result_pub.publish(
                    QRResult(
                        False,
                        dx,
                        dy,
                        x_m,
                        y_m,
                        self.decoded_text,
                    )
                )
            if VERBOSE:
                print("QR Code detected: {}".format(self.decoded_text))
                print("points: x:{} y:{} w:{} h:{}".format(x, y, w, h))
                print("dx: {} dy: {}".format(dx, dy))
            if self.show_image:
                # draw the bounding box
                cv.rectangle(img, (x, y), (x + w, y + h), (255, 0, 0), 2)
                # show the image
                cv.imshow("QR code", img)
                cv.waitKey(1)

    def detect_target(self):
        """
        This function is called when the target detection is activated.
        this function called from the main
        It detects the target from the image and publishes the result.
        It will publish to the /vision/target/result topic
        with msg type DResult
        """
        img = self.down_img
        Fwidth = img.shape[1]
        Fheight = img.shape[0]
        FWcenter = Fwidth // 2
        FHcenter = Fheight // 2
        hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
        mask = cv.inRange(hsv, self.target_lower_hsv, self.target_upper_hsv)

        # FILTER
        # morph size for the filter
        MORPH_SIZE = 3
        # create kernel for filter
        element = cv.getStructuringElement(
            cv.MORPH_RECT, (2 * MORPH_SIZE, 2 * MORPH_SIZE), (MORPH_SIZE, MORPH_SIZE)
        )

        # morphological transformation:
        # https://www.youtube.com/watch?v=xSzsD4kXhRw
        # apply filter morphology opening to the image
        # erode and dilate to remove noise
        mask_opening = cv.morphologyEx(mask, cv.MORPH_OPEN, element, iterations=1)
        # apply filter morphology closing to the image
        # dilate and erode to fill holes
        mask_closing = cv.morphologyEx(
            mask_opening, cv.MORPH_CLOSE, element, iterations=2
        )
        # find contours in the masked and filtered image
        contours, hierarchy = cv.findContours(
            mask_closing, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE
        )

        # isolate object from background
        res = cv.bitwise_and(img, img, mask=mask_closing)

        # find the biggest contour
        area_list = [None] * len(contours)
        contours_poly = [None] * len(contours)
        boundRect = [None] * len(contours)
        max_val = 0
        max_index = -1

        for i, c in enumerate(contours):
            # calculate area of the contours
            area_list[i] = cv.contourArea(c)
            if area_list[i] > max_val:
                # save max area and index
                max_val = area_list[i]
                max_index = i
                contours_poly = cv.approxPolyDP(c, 3, True)
                boundRect = cv.boundingRect(contours_poly[i])

        # if elp detected
        if max_index != -1:
            x, y, w, h = boundRect
            # calculate the difference between the center
            # of the elp and the center of the image frame
            dx = int(w / 2 + x - FWcenter)
            dy = int(h / 2 + y - FHcenter)
            # calculate the difference in meters, currently not working as expected
            x_m, y_m = self.calculate_meter_from_pixel(dx, dy, Fwidth, Fheight)
            print("dx:", dx, "dy:", dy, "x_m:", x_m, "y_m:", y_m)
            self.elp_result_pub.publish(DResult(True, dx, dy, x_m, y_m))
        else:
            # if no elp is detected, publish false
            self.elp_result_pub.publish(DResult(False, 0, 0, 0, 0))

        if self.show_image:
            # copy image, so that the original image is not modified
            img_copy = img.copy()

            if len(contours) > 0:
                for i in range(len(contours)):
                    color = (
                        rng.randint(0, 256),
                        rng.randint(0, 256),
                        rng.randint(0, 256),
                    )
                    cv.drawContours(img_copy, contours, i, (0, 0, 255), 2)
                # draw bounding box around the biggest contour
                cv.rectangle(
                    img_copy,
                    (int(boundRect[0]), int(boundRect[1])),
                    (
                        int(boundRect[0] + boundRect[2]),
                        int(boundRect[1] + boundRect[3]),
                    ),
                    color,
                    2,
                )
                # draw line from center of image to center of contour
                cv.line(
                    img_copy,
                    (FWcenter, FHcenter),
                    (FWcenter + dx, FHcenter + dy),
                    color,
                    3,
                )
                cv.circle(img_copy, (FWcenter + dx, FHcenter), 3, (0, 255, 255), -1)
                cv.circle(img_copy, (FWcenter, FWcenter + dy), 3, (0, 255, 255), -1)
                cv.putText(
                    img_copy,
                    "dx:" + str(dx),
                    (FWcenter + dx // 2, FHcenter + 10),
                    cv.FONT_HERSHEY_PLAIN,
                    1,
                    (0, 100, 255),
                    1,
                )
                cv.putText(
                    img_copy,
                    "dy:" + str(dy),
                    (FWcenter - 10, FHcenter + dy // 2),
                    cv.FONT_HERSHEY_PLAIN,
                    1,
                    (0, 100, 255),
                    1,
                )
            # draw horizontal line
            cv.line(
                img_copy,
                (0, FHcenter),
                (Fwidth, FHcenter),
                (0, 255, 0),
                2,
            )
            cv.line(
                img_copy,
                (FWcenter, 0),
                (FWcenter, Fheight),
                (0, 255, 0),
                2,
            )

            # cv.rectangle(res, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv.imshow("img_copy", img_copy)
            # cv.imshow("obj", res)
            # cv.imshow("hsv", hsv)
            # cv.imshow("mask", mask)
            cv.waitKey(1)

    def detect_elp(self, hough_circle):
        """
        This function is called when the elp detection is activated.
        this function called from the main
        It detects the elp from the image and publishes the result.
        It will publish to the /vision/elp/result topic
        with msg type DResult
        """
        img = self.down_img
        img_copy = img.copy()
        Fwidth = img.shape[1]
        Fheight = img.shape[0]
        FWcenter = Fwidth // 2
        FHcenter = Fheight // 2
        hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
        mask = cv.inRange(hsv, self.elp_lower_hsv, self.elp_upper_hsv)

        # FILTER
        # morph size for the filter
        MORPH_SIZE = 3
        # create kernel for filter
        element = cv.getStructuringElement(
            cv.MORPH_RECT, (2 * MORPH_SIZE, 2 * MORPH_SIZE), (MORPH_SIZE, MORPH_SIZE)
        )

        # morphological transformation:
        # https://www.youtube.com/watch?v=xSzsD4kXhRw
        # apply filter morphology opening to the image
        # erode and dilate to remove noise
        mask_opening = cv.morphologyEx(mask, cv.MORPH_OPEN, element, iterations=1)
        # apply filter morphology closing to the image
        # dilate and erode to fill holes
        mask_closing = cv.morphologyEx(
            mask_opening, cv.MORPH_CLOSE, element, iterations=2
        )
        if hough_circle and self.alt > 1:
            contours = []
            circles = cv.HoughCircles(
                mask_closing,
                cv.HOUGH_GRADIENT,
                1.8,
                minDist=Fwidth * 2 / 3,
                param1=150,
                param2=44,
                minRadius=20,
                maxRadius=0,
            )
            if circles is not None:
                rospy.loginfo("cirlce found")
                circles = np.uint16(np.around(circles))
                for i in circles[0, :]:
                    color = (
                        rng.randint(0, 256),
                        rng.randint(0, 256),
                        rng.randint(0, 256),
                    )
                    center = (i[0], i[1])
                    radius = i[2]
                    cv.circle(img_copy, center, radius, color, 3)
                    dx = i[0] - FWcenter
                    dy = i[1] - FHcenter
                    x_m, y_m = self.calculate_meter_from_pixel(dx, dy, Fwidth, Fheight)
                    # print("dx:", dx, "dy:", dy, "x_m:", x_m, "y_m:", y_m)
                    self.elp_result_pub.publish(DResult(True, dx, dy, x_m, y_m))
            self.elp_result_pub.publish(DResult(False, 0, 0, 0, 0))

        else:
            # find contours in the masked and filtered image
            contours, hierarchy = cv.findContours(
                mask_closing, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE
            )
            # isolate object from background
            res = cv.bitwise_and(img, img, mask=mask_closing)

            # find the biggest contour
            area_list = [None] * len(contours)
            contours_poly = [None] * len(contours)
            for i, c in enumerate(contours):
                # calculate area of the contours
                area_list[i] = cv.contourArea(c)
                # print(f"c: {c} area : {area_list[i]} ")
                if area_list[i] >= self.max_val:
                    # save max area and index
                    self.max_val = area_list[i]
                    self.max_index = i
                contours_poly[i] = cv.approxPolyDP(c, 3, True)

            # if elp detected
            if self.max_index != -1:
                x, y, w, h = cv.boundingRect(contours_poly[self.max_index])
                # calculate the difference between the center
                # of the elp and the center of the image frame
                dx = int(w / 2 + x - FWcenter)
                dy = int(h / 2 + y - FHcenter)
                # calculate the difference in meters, currently not working as expected
                x_m, y_m = self.calculate_meter_from_pixel(dx, dy, Fwidth, Fheight)
                # print("dx:", dx, "dy:", dy, "x_m:", x_m, "y_m:", y_m)
                self.elp_result_pub.publish(DResult(True, dx, dy, x_m, y_m))
            else:
                dx = 0
                dy = 0
                # if no elp is detected, publish false
                self.elp_result_pub.publish(DResult(False, 0, 0, 0, 0))

        if self.show_image:
            # copy image, so that the original image is not modified

            if len(contours) > 0 or circles is not None:
                # for i in range(len(contours)):
                color = (
                    rng.randint(0, 256),
                    rng.randint(0, 256),
                    rng.randint(0, 256),
                )
                # draw bounding box around the biggest contour
                if not hough_circle:
                    cv.drawContours(img_copy, contours_poly, self.max_index, color, 3)
                    cv.rectangle(
                        img_copy,
                        (int(x), int(y)),
                        (
                            int(x + w),
                            int(y + h),
                        ),
                        color,
                        2,
                    )
                # draw line from center of image to center of contour
                cv.line(
                    img_copy,
                    (FWcenter, FHcenter),
                    (FWcenter + dx, FHcenter + dy),
                    color,
                    3,
                )
                cv.circle(img_copy, (FWcenter + dx, FHcenter + dy), 5, color, -1)
                cv.putText(
                    img_copy,
                    "dx:" + str(dx),
                    (FWcenter + dx // 2, FHcenter + 10),
                    cv.FONT_HERSHEY_PLAIN,
                    1,
                    (0, 100, 255),
                    1,
                )
                cv.putText(
                    img_copy,
                    "dy:" + str(dy),
                    (FWcenter - 10, FHcenter + dy // 2),
                    cv.FONT_HERSHEY_PLAIN,
                    1,
                    (0, 100, 255),
                    1,
                )
            # draw horizontal line
            cv.line(
                img_copy,
                (0, FHcenter),
                (Fwidth, FHcenter),
                (0, 255, 0),
                2,
            )
            cv.line(
                img_copy,
                (FWcenter, 0),
                (FWcenter, Fheight),
                (0, 255, 0),
                2,
            )

            # if not hough_circle:
            #     cv.imshow("obj", res)
            cv.imshow("img", img_copy)
            # cv.imshow("hsv", hsv)
            cv.imshow("mask", mask_closing)
            cv.waitKey(1)

    def calculate_meter_from_pixel(self, dx, dy, Fw, Fh, Front=True):
        """
        this function is used to calculate the position in meter we should move
        based on error in pixel, lidar, and the fov of the camera
        https://jamboard.google.com/d/1lls6bwxasvXhjlHUzlAPdn7H457EWCQQhZ9MEsxx3u0
        """
        if Front:
            if self.lidar_range == -1:
                return 0, 0
            Rx = tan(radians(self.front_fov["x"] / 2)) * self.lidar_range
            Ry = tan(radians(self.front_fov["y"] / 2)) * self.lidar_range
        else:
            if self.alt == -99:
                return 0, 0
            Rx = tan(radians(self.down_fov["x"] / 2)) * self.alt
            Ry = tan(radians(self.down_fov["y"] / 2)) * self.alt
        x = dx * Rx / (Fw / 2)
        y = dy * Ry / (Fh / 2)
        return float(x), float(y)

    def main(self):
        last = rospy.Time.now()
        r = rospy.Rate(50)
        use_hough_circle = rospy.get_param("~use_hough_circle", False)
        while not rospy.is_shutdown():
            if rospy.Time.now() - last > rospy.Duration(5):
                rospy.loginfo("[Vision] Heartbeat every 5 seconds")
                last = rospy.Time.now()

            if self.qr:
                if self.front_img.all() != None:
                    self.read_qr()
            else:
                pass
            if self.elp:
                if self.down_img.all() != None:
                    self.detect_elp(hough_circle=use_hough_circle)
            else:
                pass
            if self.target:
                if self.front_img.all() != None:
                    self.detect_target()
            else:
                pass
            r.sleep()


if __name__ == "__main__":
    try:
        img_feature = Vision()
        img_feature.main()
    except rospy.ROSInterruptException:
        pass
