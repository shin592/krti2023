import rospy
# ROS Image message
from sensor_msgs.msg import Image
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
import cv2 as cv
from std_msgs.msg import Bool,Float32,Int32
from krti2022.srv import qrfound,qrfoundResponse

# Instantiate CvBridge
bridge = CvBridge()
VERBOSE = False
class Image_feature:
    qr=False
    cv2_img = None
    def __init__(self):
            # Define your image topic
        self.image_topic = "/front_facing_camera/image_raw"
        # Set up your subscriber and define its callback
        self.qr_is_active = rospy.Service('activate_qr', qrfound, self.activate_qr)
        self.qr_bool_pub = rospy.Publisher('/qr_code/is_available',Bool,queue_size=10)
        if VERBOSE:
            print("Subscribed to {}".format(self.image_topic))

    def activate_qr(self,req):
        if req.req:
            self.img_sub = rospy.Subscriber(self.image_topic, Image, self.callback)
            self.qr_dx_pub = rospy.Publisher('/qr_code/dx',Int32,queue_size=10)
            self.qr_dy_pub = rospy.Publisher('/qr_code/dy',Int32,queue_size=10)
            self.qr = True
        else:
            self.img_sub.unregister()
            self.qr_dx_pub.unregister()
            self.qr_dy_pub.unregister()
            self.qr = False
        qrfoundResponse(self.qr)

    def callback(self, msg):
        if VERBOSE:
            print("Received an image!")
        try:
            # Convert your ROS Image message to OpenCV2
            self.cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError:
            print(Warning("Conversion failed: "))

    def read_qr(self):
        decoder = cv.QRCodeDetector()
        gray = cv.cvtColor(self.cv2_img, cv.COLOR_BGR2GRAY)
        self.decoded_text,self.qr_points,_ = decoder.detectAndDecode(gray)
        print(self.decoded_text)
        print("points : ", self.qr_points)
        if self.decoded_text is not None:
            if VERBOSE:
                print("QR Code detected: {}".format(self.decoded_text))
            self.qr_bool_pub.publish(True)
        else:
            self.qr_bool_pub.publish(False)

def main():
    img_feature = Image_feature()
    rospy.init_node('front_vision')
    
    '''
    dx and dy means difference between the center of the qr code and the center of the image
    _____________
    | -,+ | +,+ |  (x,y)
    |_____|_____| 
    | -,- | +,- |
    |_____|_____|
    '''
    while not rospy.is_shutdown():
        if img_feature.qr:
            img_feature.read_qr()
            if img_feature.qr_points is not None:
#                 [[ [354.99286 186.     ][356.00714 328.     ] [214.      328.     ][214.      186.     ] ]]
#   [356.00714 328.     ]
#   [214.      328.     ]
#   [214.      186.     ]]]
                x,y,w,h = cv.boundingRect(img_feature.qr_points)
                dx = w/2+x - img_feature.cv2_img.shape[1]/2
                dy = h/2+y - img_feature.cv2_img.shape[0]/2
                print("dx : ",dx)
                print("dy : ",dy)
                img = img_feature.cv2_img
                cv.putText(img, img_feature.decoded_text, (10, 30), cv.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                cv.rectangle(img, (x,y),(x+w,y+h),(0, 0, 255), 2)
                cv.imshow("Image window", img)
                cv.waitKey(1)
                # img_feature.qr_dx_pub.publish(dx)
                # img_feature.qr_dy_pub.publish(dy)
        else:
            img_feature.qr_bool_pub.publish(False)
        rospy.sleep(0.1)

    # Spin until ctrl + c
    rospy.spin()


if __name__ == "__main__":
    main()