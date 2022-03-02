from time import sleep
from krti2022.drone_api import DroneAPI
from krti2022.msg import QRResult, DResult
from krti2022.srv import activate, activateResponse
import mavros
import rospy
from math import radians, cos, sin
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from geographic_msgs.msg import GeoPointStamped

mavros.set_namespace()


class Game:
    def __init__(self):
        self.vision_activate_client = rospy.ServiceProxy(
            mavros.get_topic("/vision/activate"), activate
        )
        self.QR_sub = rospy.Subscriber(
            mavros.get_topic("vision/qr/result"), QRResult, self.qr_callback
        )
        self.elp_sub = rospy.Subscriber(
            mavros.get_topic("/vision/elp/result"), DResult, self.elp_callback
        )

    def qr_callback(self, data):
        pass

    def elp_callback(self, data):
        pass

    def main(self):
        rospy.init_node("tes_main")
        # Create API object
        waypoints = [
            {"x": 7, "y": 4, "z": 2},
            {"x": 3, "y": -2, "z": 0},
            {"x": 0, "y": 0, "z": 0},
        ]
        api = DroneAPI()
        api.wait4start()
        # api.takeoff(2)
        # api.move()
        # api.next()
        # # self.vision_activate_client.call(activate(True, False, False))

        # sleep(0.2)
        # api.move()
        # api.next()

        # api.land()

    def main2(self):
        pass

    def main3(self):
        pass


if __name__ == "__main__":
    game = Game()
    game.main()
