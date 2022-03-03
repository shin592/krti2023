from krti2022.drone_api import DroneAPI
from krti2022.msg import QRResult, DResult
from krti2022.srv import activate, activateResponse
import rospy
from time import sleep


class Game:
    def __init__(self):
        self.vision_activate_client = rospy.ServiceProxy("/vision/activate", activate)
        self.QR_sub = rospy.Subscriber("vision/qr/result", QRResult, self.qr_callback)
        self.elp_sub = rospy.Subscriber(
            "/vision/elp/result", DResult, self.elp_callback
        )

    def qr_callback(self, data):
        pass

    def elp_callback(self, data):
        pass

    def main(self):
        rospy.init_node("tes_main")
        # Create API object
        waypoints = [
            {"x": 3, "y": 2, "z": 2},
            {"x": 3, "y": 4, "z": 2},
            {"x": 0, "y": 0, "z": 2},
        ]

        wpt_objective = ["read qr", "landing elp", "read qr"]

        api = DroneAPI(waypoints=waypoints)
        api.wait4start()
        api.takeoff(2)

        while True:
            if api.check_waypoint_reached():
                # blabla
                if wpt_objective[api.current_waypoint] == "read qr":
                    # read qr
                    print("read qr")
                elif wpt_objective[api.current_waypoint] == "landing elp":
                    # landing elp
                    print("landing elp")

                try:
                    api.next()
                except:
                    break
            else:
                api.move()

    def main2(self):
        pass

    def main3(self):
        pass


if __name__ == "__main__":
    game = Game()
    game.main()
