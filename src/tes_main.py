from krti2022.drone_api import DroneAPI
from krti2022.msg import QRResult, DResult
from krti2022.srv import Activate, ActivateRequest, ActivateResponse
import rospy
from time import sleep


class Game:
    def __init__(self):
        rospy.init_node("tes_main")
        self.activate_qr = rospy.ServiceProxy("/vision/activate/qr", Activate)
        self.activate_elp = rospy.ServiceProxy("/vision/activate/elp", Activate)
        self.activate_target = rospy.ServiceProxy("/vision/activate/target", Activate)
        self.QR_sub = rospy.Subscriber("vision/qr/result", QRResult, self.qr_callback)
        self.elp_sub = rospy.Subscriber(
            "/vision/elp/result", DResult, self.elp_callback
        )
        self.elp_data = DResult()
        self.elp_data.is_found = False

        waypoints = [
            {"x": -4.8, "y": 5, "z": 2},
            {"x": -2.5, "y": 4, "z": 2},
        ]
        self.drone = DroneAPI(waypoints=waypoints)

    def qr_callback(self, data):
        print(type(data.data))
        print("data : {} dx : {} dy : {} ".format(data.data, data.dx, data.dy))

    def elp_callback(self, data):
        print("dx : {} dy : {} ".format(data.dx, data.dy))
        self.elp_data = data

    def land_algorithm(self, cur_pose):
        now = rospy.Time.now()
        x_done = False
        y_done = False

        while self.elp_data.is_found:
            cur_pose = {
                "x": self.drone.current_pose.pose.pose.position.x,
                "y": self.drone.current_pose.pose.pose.position.y,
                "z": self.drone.current_pose.pose.pose.position.z,
            }

            if self.elp_data.dx > 10:
                print("move right")
                cur_pose["x"] += 0.2
            elif self.elp_data.dx < -10:
                print("move left")
                cur_pose["x"] -= 0.2
            else:
                x_done = True
            if self.elp_data.dy > 10:
                print("move backward")
                cur_pose["y"] -= 0.2

            elif self.elp_data.dy < -10:
                print("move forward")
                cur_pose["y"] += 0.2
            else:
                y_done = True
            if x_done and y_done and cur_pose["z"] > 1.2:
                cur_pose["z"] = 1
                self.drone.move(cur_pose)
                continue
            elif x_done and y_done and cur_pose["z"] < 1.2 and cur_pose["z"] > 0.6:
                cur_pose["z"] = 0.5
            elif x_done and y_done and cur_pose["z"] < 0.3:
                rospy.loginfo("landing on elp complete activating land mode")
                self.drone.set_mode("LAND")
                return 0

            if rospy.Time.now() - now > rospy.Duration(0.2):
                cur_pose["z"] -= 0.15
                now = rospy.Time.now()

            print(cur_pose)
            self.drone.move(cur_pose)
            sleep(0.5)
            print("landing elp")
        return 1

    def main(self):
        # rospy.init_node("tes_main")
        # rospy.Rate(10)
        # Create API object

        wpt_objective = ["read qr", "landing elp"]

        # self.drone = DroneAPI(waypoints=waypoints)
        self.drone.wait4start()
        self.drone.takeoff(2)
        self.drone.next()
        land = False
        try:
            while True:
                try:
                    if self.drone.check_waypoint_reached() and not land:
                        # blabla
                        if wpt_objective[self.drone.current_waypoint] == "read qr":
                            # read qr
                            print("read qr")
                            self.activate_qr(ActivateRequest(True))
                            sleep(2)
                        elif (
                            wpt_objective[self.drone.current_waypoint] == "landing elp"
                        ):
                            cur_pose = {
                                "x": self.drone.current_pose.pose.pose.position.x,
                                "y": self.drone.current_pose.pose.pose.position.y,
                                "z": self.drone.current_pose.pose.pose.position.z,
                            }
                            # landing elp
                            self.activate_elp(ActivateRequest(True))
                            self.land_algorithm(cur_pose)
                            land = True
                        try:
                            self.drone.next()
                        except IndexError:
                            break
                    elif land:
                        self.land_algorithm(cur_pose)
                    else:
                        self.drone.move()
                except KeyboardInterrupt:
                    self.drone.set_mode("LAND")
                    exit()
                except IndexError:
                    break
        except KeyboardInterrupt:
            self.drone.set_mode("LAND")
            exit()

    def main2(self):

        r = rospy.Rate(50)
        # Create API object
        # waypoints = [
        #     {"x": -4.8, "y": 5, "z": 2},
        #     {"x": -2, "y": 4, "z": 2},
        # ]
        while not rospy.is_shutdown():
            wpt_objective = ["read qr", "landing elp"]

            # self.drone = DroneAPI(waypoints=waypoints)
            self.drone.wait4start()
            self.drone.takeoff(2)
            self.drone.next()
            try:
                self.drone.move()

                while not self.drone.check_waypoint_reached():
                    self.drone.move()
                    rospy.sleep(0.1)
                # landing elp
                self.activate_elp(ActivateRequest(True))
                cur_pose = {
                    "x": self.drone.current_pose.pose.pose.position.x,
                    "y": self.drone.current_pose.pose.pose.position.y,
                    "z": self.drone.current_pose.pose.pose.position.z,
                }
                rospy.sleep(1)
                while self.land_algorithm(cur_pose):
                    pass

            except KeyboardInterrupt:
                self.drone.set_mode("LAND")
                exit()
            except IndexError:
                pass
        r.sleep()

    def main3(self):
        pass


if __name__ == "__main__":
    game = Game()
    try:
        game.main2()
    except KeyboardInterrupt:
        game.drone.set_mode("LAND")
        exit()
    except rospy.ROSInterruptException:
        pass
