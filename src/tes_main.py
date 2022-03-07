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
        self.qr_data = QRResult()
        self.elp_data.is_found = False

        waypoints = [
            {"x": -5, "y": 5.5, "z": 2},  # QR gedung 2m
            {"x": -5, "y": 7, "z": 2.6},  # target gedung 2m
            {"x": 2.3, "y": 7, "z": 1.5},  # QR gedung 1.5m
            {"x": 2.3, "y": 8.3, "z": 2.3},  # target gedung 1.5m
            {"x": -1.5, "y": 8.95, "z": 1.6},  # target gedung 1m
            {"x": -2.5, "y": 4, "z": 2},  # elp
        ]

        self.drone = DroneAPI(waypoints=waypoints)

    def qr_callback(self, data):
        self.qr_data = data

    def elp_callback(self, data):
        # print("dx : {} dy : {} ".format(data.dx, data.dy))
        self.elp_data = data

    def land_algorithm(self):
        now = rospy.Time.now()
        x_done = False
        y_done = False

        while self.elp_data.is_found:
            cur_pose = {
                "x": self.drone.current_pose.pose.pose.position.x,
                "y": self.drone.current_pose.pose.pose.position.y,
                "z": self.drone.current_pose.pose.pose.position.z,
            }

            if self.elp_data.dx > 20:
                print("move right")
                cur_pose["x"] += 0.2
            elif self.elp_data.dx < -20:
                print("move left")
                cur_pose["x"] -= 0.2
            else:
                x_done = True
            if self.elp_data.dy > 20:
                print("move backward")
                cur_pose["y"] -= 0.2

            elif self.elp_data.dy < -20:
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
                            self.land_algorithm()
                            land = True
                        try:
                            self.drone.next()
                        except IndexError:
                            break
                    elif land:
                        self.land_algorithm()
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
        qr = ["", ""]
        while not rospy.is_shutdown():
            wpt_objective = ["read qr", "landing elp"]

            # self.drone = DroneAPI(waypoints=waypoints)
            self.drone.wait4start()
            self.drone.takeoff(2)
            try:
                self.drone.move()

                while not self.drone.check_waypoint_reached():
                    self.drone.move()
                    r.sleep()
                self.activate_qr(ActivateRequest(True))
                rospy.sleep(1)

                while not self.qr_data.is_active:
                    pass

                else:
                    qr[0] = self.qr_data.data
                    print("qr : ", qr[0])
                    self.activate_qr(ActivateRequest(False))
                    rospy.sleep(1)

                if qr[0] == "VTOL 1":
                    rospy.loginfo("[MISSION] VTOL 1 detected, drop red box")
                elif qr[0] == "VTOL 2":
                    rospy.loginfo("[MISSION] VTOL 2 detected, drop green box")
                elif qr[0] == "VTOL 3":
                    rospy.loginfo("[MISSION] VTOL 3 detected, drop blue box")

                self.drone.next()
                self.drone.move()
                while not self.drone.check_waypoint_reached():
                    self.drone.move()
                    r.sleep()

                rospy.loginfo("[MISSION] Dropping box")
                sleep(5)

                self.drone.next()
                self.drone.move()
                while not self.drone.check_waypoint_reached():
                    self.drone.move()
                    r.sleep()

                self.activate_qr(ActivateRequest(True))
                rospy.sleep(1)
                while not self.qr_data.is_active:
                    pass
                else:
                    qr[1] = self.qr_data.data
                    self.activate_qr(ActivateRequest(False))
                    rospy.sleep(1)

                print("qr : ", qr[1])
                if qr[1] == "VTOL 1":
                    rospy.loginfo("[MISSION] VTOL 1 detected, drop red box")
                elif qr[1] == "VTOL 2":
                    rospy.loginfo("[MISSION] VTOL 2 detected, drop green box")
                elif qr[1] == "VTOL 3":
                    rospy.loginfo("[MISSION] VTOL 3 detected, drop blue box")

                self.drone.next()
                self.drone.move()

                while not self.drone.check_waypoint_reached():
                    self.drone.move()
                    r.sleep()
                rospy.loginfo("[MISSION] Dropping box")
                sleep(5)

                self.drone.next()
                self.drone.move()

                while not self.drone.check_waypoint_reached():
                    self.drone.move()
                    r.sleep()

                if "VTOL 1" not in qr:
                    rospy.loginfo("[MISSION] dropping red box")
                elif "VTOL 2" not in qr:
                    rospy.loginfo("[MISSION] dropping green box")
                elif "VTOL 3" not in qr:
                    rospy.loginfo("[MISSION] dropping blue box")

                self.drone.next()
                self.drone.move()

                while not self.drone.check_waypoint_reached():
                    self.drone.move()
                    r.sleep()

                # landing elp
                self.activate_elp(ActivateRequest(True))
                rospy.sleep(1)
                while self.land_algorithm():
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
