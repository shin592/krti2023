from math import radians, cos, sin
import rospy
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from geographic_msgs.msg import GeoPointStamped
from mavros_msgs.msg import State, Thrust
from mavros_msgs.srv import SetMode, SetModeRequest
from mavros_msgs.srv import ParamSet, ParamSetRequest
from mavros_msgs.srv import CommandBool, CommandBoolRequest
from mavros_msgs.srv import CommandTOL, CommandTOLRequest


class DroneAPI:
    """
    Control Functions
    This module is designed to make high level control programming simple.
    """

    """
        TODO:
            1. Emergency cancel (stop all movement) if LIDAR detects an obstacle
               within 1 meter (http://wiki.ros.org/mavros#mavros.2FPlugins.local_position)
            2. Set velocity (setpoint_velocity)
    """

    def __init__(
        self,
        waypoints: list = [],
        global_position: dict = {
            "latitude": -7.265572783693384,
            "longitude": 112.78452265474213,
            "altitude": 0.0,
        },
        parameters: dict = {},
    ) -> None:
        """
        Initialize the drone API

        Args:
            waypoints (list): list of waypoints
            global_position (dict): global position
            parameters (dict): parameters to set
        """

        # Set waypoints
        self.current_waypoint = 0
        self.follow_waypoint = True
        self.waypoints = waypoints

        # Set origin
        self.set_origin(global_position)

        # Set parameters
        for name, value in parameters.items():
            self.set_parameter(name, value)

        # Set state
        self.current_state = State()
        state_sub = rospy.Subscriber("/mavros/state", State, self.state_cb)

        # Wait for connection
        self.wait4connect()

        # Print success
        rospy.loginfo("Initialization completed.")

    def state_cb(self, msg):
        """
        A function for state's subscriber callback
        Will set self.current_state to the message received
        """

        self.current_state = msg

    def set_origin(self, origin: dict):
        """
        A function to set origin to custom coordinates
        We need to set this if we're flying without GPS

        Args:
            origin (dict): origin coordinates
                - latitude
                - longitude
                - altitude

        Refer to https://discuss.ardupilot.org/t/guided-mode-with-optical-flow-without-gps-in-simulation/53494/6
        """

        # Get client
        setter = rospy.Publisher(
            "/mavros/global_position/set_gp_origin", GeoPointStamped, queue_size=10
        )

        # Set position
        position = GeoPointStamped()
        position.header.frame_id = "global"
        position.header.stamp = rospy.Time.now()
        position.position.latitude = origin["latitude"]
        position.position.longitude = origin["longitude"]
        position.position.altitude = origin["altitude"]

        setter.publish(position)

    def set_parameter(self, name: str, value: float):
        """
        A function to set parameters

        Args:
            name (str): name of parameter
            value (float): value of parameter
        """

        # Get client
        client = rospy.ServiceProxy(name="/mavros/param/set", service_class=ParamSet)

        # Set parameter
        request = ParamSetRequest()
        request.param_id = name
        request.value.real = value

        # Send request
        client(request)

    def set_mode(self, mode: str = "GUIDED"):
        """
        A function to set mode

        Args:
            mode (str): mode to set. Default to GUIDED
        """

        # Get client
        rospy.wait_for_service("/mavros/set_mode")
        client = rospy.ServiceProxy(name="/mavros/set_mode", service_class=SetMode)

        # Set mode
        client(SetModeRequest(0, mode))

        # Check if mode is set
        if self.current_state.mode == mode:
            # Print success message
            rospy.loginfo(f"Mode is set to {mode}")
        else:
            # Print failed message
            rospy.loginfo(f"Failed to set mode to {mode}")
            rospy.loginfo(f"Current mode is {self.current_state.mode}")

    def set_thrust(self, thrust: float):
        """
        A function to set thrust

        Args:
            thrust (float): thrust to set. Value should be between 0 and 1
        """

        # Check if thrust is not in range
        if thrust < 0 or thrust > 1:
            print("Illegal thrust value. It should be between 0 and 1 (inclusive).")
            return

        # Get client
        client = rospy.Publisher(
            name="/mavros/setpoint_attitude/thrust", service_class=Thrust, queue_size=10
        )

        # Create thrust message
        request = Thrust()
        request.header.stamp = rospy.Time.now()
        request.thrust = thrust

        # Set thrust
        client.publish(request)

    def arm(self, status: bool = True):
        """
        A function to arm or disarm the drone

        Args:
            status (bool): True to arm, False to disarm
        """

        # Send 100 pose
        for _ in range(100):
            self.move({"x": 0, "y": 0, "z": 0})
            rospy.sleep(0.01)

        # Get client
        rospy.wait_for_service("/mavros/cmd/arming")
        arming_client = rospy.ServiceProxy(
            name="/mavros/cmd/arming", service_class=CommandBool
        )

        # Arm
        while not rospy.is_shutdown() and not self.current_state.armed:
            arming_client(CommandBoolRequest(status))
        else:
            if status == True:
                rospy.loginfo("Drone is armed and ready to fly")
            else:
                rospy.loginfo("Drone is disarmed")

    def takeoff(self, altitude: float = 3.0):
        """
        A function to give drone a takeoff command

        Args:
            altitude (float): altitude to takeoff to
        """

        # Arm drone
        self.arm()

        # Get client
        rospy.wait_for_service("/mavros/cmd/takeoff")
        takeoff_client = rospy.ServiceProxy(
            name="/mavros/cmd/takeoff", service_class=CommandTOL
        )

        # Takeoff
        takeoff_client(CommandTOLRequest(0, 0, 0, 0, altitude))

        # Print success message
        rospy.loginfo("Taking off ...")

    def land(self):
        """
        A function to give drone a land command
        """

        # Get client
        rospy.wait_for_service("/mavros/cmd/land")
        client = rospy.ServiceProxy(name="/mavros/cmd/land", service_class=CommandTOL)

        # Landing
        client(CommandTOLRequest(0, 0, 0, 0, 0))

        # Print success message
        rospy.loginfo(
            "Landing command sent. Drone should be disarming itself in 10-15 seconds after it touches the ground. ..."
        )

    def move(self, destination: dict = None):
        """
        A function to move the drone to certain position

        Args:
            destination (dict | None): destination coordinates
                - x
                - y
                - z
                - heading (optional, if not passed then it will use the current heading)

                If no destination passed then drone will go to
                current destination in the waypoint list
        """

        # Get client
        client = rospy.Publisher(
            name="/mavros/setpoint_position/local",
            service_class=PoseStamped,
            queue_size=10,
        )

        # If no destination is given, use current destination
        # indicated by current waypoint index
        if destination == None:
            destination = self.waypoints[self.current_waypoint]

        # Set position
        request = PoseStamped()
        request.header.stamp = rospy.Time.now()
        request.pose.position = Point(
            x=destination["x"], y=destination["y"], z=destination["z"]
        )

        # If have heading then we set the heading
        if "heading" in destination:
            request.pose.orientation = self.calculate_heading(destination["heading"])

        # Send request
        client.publish(request)

        # Print success message
        rospy.loginfo(
            f"Moving to x: {destination['x']}; y: {destination['y']}; z: {destination['z']}"
        )

    def next(self):
        """
        A function to move to next waypoint
        """
        self.current_waypoint += 1

    def wait4connect(self):
        """
        Wait for connect is a function that will hold the program until communication with the FCU is established.
        Returns:
                0 (int): Connected to FCU.
                -1 (int): Failed to connect to FCU.
        """
        rospy.loginfo("Waiting for FCU connection")
        while not rospy.is_shutdown() and not self.current_state_g.connected:
            rospy.sleep(0.01)
        else:
            if self.current_state_g.connected:
                rospy.loginfo("FCU connected")
                return 0
            else:
                rospy.logerr("Error connecting to drone's FCU")
                return -1

    def wait4start(self):
        """
        This function will hold the program until the user signals the FCU to mode enter GUIDED mode. This is typically done from a switch on the safety pilot's remote or from the Ground Control Station.
        Returns:
                0 (int): Mission started successfully.
                -1 (int): Failed to start mission.
        """
        rospy.loginfo("Waiting for user to set mode to GUIDED")
        while not rospy.is_shutdown() and self.current_state_g.mode != "GUIDED":
            rospy.sleep(0.01)
        else:
            if self.current_state_g.mode == "GUIDED":
                rospy.loginfo("Mode set to GUIDED. Starting Mission...")
                return 0
            else:
                rospy.logerr("Error startting mission")
                return -1

    def calculate_heading(self, heading) -> Quaternion:
        """
        This function is used to specify the drone's heading in the local reference frame. Psi is a counter clockwise rotation following the drone's reference frame defined by the x axis through the right side of the drone with the y axis through the front of the drone.
        Args:
                heading (Float): θ(degree) Heading angle of the drone.
        """
        yaw = radians(heading)
        pitch = 0.0
        roll = 0.0

        cy = cos(yaw * 0.5)
        sy = sin(yaw * 0.5)

        cr = cos(roll * 0.5)
        sr = sin(roll * 0.5)

        cp = cos(pitch * 0.5)
        sp = sin(pitch * 0.5)

        qw = cy * cr * cp + sy * sr * sp
        qx = cy * sr * cp - sy * cr * sp
        qy = cy * cr * sp + sy * sr * cp
        qz = sy * cr * cp - cy * sr * sp

        return Quaternion(qx, qy, qz, qw)
