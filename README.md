## KRTI 2022

This is a catkin package built by VTOL team for KRTI 2022. Wish us luck.

### Cloning the repository

To clone the repository, make sure you already have created your own catkin workspace. If you already done that, clone this repository to your `src` folder inside the catkin workspace. Don't forget to run `catkin build` or `catkin_make` to build the package.

### Telling Gazebo where to look for KRTI 2022 models 

Run the following command:

```bash
echo "GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:$HOME/catkin_ws/src/krti2022/models" >> ~/.bashrc
```

If you put the catkin workspace somewhere else, modify the `/catkin_ws/` to the appropriate path.

### Launching the world

To start using this package, there are some launch file provided out of the box.
1. `roslaunch krti2022 full.launch` to launch both Gazebo and MAVROS
2. `roslaunch krti2022 gazebo_only.launch` to launch Gazebo only
3. `roslaunch krti2022 mavros_only.launch` to launch MAVROS only

### Disabling GPS and enabling optical flow (for indoors flight)

To disable GPS and enable optical flow, you must enter a few parameters to ArduPilot.

First, you need to start your SITL and MAVROS. After that, in the SITL, copy and paste these parameters

```
# 1. To add rangefinder (needed by optical flow)
param set SIM_SONAR_SCALE 10
param set RNGFND1_TYPE 1
param set RNGFND1_SCALING 10
param set RNGFND1_PIN 0
param set RNGFND1_MAX_CM 5000
param set RNGFND1_MIN_CM 0

# 2. To add optical flow
param set SIM_FLOW_ENABLE 1
param set FLOW_TYPE 10

# 3. Rely on optical flow and disable GPS
param set GPS_TYPE 0
param set EK3_ENABLE 1
param set AHRS_EKF_TYPE 3
param set EK3_SRC_OPTIONS 0
param set EK3_FLOW_DELAY 10
param set EK3_SRC1_POSXY 0
param set EK3_SRC1_VELXY 5
param set EK3_SRC1_POSZ 1
param set EK3_SRC1_VELZ 0
param set EK3_SRC1_YAW 1

# 4. To allow arming and takeoff without GPS
param set ARMING_CHECK 0
```

When you've done that, restart your SITL (by using CTRL + C and start the `startsitl.sh` again). In your next flight you should see that you don't have GPS anymore.

### License

See [LICENSE](LICENSE.md).
