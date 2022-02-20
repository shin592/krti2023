#!/usr/bin/env python

import rospy
import mavros

from geometry_msgs.msg import PoseStamped, Twist
from mavros_msgs.msg import State #, Thrust, AttitudeTarget
from mavros_msgs.srv import CommandBool, SetMode
from pymavlink import mavutil # Needed for command message definitions

mavros.set_namespace()

# callback method for state sub
current_state = State() 
offb_set_mode = SetMode()

cmd_vel = Twist()
cmd_vel.angular.x = 0.0
cmd_vel.angular.y = 0.0
cmd_vel.angular.z = 0.0
cmd_vel.linear.x = 0.0
cmd_vel.linear.y = 0.0
cmd_vel.linear.z = 0.0
current_pose = PoseStamped()

def state_cb(state):
    global current_state
    current_state = state

def pose_callback(pose_cb):
    global current_pose
    current_pose = pose_cb

# local_pos_pub = rospy.Publisher(mavros.get_topic('setpoint_position/local'), PoseStamped, queue_size=10)
cmd_vel_pub = rospy.Publisher(mavros.get_topic('setpoint_velocity/cmd_vel_unstamped'), Twist, queue_size=10)
state_sub = rospy.Subscriber(mavros.get_topic('state'), State, state_cb)
arming_client = rospy.ServiceProxy(mavros.get_topic('cmd/arming'), CommandBool)
set_mode_client = rospy.ServiceProxy(mavros.get_topic('set_mode'), SetMode) 
pose_sub = rospy.Subscriber(mavros.get_topic('setpoint_position/local'), PoseStamped, pose_callback)
# thrust_pub = rospy.Publisher(mavros.get_topic('setpoint_attitude/thrust'), Thrust, queue_size=10)



def main():
    global current_pose, thrust, pose
    rospy.init_node('guided', anonymous=True)
    prev_state = current_state
    rate = rospy.Rate(20.0) # MUST be more then 2Hz

    # wait for FCU connection
    while not current_state.connected and not rospy.is_shutdown():
        rate.sleep()

    # send a few setpoints before starting

    for i in range(100):
        print("%d / 100" % (i+1))
        # local_pos_pub.publish(pose)
        cmd_vel_pub.publish(cmd_vel)
        rate.sleep()
    
    takeoff = False
    land = False

    last_request = rospy.get_rostime()
    while not rospy.is_shutdown():
        now = rospy.get_rostime()
        if(not takeoff):
            if current_state.mode != "GUIDED" and (now - last_request > rospy.Duration(5.)):
                set_mode_client(base_mode=0, custom_mode="GUIDED")
                last_request = now 
            else:
                if not current_state.armed and (now - last_request > rospy.Duration(5.)):
                    arming_client(True)
                    rospy.loginfo("arming vehicle")
                    last_request = now 

            # older versions of PX4 always return success==True, so better to check Status instead
            if prev_state.armed != current_state.armed:
                rospy.loginfo("Vehicle armed: %r" % current_state.armed)
            if prev_state.mode != current_state.mode: 
                rospy.loginfo("Current mode: %s" % current_state.mode)
            prev_state = current_state
                
            if current_state.armed and current_state.mode == "GUIDED":
                rospy.loginfo("taking off")
                takeoff_alt = 3
                cmd_vel.linear.z = 5
                cmd_vel.linear.x = 0
                cmd_vel.linear.y = 0
                cmd_vel_pub.publish(cmd_vel)
                rospy.loginfo("target_alt = %f current alt = %f" % (takeoff_alt, current_pose.pose.position.z))

                if(current_pose.pose.position.z > takeoff_alt*0.85):
                    cmd_vel_pub.publish(cmd_vel)
                    takeoff = True
                    takeoff_complete_time = rospy.get_rostime()
                    rospy.loginfo("takeoff complete")

                
        elif(not land):
            now = rospy.get_rostime()
            # pose.pose.position.x = 4.0
            cmd_vel.linear.x = 4
            cmd_vel_pub.publish(cmd_vel)
            if now - takeoff_complete_time > rospy.Duration(5.0):
                cmd_vel.linear.x = 0.0
                cmd_vel_pub.publish(cmd_vel)
                rospy.loginfo("return to home")
                set_mode_client(base_mode=0, custom_mode="RTL")
                land = True
        else:
            if current_state.mode != "RTL":
                set_mode_client(base_mode=0, custom_mode="RTL")

        # Update timestamp and publish pose 
        pose = current_pose
        pose.header.stamp = rospy.Time.now()
        cmd_vel_pub.publish(cmd_vel)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass