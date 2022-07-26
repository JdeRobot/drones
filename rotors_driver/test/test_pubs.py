#! /usr/bin/env python

import os
import rospy
from mavros_msgs.msg import State, ExtendedState
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import BatteryState, NavSatFix

ns_bat = 'mavros/battery'
ns_state = 'mavros/state'
ns_ext_state = 'mavros/extended_state'
ns_pose = 'mavros/local_position/pose'
ns_vel = 'mavros/local_position/velocity_body'
ns_global = 'mavros/global_position/global'

RATE = 10
CONT = 0
TEMP = "{}\n" \
       "\n" \
       "###########################\n" \
       "#      TELLO STATUS       #\n" \
       "#                         #\n" \
       "#  MODE: {}       #\n" \
       "#  STATE: {}       #\n" \
       "#  BAT:   {:02d} %            #\n" \
       "#  Z:     {} m            #\n" \
       "#                         #\n" \
       "###########################\n"

bat_percent = 0
state = State()
ext_state = 0
pose = PoseStamped()
h = 0
vel = TwistStamped()
glob = NavSatFix()

ext_state_codes = {0: "UNDEFINED", 1: "ON_GROUND", 2: "IN_AIR   ", 3: "TAKEOFF  ", 4: "LANDING  "}


def battery_cb(msg):
    global bat_percent
    bat_percent = int(msg.percentage)


def state_cb(msg):
    global state
    state = msg


def ext_state_cb(msg):
    global ext_state
    ext_state = int(msg.landed_state)


def pose_cb(msg):
    global pose, h
    pose = msg
    h = msg.pose.position.z


def vel_cb(msg):
    global vel
    vel = msg


def global_cb(msg):
    global glob
    glob = msg


def print_status():
    os.system('clear')  # Elegant-less
    print(TEMP.format(CONT, state.mode, ext_state_codes[ext_state], bat_percent, h), end='\r')


def main():
    rospy.init_node("test_pubs")
    rospy.Subscriber(ns_bat, BatteryState, battery_cb)
    rospy.Subscriber(ns_state, State, state_cb)
    rospy.Subscriber(ns_ext_state, ExtendedState, ext_state_cb)
    rospy.Subscriber(ns_pose, PoseStamped, pose_cb)
    rospy.Subscriber(ns_vel, TwistStamped, vel_cb)
    rospy.Subscriber(ns_global, NavSatFix, global_cb)
    # rospy.spin()

    rate = rospy.Rate(RATE)  # 10hz

    while not rospy.is_shutdown():
        print_status()
        global CONT
        CONT += 1
        rate.sleep()  # sleeps (10Hz)


if __name__ == "__main__":
    main()
