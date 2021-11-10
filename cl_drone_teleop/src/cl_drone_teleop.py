#!/usr/bin/env python

import rospy
import time
from drone_wrapper import DroneWrapper

RATE = 0.1
LIN_VEL = 0.1
ANG_VEL = 0.1
msg = """
    Teleoperate Target!
---------------------------
Take Off:
    tk
Land:
    la
Moving around:
        w                i
   a    s    d      j    k    l
        x                ,
w/x : increase/decrease linear Z velocity (~ 0.1)
a/d : increase/decrease angular velocity (~ 0.1)
i/, : increase/decrease linear X velocity (~ 0.1)
j/l : increase/decrease linear Y velocity (~ 0.1)
space key, s, k : force stop
CTRL-D to quit
"""


def print_status(vx, vy, vz, az):
    print("\n\t   STATUS")
    print("---------------------------")
    print("%s\t%s\t%s\t%s\n" % (str(vx), str(vy), str(vz), str(az)))


if __name__ == "__main__":
    drone = DroneWrapper()
    vx, vy, vz, az = 0, 0, 0, 0

    while not rospy.is_shutdown():
        try:
            cmd = raw_input(msg).lower()
            if cmd == "tk":
                drone.takeoff()
                continue
            elif cmd == "la":
                drone.land()
                continue
            elif cmd == "w":
                vz += LIN_VEL
            elif cmd == "a":
                az += ANG_VEL
            elif cmd == "d":
                az -= ANG_VEL
            elif cmd == "x":
                vz -= LIN_VEL
            elif cmd == "i":
                vx += LIN_VEL
            elif cmd == "j":
                vy -= LIN_VEL
            elif cmd == "l":
                vy += LIN_VEL
            elif cmd == ",":
                vx -= LIN_VEL
            elif cmd == "s" or "k" or " ":
                vx, vy, vz, az = 0, 0, 0, 0
            elif cmd == "00":
                vx, vy, vz, az = 0, 0, 0, 0
            else:
                print("[Error] Invalid command")
        except EOFError:
            break
        else:
            drone.set_cmd_vel(vx, vy, vz, az)
            print_status(vx, vy, vz, az)

        rospy.sleep(RATE)

    rospy.loginfo("BYE")
    rospy.sleep(1)