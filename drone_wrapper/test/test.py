#! /usr/bin/env python

from drone_wrapper import DroneWrapper
import rospy
import time

d = DroneWrapper()
d.takeoff()

#d.set_cmd_pos(1,-2, 3, 0)
#time.sleep(3)
#print(d.get_position())

d.set_cmd_vel(vx=1, frame="enu")
print(d.get_velocity())

rospy.spin()