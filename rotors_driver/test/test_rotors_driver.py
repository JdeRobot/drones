#! /usr/bin/env python

import rospy
from mavros_msgs.srv import CommandTOL
from mavros_msgs.msg import PositionTarget
import time

from drone_wrapper import DroneWrapper
# setpoint_raw_publisher = rospy.Publisher('mavros/setpoint_raw/local', PositionTarget, queue_size=1)

d = DroneWrapper()
time.sleep(5.0)#for letting the rotors node initialise

# a  =d.get_frontal_image()
# print("pos:",a)
# time.sleep(3)

# time.sleep(2.0)
# d.set_cmd_pos(0.0,0.0,3.0,0.0)
# time.sleep(3.0)
# d.set_cmd_vel(0.3,0.3,0.0,0.0)
# time.sleep(3.5)
# d.set_cmd_mix(0.3,0.3,1.0,0.0)
# time.sleep(3.5)
# time.sleep(5)
while True:
    d.set_cmd_pos(0.0,0.0,3.0,0.0)
    time.sleep(0.5)
    # d.set_cmd_vel(0.3,0.3,0.0,0.0)
    # time.sleep(0.5)
    # d.set_cmd_mix(0.3,0.3,1.0,0.0)
    # time.sleep(0.5)
