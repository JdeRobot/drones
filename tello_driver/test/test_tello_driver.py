#! /usr/bin/env python

import rospy
from mavros_msgs.srv import CommandTOL
from mavros_msgs.msg import PositionTarget
import time

ns_takeoff = 'mavros/cmd/takeoff'
ns_land = 'mavros/cmd/land'

rospy.init_node("test_tello_driver")

try:
    rospy.wait_for_service(ns_takeoff, timeout=5)
    takeoff_srv = rospy.ServiceProxy(ns_takeoff, CommandTOL)
    print("Takeoff service ready!")
except rospy.ServiceException as e:
    print(e)

try:
    rospy.wait_for_service(ns_land, timeout=5)
    land_srv = rospy.ServiceProxy(ns_land, CommandTOL)
    print("Land service ready!")
except rospy.ServiceException as e:
    print(e)

setpoint_raw_publisher = rospy.Publisher('mavros/setpoint_raw/local', PositionTarget, queue_size=1)

resp = takeoff_srv(0.0, 0.0, 0.0, 0.0, 0.0)
print("Taking off!", resp)

time.sleep(5)

# POSE
# msg = PositionTarget()
# msg.coordinate_frame = 8
# msg.type_mask = 3064  # xyz yaw
#
# msg.position.x = 0
# msg.position.y = 0
# msg.position.z = 0
# msg.yaw = 2
#
# print("Going to", msg.yaw_rate)
# setpoint_raw_publisher.publish(msg)
# time.sleep(10)

# VEL
msg = PositionTarget()
msg.coordinate_frame = 21
msg.type_mask = 1991  # vx vy vz yaw_rate

msg.position.x = 0
msg.position.y = 0
msg.position.z = 0
msg.velocity.y = 0.1
# msg.yaw_rate = 1

print("Going to", msg.yaw_rate)
setpoint_raw_publisher.publish(msg)
time.sleep(5)

msg = PositionTarget()
msg.coordinate_frame = 21
msg.type_mask = 1991  # vx vy vz yaw_rate

msg.position.x = 0
msg.position.y = 0
msg.position.z = 0
msg.velocity.y = -0.1

print("Going to", msg.yaw_rate)
setpoint_raw_publisher.publish(msg)
time.sleep(5)

resp = land_srv(0.0, 0.0, 0.0, 0.0, 0.0)
print("Landing!", resp)
