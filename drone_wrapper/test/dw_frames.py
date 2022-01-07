#!/usr/bin/env python

from drone_wrapper import DroneWrapper
from geometry_msgs.msg import Twist, Pose, Point, Quaternion, Vector3
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import rospy
import time
from math import radians, cos

def are_poses_equal(p1, p2, pose_tol=0.001, orient_tol=0.001):
    pos1 = p1.position
    pos2 = p2.position
    if isclose(pos1.x, pos2.x, pose_tol) and isclose(pos1.y, pos2.y, pose_tol) and isclose(pos1.z, pos2.z, pose_tol):
        o1 = p1.orientation
        o2 = p2.orientation
        r1, p1, y1 = euler_from_quaternion([o1.x, o1.y, o1.z, o1.w])
        r2, p2, y2 = euler_from_quaternion([o2.x, o2.y, o2.z, o2.w])
        if isclose(cos(r1), cos(r2), orient_tol) and isclose(cos(p1), cos(p2), orient_tol) and isclose(cos(y1), cos(y2), orient_tol):
            return True
    return False


def are_twists_equal(t1, t2, tol=0.001):
    lin1 = t1.linear
    lin2 = t2.linear
    if isclose(lin1.x, lin2.x, tol) and isclose(lin1.y, lin2.y, tol) and isclose(lin1.z, lin2.z, tol):
        ang1 = t1.angular
        ang2 = t2.angular
    	if isclose(ang1.x, ang2.x, tol) and isclose(ang1.y, ang2.y, tol) and isclose(ang1.z, ang2.z, tol):
            return True
    return False


def isclose(n1, n2, tol=0.001):
    return abs(n1 - n2) <= tol


d = DroneWrapper()
time.sleep(4)

# ENU (EAST, NORTH and UP)  ===>  NED (NORTH, EAST, DOWN)
#     +Z     +Y 					   		+X
#      ^    ^						   	   ^
#      |  /					     	     /
#      |/							   /
#    world------> +X		===>	world-----> +Y
#									  |
#									  |
#									  v
#									 +Z

print("POSES")
pin = Pose(position=Point(x=1.0, y=2.0, z=3.0), orientation=Quaternion(0.0, 0.0, 0.0, 1.0))
pout = Pose(position=Point(x=2.0, y=1.0, z=-3.0), 
		    orientation=Quaternion(*quaternion_from_euler(radians(180), 
		    											  radians(0),
		    											  radians(90))))
p = d.frames_tf.transform("enu", "ned", pin)
print("ENU2NED", are_poses_equal(p.pose, pout))

p = d.frames_tf.transform("ned", "enu", p.pose)
print("NED2ENU", are_poses_equal(p.pose, pin))


# FLU (Forward, Left and Up)  ===>  FRD (Forward, Right, Down)
#           +Z     +X 					   		 +X
#            ^    ^						   	    ^
#            |  /					     	  /
#            |/							   	/
#  +Y <------body			  ===>		   body-----> +Y
#									  		|
#									  		|
#									  		v
#									 	   +Z

pin = Pose(position=Point(x=1.0, y=2.0, z=3.0), orientation=Quaternion(0.0, 0.0, 0.0, 1.0))
pout = Pose(position=Point(x=1.0, y=-2.0, z=-3.0), 
		    orientation=Quaternion(*quaternion_from_euler(radians(180), 
		    											  radians(0),
		    											  radians(0))))
p = d.frames_tf.transform("flu", "frd", pin)
print("FLU2FRD", are_poses_equal(p.pose, pout))

p = d.frames_tf.transform("frd", "flu", p.pose)
print("FRD2FLU", are_poses_equal(p.pose, pin))

# FLU (Forward, Left and Up)  ===>  ENU (EAST, NORTH, UP)
#           +Z     +X                      +Z     +Y
#            ^    ^                         ^    ^
#            |  /                           |  /
#            |/                             |/
#  +Y <------body             ===>        world-----> +X

pin = Pose(position=Point(x=1.0, y=2.0, z=3.0), orientation=Quaternion(0.0, 0.0, 0.0, 1.0))
my_pose = d.get_position(frame="map")
my_orient = d.get_orientation(frame="map")
pout = Pose(position=Point(x=2.0 + my_pose[0], y=-1.0 + my_pose[1], z=3.0 + my_pose[2]), 
            orientation=Quaternion(*quaternion_from_euler(my_orient[0] + radians(0), 
                                                          my_orient[1] + radians(0),
                                                          my_orient[2] + radians(-90))))
p = d.frames_tf.transform("flu", "enu", pin)
print("FLU2ENU", are_poses_equal(p.pose, pout, pose_tol=0.5))

p = d.frames_tf.transform("enu", "flu", p.pose)
print("ENU2FLU", are_poses_equal(p.pose, pin))


print("\nVELS")
vin = Twist(linear=Vector3(x=1.0, y=2.0, z=3.0), angular=Vector3(x=4.0, y=5.0, z=6.0))
vout = Twist(linear=Vector3(x=2.0, y=1.0, z=-3.0), angular=Vector3(x=5.0, y=4.0, z=-6.0))

v = d.frames_tf.transform("enu", "ned", vin)
print("ENU2NED", are_twists_equal(v.twist, vout))

v = d.frames_tf.transform("ned", "enu", v.twist)
print("NED2ENU", are_twists_equal(v.twist, vin))

# FLU --> FRD
vin = Twist(linear=Vector3(x=1.0, y=2.0, z=3.0), angular=Vector3(x=4.0, y=5.0, z=6.0))
vout = Twist(linear=Vector3(x=1.0, y=-2.0, z=-3.0), angular=Vector3(x=4.0, y=-5.0, z=-6.0))

v = d.frames_tf.transform("flu", "frd", vin)
print("FLU2FRD", are_twists_equal(v.twist, vout))

v = d.frames_tf.transform("frd", "flu", v.twist)
print("FRD2FLU", are_twists_equal(v.twist, vin))

# FLU --> ENU
vin = Twist(linear=Vector3(x=1.0, y=2.0, z=3.0), angular=Vector3(x=4.0, y=5.0, z=6.0))
vout = Twist(linear=Vector3(x=-2.0, y=1.0, z=3.0), angular=Vector3(x=-5.0, y=4.0, z=6.0))
v = d.frames_tf.transform("flu", "enu", vin)
print("FLU2ENU", are_twists_equal(v.twist, vout, tol=0.5))

v = d.frames_tf.transform("enu", "flu", v.twist)
print("ENU2FLU", are_twists_equal(v.twist, vin))


# DRONE
print("\nDRONE POSE")
print("ENU", d.get_position(frame="map"))
print("NED", d.get_position(frame="ned"))
print("FLU", d.get_position(frame="flu"))
print("FRD", d.get_position(frame="frd"))

# d.takeoff()
d.set_cmd_pos(2,2,1,0,"map")
d.set_cmd_pos(0,0,1,0,"flu")
# print(self.get_orientation(frame="map"))
# print(self.get_orientation(frame="ned"))
# print(self.get_velocity(frame="map"))
# print(self.get_velocity(frame="ned"))

rospy.spin()