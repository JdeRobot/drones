#!/usr/bin/env python

from drone_wrapper import DroneWrapper
import time

d = DroneWrapper()
d.takeoff()

d.set_cmd_pos(0, 0, 3, max_vel=6.0)

while True:
	print(d.get_velocity())
	time.sleep(0.2)