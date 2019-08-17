#!/usr/bin/env python

import rospy
import tf
import numpy as np
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import NavSatFix, Image
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State, PositionTarget
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest, CommandTOL, CommandTOLRequest

class DroneWrapper():
	def state_cb(self, msg):
		self.state = msg
		rospy.logdebug('State updated')

	def pose_stamped_cb(self, msg):
		self.pose_stamped = msg
		rospy.logdebug('Pose updated')
	
	def global_position_cb(self, msg):
		self.global_position = msg
		rospy.logdebug('Global position updated')
	
	def cam_frontal_cb(self, msg):
		self.frontal_image = msg
		rospy.logdebug('Frontal image updated')
	
	def cam_ventral_cb(self, msg):
		self.ventral_image = msg
		rospy.logdebug('Ventral image updated')
	
	def stay_armed_stay_offboard_cb(self, event):
		if self.state.mode != 'OFFBOARD':
			if self.request_mode('OFFBOARD'):
				rospy.loginfo("OFFBOARD requested")
		elif not self.state.armed:
			if self.arm(True):
				rospy.loginfo("Vehicle Armed")
			
	def get_frontal_image(self):
		return self.bridge.imgmsg_to_cv2(self.frontal_image)
	
	def get_ventral_image(self):
		return self.bridge.imgmsg_to_cv2(self.ventral_image)
	
	def get_position(self):
		return np.array([self.pose_stamped.pose.position.x, self.pose_stamped.pose.position.y, self.pose_stamped.pose.position.z])
	
	def get_orientation(self):
		return np.array(tf.transformations.euler_from_quaternion([self.pose_stamped.pose.orientation.x, self.pose_stamped.pose.orientation.y, self.pose_stamped.pose.orientation.z, self.pose_stamped.pose.orientation.w]))

	def get_roll(self):
		return self.get_orientation()[0]
	
	def get_pitch(self):
		return self.get_orientation()[1]
	
	def get_yaw(self):
		return self.get_orientation()[2]

	def arm(self, value = True):
		req = CommandBoolRequest()
		req.value = value
		if self.arm_client(req).success:
			rospy.loginfo('Arming/Disarming successful')
			return True
		else:
			rospy.logwarn('Arming/Disarming unsuccessful')
			return False
		
	def request_mode(self, mode = 'OFFBOARD'):
		rospy.sleep(2)
		rospy.loginfo('Current mode: %s', self.state.mode)
		req = SetModeRequest()
		req.custom_mode = mode
		if self.mode_client(req).mode_sent:
			rospy.loginfo('Mode change request successful')
			return True
		else:
			rospy.logwarn('Mode change request unsuccessful')
			return False

	def set_cmd_vel(self, vx = 0, vy = 0, vz = 0, az = 0):
		self.setpoint_raw.type_mask = int('0b010111000011', 2)
		self.setpoint_raw.coordinate_frame = 8
		self.setpoint_raw.velocity.x = -vy
		self.setpoint_raw.velocity.y = vx
		self.setpoint_raw.velocity.z = vz

		self.setpoint_raw.position.z = self.pose_stamped.pose.position.z + vz * self.vz_factor
		
		self.setpoint_raw.yaw_rate = az
		self.setpoint_raw_publisher.publish(self.setpoint_raw)

	def repeat_setpoint_raw(self, event):
		self.setpoint_raw.position.z = self.pose_stamped.pose.position.z + self.setpoint_raw.velocity.z * self.vz_factor
		self.setpoint_raw_publisher.publish(self.setpoint_raw)

	def hold_setpoint_raw(self):
		if not self.setpoint_raw_flag:
			self.setpoint_raw_timer = rospy.Timer(rospy.Duration(nsecs=50000000), self.repeat_setpoint_raw)

	def takeoff(self, uptime = 4):
		self.set_cmd_vel(0, 0, 0, 0)
		self.hold_setpoint_raw()
		self.arm(True)
		self.stay_armed_stay_offboard_timer = rospy.Timer(rospy.Duration(3), self.stay_armed_stay_offboard_cb)
		while True:
			while not (self.state.armed and self.state.mode == 'OFFBOARD'):
				self.rate.sleep()
			rospy.loginfo('Sleeping 3 secs to confirm change')
			rospy.sleep(3)
			if self.state.mode == 'OFFBOARD':
				break
		self.set_cmd_vel(vz = 3)
		rospy.loginfo('Taking off!!!')
		rospy.sleep(uptime)
		self.set_cmd_vel()

	def take_control(self):
		self.set_cmd_vel(0, 0, 0, 0)
		self.hold_setpoint_raw()
		self.arm(True)
		self.stay_armed_stay_offboard_timer = rospy.Timer(rospy.Duration(3), self.stay_armed_stay_offboard_cb)	
		
	def land(self):
		self.setpoint_raw_timer.shutdown()
		self.stay_armed_stay_offboard_timer.shutdown()
		req = CommandTOLRequest()
		req.latitude = self.global_position.latitude
		req.longitude = self.global_position.longitude
		self.land_client(req)

	def __init__(self, name = 'drone', verbose = False):
		if verbose:
			rospy.init_node(name, anonymous = True, log_level = rospy.DEBUG)
		else:
			rospy.init_node(name)
		
		self.state = State()
		self.pose_stamped = PoseStamped()
		self.rate = rospy.Rate(20)
		self.setpoint_raw = PositionTarget()
		self.setpoint_raw_flag = False
		self.vz_factor = 0.4
		self.bridge = CvBridge()
		
		self.setpoint_raw_timer = rospy.Timer(rospy.Duration(nsecs=50000000), self.repeat_setpoint_raw)
		self.setpoint_raw_timer.shutdown()
		self.stay_armed_stay_offboard_timer = rospy.Timer(rospy.Duration(5), self.stay_armed_stay_offboard_cb)
		self.stay_armed_stay_offboard_timer.shutdown()

		rospy.wait_for_service('mavros/cmd/arming')
		self.arm_client = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
		rospy.wait_for_service('mavros/set_mode')
		self.mode_client = rospy.ServiceProxy('mavros/set_mode', SetMode)
		rospy.wait_for_service('mavros/cmd/land')
		self.land_client = rospy.ServiceProxy('mavros/cmd/land', CommandTOL)
		
		rospy.Subscriber('mavros/state', State, self.state_cb)
		rospy.Subscriber('mavros/local_position/pose', PoseStamped, self.pose_stamped_cb)
		rospy.Subscriber('mavros/global_position/global', NavSatFix, self.global_position_cb)
		cam_frontal_topic = rospy.get_param('cam_frontal_topic', '/iris/cam_frontal/image_raw')
		cam_ventral_topic = rospy.get_param('cam_ventral_topic', '/iris/cam_ventral/image_raw')
		rospy.Subscriber(cam_frontal_topic, Image, self.cam_frontal_cb)
		rospy.Subscriber(cam_ventral_topic, Image, self.cam_ventral_cb)

		self.setpoint_raw_publisher = rospy.Publisher('mavros/setpoint_raw/local', PositionTarget, queue_size = 1)