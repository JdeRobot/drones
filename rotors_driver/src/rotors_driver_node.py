#!/usr/bin/env python

import rospy
import tf
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from geometry_msgs.msg import Point,Transform, Quaternion, Twist ,PoseStamped, TwistStamped, Pose, Vector3
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix, Image, BatteryState, NavSatStatus
from mavros_msgs.msg import State, ExtendedState, PositionTarget, ParamValue
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL, \
    ParamSet, ParamGet
import tf
import time

class RotorsConstraints:
    MIN_SPEED = 10
    MAX_SPEED = 100

    def constraint_speed(self, s):
        '''
        s: speed
        '''
        return self.MIN_SPEED if s < self.MIN_SPEED else self.MAX_SPEED if s > self.MAX_SPEED else s

class Rotors_Driver():
	CONSTRAINTS = RotorsConstraints()

	def __init__(self):
		self.sample_time = 1.0
		self.mav_name = "firefly"
		self.__is_armed = False
		self.__is_flying = False
		self.__state_dict = {}
		self.current_state = Odometry()
		self.current_x=self.current_state.pose.pose.position.x
		self.current_y=self.current_state.pose.pose.position.y
		self.current_z=self.current_state.pose.pose.position.z
		self.current_vx=self.current_state.twist.twist.linear.x
		self.current_vy=self.current_state.twist.twist.linear.y
		self.current_vz=self.current_state.twist.twist.linear.z
		self.current_ang_vx=self.current_state.twist.twist.angular.x
		self.current_ang_vy=self.current_state.twist.twist.angular.y
		self.current_ang_vz=self.current_state.twist.twist.angular.z
		self.quaternion = [self.current_state.pose.pose.orientation.x, self.current_state.pose.pose.orientation.y,
					self.current_state.pose.pose.orientation.z, self.current_state.pose.pose.orientation.w
				]
		self.current_roll, self.current_pitch, self.current_yaw = tf.transformations.euler_from_quaternion(self.quaternion)
		self.current_yaw_rate = self.current_state.twist.twist.angular.z

		self.frame_id = self.current_state.header.frame_id
		
		self.arm_client = rospy.Service('mavros/cmd/arming', CommandBool, self.rotors_arm)
		self.mode_client = rospy.Service('mavros/set_mode', SetMode, self.rotors_set_mode)
		self.land_client = rospy.Service('mavros/cmd/land', CommandTOL, self.rotors_land)
		self.set_param = rospy.Service('mavros/param/set', ParamSet, self.rotors_param_set)
		self.get_param = rospy.Service('mavros/param/get', ParamGet, self.rotors_param_get)
		
		self.rqt_extended_state_publisher = rospy.Publisher('mavros/extended_state', ExtendedState,
                                                            queue_size=1)
		self.rqt_state_publisher = rospy.Publisher('mavros/state', State,
                                                            queue_size=1)
		self.battery_state_publisher = rospy.Publisher('mavros/battery', BatteryState, queue_size=1)
		self.rqt_pose_publisher = rospy.Publisher( 'mavros/local_position/pose', PoseStamped,
                                                  queue_size=1)
		self.rqt_global_position_publisher = rospy.Publisher( 'mavros/global_position/global', NavSatFix,
                                                  queue_size=1)
		self.rqt_velocity_body_publisher = rospy.Publisher('mavros/local_position/velocity_body',
                                                           TwistStamped, queue_size=1)
		self.rqt_cam_frontal_publisher = rospy.Publisher('/' + self.mav_name + '/cam_frontal/image_raw', Image,
                                                         queue_size=1)
		self.rqt_cam_ventral_publisher = rospy.Publisher('/' + self.mav_name + '/cam_ventral/image_raw', Image,
                                                         queue_size=1)


		
		
		
		rospy.Subscriber("mavros/setpoint_raw/local", PositionTarget, self.publish_position_desired)
		

		rospy.Subscriber("/firefly/ground_truth/odometry", Odometry, self.get_pose)
		rospy.Subscriber("/firefly/frontal_cam/camera_nadir/image_raw", Image, self.cam_frontal)
		rospy.Subscriber("/firefly/ventral_cam/camera_nadir/image_raw", Image, self.cam_ventral)
		self.firefly_command_publisher = rospy.Publisher('/firefly/command/trajectory', MultiDOFJointTrajectory, queue_size=10)
		time.sleep(1.0)
		rospy.Timer(rospy.Duration(1.0/10.0), self.timer_callback)
		
	def timer_callback(self, event=None):
		landed_state = 2 if self.__is_flying else 1
		ext_state = ExtendedState(vtol_state=0, landed_state=landed_state)
		self.rqt_extended_state_publisher.publish(ext_state)


		state = State()
		self.rqt_state_publisher.publish(state)
		rospy.logdebug('State updated')


		try:
			bat_percent = int(self.__state_dict["bat"])
		except KeyError:
			bat_percent = float('nan')
			# rospy.logwarn("Battery state unknown.")
		bat = BatteryState(voltage=0.0, current=float('nan'), charge=float('nan'),
                           capacity=float('nan'), design_capacity=float('nan'), percentage=bat_percent,
                           power_supply_status=0, power_supply_health=0, power_supply_technology=0, present=True,
                           cell_voltage=[float('nan')], location="0",
                           serial_number="")
		self.battery_state_publisher.publish(bat)


		pose = PoseStamped(pose=Pose(position=Point(x=self.current_x, y=self.current_y, z=self.current_z),
                                     orientation=Quaternion(x=float(self.quaternion[0]), y=float(self.quaternion[1]), z=float(self.quaternion[2]),
                                                            w=float(self.quaternion[3]))),header = Header(frame_id = self.frame_id))
		self.rqt_pose_publisher.publish(pose)


		# Empty, global pos not known
		nav_sat = NavSatFix(status=NavSatStatus(status=-1, service=0), latitude=float('nan'),
                            longitude=float('nan'), altitude=float('nan'), position_covariance=[float('nan')] * 9,
                            position_covariance_type=0)
		self.rqt_global_position_publisher.publish(nav_sat)


		twist = TwistStamped(twist=Twist(linear=Vector3(x=self.current_vx, y=self.current_vy, z=self.current_vz),
                                         angular=Vector3(x=self.current_ang_vx, y=self.current_ang_vy, z=self.current_ang_vz)),
										 header = Header(frame_id = self.frame_id))
		self.rqt_velocity_body_publisher.publish(twist)


		self.rqt_cam_frontal_publisher.publish(self.frontal_image)
		self.rqt_cam_ventral_publisher.publish(self.ventral_image)


	def cam_frontal(self, msg):
		self.frontal_image = msg
		rospy.logdebug('Frontal image updated')

	def cam_ventral(self, msg):
		self.ventral_image = msg
		rospy.logdebug('Ventral image updated')

	def get_pose(self,msg):

		self.current_state = msg
		
		self.current_x=self.current_state.pose.pose.position.x
		self.current_y=self.current_state.pose.pose.position.y
		self.current_z=self.current_state.pose.pose.position.z
		self.current_vx=self.current_state.twist.twist.linear.x
		self.current_vy=self.current_state.twist.twist.linear.y
		self.current_vz=self.current_state.twist.twist.linear.z
		self.current_ang_vx=self.current_state.twist.twist.angular.x
		self.current_ang_vy=self.current_state.twist.twist.angular.y
		self.current_ang_vz=self.current_state.twist.twist.angular.z
		self.quaternion = [self.current_state.pose.pose.orientation.x, self.current_state.pose.pose.orientation.y,
					self.current_state.pose.pose.orientation.z, self.current_state.pose.pose.orientation.w
				]
		self.current_roll, self.current_pitch, self.current_yaw = tf.transformations.euler_from_quaternion(self.quaternion)
		self.current_yaw_rate = self.current_state.twist.twist.angular.z

		self.frame_id = self.current_state.header.frame_id

	def rotors_takeoff_land(self,req):
		quaternion = tf.transformations.quaternion_from_euler(0, 0, 0.0)
		traj = MultiDOFJointTrajectory()
		
		header = Header()
		header.stamp = rospy.Time()
		header.frame_id = 'frame'
		traj.joint_names.append('base_link')
		traj.header=header
		
		if req:
			transforms =Transform(translation=Point(0.0, 0.0, 1.0), rotation=Quaternion(quaternion[0],quaternion[1],quaternion[2],quaternion[3]))
		else:
			transforms =Transform(translation=Point(0.0, 0.0, 0.0), rotation=Quaternion(quaternion[0],quaternion[1],quaternion[2],quaternion[3]))

		velocities =Twist()
		accelerations=Twist()
		point = MultiDOFJointTrajectoryPoint([transforms],[velocities],[accelerations],rospy.Time(2))

		traj.points.append(point)

		self.firefly_command_publisher.publish(traj)
		time.sleep(1) #change the time if altitude val doesn't change
		if req and 0.8<self.current_z<1.2:
			self.__is_flying = True
			return True, 0
		elif not req and 0.0<=self.current_z<0.3:
			rospy.loginfo("Firefly Disarming")
			self.__is_armed = False
			self.__is_flying = False
			return True, 0
		else:
			return False,1
		


	def rotors_arm(self, req):
		# bool value
		# ----
		# bool success
		# uint8 result
		if req.value:
			rospy.loginfo("Firefly Arming")
			self.__is_armed = True
			time.sleep(1)  # waits 1 sec
			tk_req = CommandTOL()
			success, result = self.rotors_takeoff_land(1)  # arming is actually taking off in OFFBOARD flight mode
			return success, result
		else:
			rospy.loginfo("Firefly Disarming")
			self.__is_armed = False
			return True, 0

	def rotors_set_mode(self, req):
		# uint8 base_mode
		# string custom_mode
		# ----
		# bool mode_sent
		return False
	def rotors_land(self, req):
		
		self.__is_flying = False
		rospy.loginfo("Landing!")
		self.__is_flying = False

		cmd = CommandBool()
		cmd.value = False
		success, result = self.rotors_takeoff_land(0)
		return success, result
		
	def rotors_change_speed(self, speed):
		'''speed in m/s'''
		speed = self.CONSTRAINTS.constraint_speed(int(speed)*100)  # to cm/s

		# if self.__send_cmd("speed {}".format(str(speed))):
		# 	rospy.loginfo("Rotors drone speed updated to " + str(speed))
		# 	rospy.set_param("/rotors/speed", speed)
		return True
		# else:
		# 	return False

	def rotors_param_set(self, param_id, value=float(12.0)):
        # string param_id
        # mavros_msgs/ParamValue value
        # ----
        # bool success
        # mavros_msgs/ParamValue value

		if param_id == "MPC_XY_VEL_MAX":
			if value.integer != 0:
				speed = value.integer
			elif value.real != 0.0:
				speed = value.real
			else:
				speed = 0

			self.rotors_change_speed(speed)
			return True, value

		return False, ParamValue(integer=0, real=0.0)		
	
	def rotors_param_get(self, req):
        # string param_id
        # ----
        # bool success
        # mavros_msgs/ParamValue value
		return False, ParamValue(integer=0, real=0.0)

	
	def publish_position_desired(self, msg):
		"""
		Publish desired pose
		"""
		desired_x_to_go = msg.position.x
		desired_y_to_go = msg.position.y
		desired_z_to_go = msg.position.z
		vx_des = msg.velocity.x
		vy_des = msg.velocity.y
		vz_des = msg.velocity.z
		yaw_des = msg.yaw
		yaw_rate_des = msg.yaw_rate

		if (desired_x_to_go ==0.0 and desired_y_to_go ==0.0 and yaw_des == 0.0) and (vx_des >0.0 or vy_des >0.0  or yaw_rate_des>0) and vz_des >=0.0:
			desired_x_to_go=self.current_x+(vx_des*self.sample_time)
			desired_y_to_go=self.current_y+(vy_des*self.sample_time)
			desired_z_to_go=self.current_z+(vz_des*self.sample_time)
			desired_yaw_to_go = self.current_yaw +(yaw_rate_des*self.sample_time)
			print("Vel control")
		elif (desired_x_to_go ==0.0 and desired_y_to_go ==0.0 and vz_des ==0.0 and yaw_des == 0.0) and ((vx_des >0.0 or vy_des >0.0 or yaw_rate_des>0)and desired_z_to_go>0.0 ):
			desired_x_to_go=self.current_x+(vx_des*self.sample_time)
			desired_y_to_go=self.current_y+(vy_des*self.sample_time)
			desired_z_to_go= desired_z_to_go
			desired_yaw_to_go = self.current_yaw +(yaw_rate_des*self.sample_time)
			print("Mixed control")
		elif (vx_des ==0.0 and vy_des ==0.0 and vz_des ==0.0 and yaw_rate_des ==0.0) and (desired_x_to_go >0.0 or desired_y_to_go >0.0 or desired_z_to_go >0.0 or yaw_des>0.0):
			desired_x_to_go=desired_x_to_go
			desired_y_to_go=desired_y_to_go
			desired_z_to_go=desired_z_to_go
			desired_yaw_to_go=yaw_des
			print("Pos control")
		else:
			desired_x_to_go=self.current_x
			desired_y_to_go=self.current_y
			desired_z_to_go=self.current_z
			desired_yaw_to_go = self.current_yaw 
			print("no motion asked")



		
		# print("publishing","current x:",self.current_x, "current y:",self.current_y, "current z:", self.current_z, "current des_vel_x:", self.vx_des, "current des_vel_y:", self.vy_des, "current des_vel_z:", self.vz_des)

		quaternion = tf.transformations.quaternion_from_euler(0, 0, desired_yaw_to_go)

		traj = MultiDOFJointTrajectory()
		
		header = Header()
		header.stamp = rospy.Time()
		header.frame_id = 'frame'
		traj.joint_names.append('base_link')
		traj.header=header

		transforms =Transform(translation=Point(desired_x_to_go, desired_y_to_go, desired_z_to_go), rotation=Quaternion(quaternion[0],quaternion[1],quaternion[2],quaternion[3]))

		velocities =Twist()
		accelerations=Twist()
		point = MultiDOFJointTrajectoryPoint([transforms],[velocities],[accelerations],rospy.Time(2))

		traj.points.append(point)

		# time.sleep(0.1) #commented out for vel control
		self.firefly_command_publisher.publish(traj)


		

if __name__ == '__main__':
	try:
		
		
		rospy.init_node("rotors_driver", anonymous = True)
		rotors = Rotors_Driver()


		rospy.spin()
		# rospy.loginfo(" >> Published waypoint: x: {}, y: {}, z: {}, yaw: {}".format(x_des, y_des, z_des, yaw_des,vx_des, vy_des, vz_des))


	except rospy.ROSInterruptException:
		print("ROS Terminated")
		pass