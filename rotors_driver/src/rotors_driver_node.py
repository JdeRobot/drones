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
from enum import Enum

class States(Enum):
    
    ARMING = 1
    ARMED = 2
    TAKINGOFF = 3
    FLYING = 4
    LANDING = 5
    LANDED = 6
    DISARMING = 7
    DISARMED = 8

class RotorsConstraints:
    MIN_SPEED = 10
    MAX_SPEED = 100

    def constraint_speed(self, s):
        '''
        s: speed
        '''
        return self.MIN_SPEED if s < self.MIN_SPEED else self.MAX_SPEED if s > self.MAX_SPEED else s

class RotorsDriver():
	CONSTRAINTS = RotorsConstraints()

	def __init__(self):
		self.ns = rospy.get_namespace()
		self.sample_time = rospy.get_param(self.ns +'sample_time', 1.0)
		self.mav_name = rospy.get_param(self.ns +'drone_model', 'iris')
		self.drone_state_upd_freq = rospy.get_param(self.ns +'drone_state_timer_frequency', 0.01)
		self.misc_state_upd_freq = rospy.get_param(self.ns +'misc_state_timer_frequency', 1.0)
		self.enable_depth = rospy.get_param(self.ns +'enable_depth', False)
		self.drone_flight_state = States.LANDED
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

		self.frontal_image = Image()
		self.ventral_image = Image()
		
		self.arm_client = rospy.Service('mavros/cmd/arming', CommandBool, self.rotors_arm)
		self.mode_client = rospy.Service('mavros/set_mode', SetMode, self.rotors_set_mode)
		self.land_client = rospy.Service('mavros/cmd/land', CommandTOL, self.rotors_land)
		self.set_param = rospy.Service('mavros/param/set', ParamSet, self.rotors_param_set)
		self.get_param = rospy.Service('mavros/param/get', ParamGet, self.rotors_param_get)
		
		self.extended_state_publisher = rospy.Publisher('mavros/extended_state', ExtendedState,
                                                            queue_size=1)
		self.state_publisher = rospy.Publisher('mavros/state', State,
                                                            queue_size=1)
		self.battery_state_publisher = rospy.Publisher('mavros/battery', BatteryState, queue_size=1)
		self.pose_publisher = rospy.Publisher( 'mavros/local_position/pose', PoseStamped,
                                                  queue_size=1)
		self.global_position_publisher = rospy.Publisher( 'mavros/global_position/global', NavSatFix,
                                                  queue_size=1)
		self.velocity_body_publisher = rospy.Publisher('mavros/local_position/velocity_body',
                                                           TwistStamped, queue_size=1)
		self.cam_frontal_publisher = rospy.Publisher('/' + self.mav_name + '/cam_frontal/image_raw', Image,
                                                         queue_size=1)
		self.cam_ventral_publisher = rospy.Publisher('/' + self.mav_name + '/cam_ventral/image_raw', Image,
                                                         queue_size=1)


		if self.enable_depth:
			self.depth_frontal_image = Image()
			self.depth_ventral_image = Image()
			self.depth_cam_frontal_publisher = rospy.Publisher(self.ns + 'depth_cam_frontal/image_raw', Image,
                                                         queue_size=1)
			self.depth_cam_ventral_publisher = rospy.Publisher(self.ns + 'depth_cam_ventral/image_raw', Image,
                                                         queue_size=1)
			rospy.Subscriber(self.ns + 'vi_sensor_frontal/camera_depth/camera/image_raw', Image, self.depth_cam_frontal)
			rospy.Subscriber(self.ns + 'vi_sensor_ventral/camera_depth/camera/image_raw', Image, self.depth_cam_ventral)

		
		rospy.Subscriber("mavros/setpoint_raw/local", PositionTarget, self.publish_position_desired)
		

		rospy.Subscriber('/' + self.mav_name +"/ground_truth/odometry", Odometry, self.odom_callback)
		rospy.Subscriber('/' + self.mav_name +"/frontal_cam/camera_nadir/image_raw", Image, self.cam_frontal)
		rospy.Subscriber('/' + self.mav_name +"/ventral_cam/camera_nadir/image_raw", Image, self.cam_ventral)
		self.firefly_command_publisher = rospy.Publisher('/firefly/command/trajectory', MultiDOFJointTrajectory, queue_size=10)
		time.sleep(1.0)
		rospy.Timer(rospy.Duration(self.drone_state_upd_freq), self.drone_state_update_callback)
		rospy.Timer(rospy.Duration(self.misc_state_upd_freq), self.misc_state_update_callback)
		
	def drone_state_update_callback(self, event=None):
		
		pose = PoseStamped(pose=Pose(position=Point(x=self.current_x, y=self.current_y, z=self.current_z),
                                     orientation=Quaternion(x=float(self.quaternion[0]), y=float(self.quaternion[1]), z=float(self.quaternion[2]),
                                                            w=float(self.quaternion[3]))),header = Header(frame_id = self.frame_id))
		self.pose_publisher.publish(pose)

		twist = TwistStamped(twist=Twist(linear=Vector3(x=self.current_vx, y=self.current_vy, z=self.current_vz),
                                         angular=Vector3(x=self.current_ang_vx, y=self.current_ang_vy, z=self.current_ang_vz)),
										 header = Header(frame_id = self.frame_id))
		self.velocity_body_publisher.publish(twist)


		self.cam_frontal_publisher.publish(self.frontal_image)
		self.cam_ventral_publisher.publish(self.ventral_image)

		if self.enable_depth:
			self.depth_cam_frontal_publisher.publish(self.depth_frontal_image)
			self.depth_cam_ventral_publisher.publish(self.depth_ventral_image)
	
	def misc_state_update_callback(self, event=None):
		landed_state = 2 if self.drone_flight_state == 4 else 1
		ext_state = ExtendedState(vtol_state=0, landed_state=landed_state)
		self.extended_state_publisher.publish(ext_state)


		state = State(mode = "OFFBOARD", armed = True)
		self.state_publisher.publish(state)
		rospy.logdebug('State updated')

		bat = BatteryState(voltage=0.0, current=float('nan'), charge=float('nan'),
                           capacity=float('nan'), design_capacity=float('nan'), percentage=float('nan'),
                           power_supply_status=0, power_supply_health=0, power_supply_technology=0, present=True,
                           cell_voltage=[float('nan')], location="0",
                           serial_number="")
		self.battery_state_publisher.publish(bat)

		# Empty, global pos not known
		nav_sat = NavSatFix(status=NavSatStatus(status=-1, service=0), latitude=float('nan'),
                            longitude=float('nan'), altitude=float('nan'), position_covariance=[float('nan')] * 9,
                            position_covariance_type=0)
		self.global_position_publisher.publish(nav_sat)

	def cam_frontal(self, msg):
		self.frontal_image = msg
		rospy.logdebug('Frontal image updated')

	def cam_ventral(self, msg):
		self.ventral_image = msg
		rospy.logdebug('Ventral image updated')
	
	def depth_cam_frontal(self, msg):
		self.depth_frontal_image = msg
		rospy.logdebug('Frontal depth image updated')

	def depth_cam_ventral(self, msg):
		self.depth_ventral_image = msg
		rospy.logdebug('Ventral depth image updated')

	def odom_callback(self,msg):
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

		if self.current_z<0.09:
			self.drone_flight_state = States.LANDED
		else:
			self.drone_flight_state = States.FLYING


	def rotors_landing(self,req):
		quaternion = tf.transformations.quaternion_from_euler(0, 0, 0.0)
		traj = MultiDOFJointTrajectory()
		
		header = Header()
		header.stamp = rospy.Time()
		header.frame_id = 'frame'
		traj.joint_names.append('base_link')
		traj.header=header
		
		if not req:
			transforms =Transform(translation=Point(self.current_x, self.current_y, 0.0), rotation=Quaternion(quaternion[0],quaternion[1],quaternion[2],quaternion[3]))
			velocities =Twist(linear = Vector3(x=0.0, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0) )
			accelerations=Twist(linear = Vector3(x=0.0, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0) )

		
		point = MultiDOFJointTrajectoryPoint([transforms],[velocities],[accelerations],rospy.Time(2))

		traj.points.append(point)
		

		
		while not(0.0<=self.current_z<0.1):
			self.firefly_command_publisher.publish(traj)

		if not req :
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
			self.drone_flight_state = States.ARMING
			time.sleep(0.01) 
			self.drone_flight_state = States.ARMED
			return True, 1
		else:
			rospy.loginfo("Firefly Disarming")
			self.drone_flight_state = States.DISARMING
			time.sleep(0.01)
			self.drone_flight_state = States.DISARMED
			return False, 0

	def rotors_set_mode(self, req):
		# uint8 base_mode
		# string custom_mode
		# ----
		# bool mode_sent
		return False
	def rotors_land(self, req):
		
		rospy.loginfo("Landing!")
		self.drone_flight_state = States.LANDING
		success, result = self.rotors_landing(0)
		self.drone_flight_state = States.LANDED
		return success, result
		
	def rotors_change_speed(self, speed):
		'''speed in m/s'''
		speed = self.CONSTRAINTS.constraint_speed(int(speed)*100)  # to cm/s

		return True
	
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

		if self.drone_flight_state == 6 and int(desired_z_to_go):
			self.drone_flight_state = States.TAKINGOFF


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

	# Deleting (Calling destructor)
	def __del__(self):
		print('Destructor called, objs deleted.')


		

if __name__ == '__main__':
	try:
		
		
		rospy.init_node("rotors_driver", anonymous = True)
		rotors = RotorsDriver()


		rospy.spin()


	except rospy.ROSInterruptException:
		print("ROS Terminated")
		del rotors
