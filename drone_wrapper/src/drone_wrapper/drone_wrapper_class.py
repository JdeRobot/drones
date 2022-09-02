#!/usr/bin/env python

import rospy
import tf
import numpy as np
from math import radians
from cv_bridge import CvBridge
from std_msgs.msg import Header
from sensor_msgs.msg import NavSatFix, Image, BatteryState
from geometry_msgs.msg import PoseStamped, TwistStamped, Pose, Twist, TransformStamped, Point, PointStamped, Quaternion, Vector3
from mavros_msgs.msg import State, ExtendedState, PositionTarget, ParamValue
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest, CommandTOL, CommandTOLRequest, \
    ParamSet, ParamGet

EPSILON = 0.00001
CMD = None


class FRAMES:
    """
    Following Mavlink MAV_FRAME and ROS frames convention
    https://mavlink.io/en/messages/common.html#MAV_FRAME
    """
    LOCAL_ENU = 4  # map/world; East North Up
    LOCAL_NED = 1  # map/world; North East Down
    BODY_FLU = 13  # base_link; Forward Left Up
    BODY_FRD = 12  # base_link; Forward Right Down

    def __init__(self):
        self.tf_listener = tf.TransformListener()

    class UnknownFrameError(Exception):
        pass

    def get_frame(self, frame):
        if str(frame).lower() in ["enu", "local_enu", "map", "world"]:
            return "map"
        elif str(frame).lower() in ["ned", "local_ned", "map_ned", "world_ned"]:
            return "map_ned"
        elif str(frame).lower() in ["flu", "body_flu", "base_link"]:
            return "base_link"
        elif str(frame).lower() in ["frd", "body_frd", "base_link_frd"]:
            return "base_link_frd"
        else:
            raise self.UnknownFrameError

    def get_frame_id(self, frame):
        if str(frame).lower() in ["enu", "local_enu", "map", "world"]:
            return 4
        elif str(frame).lower() in ["ned", "local_ned", "map_ned", "world_ned"]:
            return 1
        elif str(frame).lower() in ["flu", "body_flu", "base_link"]:
            return 13
        elif str(frame).lower() in ["frd", "body_frd", "base_link_frd"]:
            return 12
        else:
            raise self.UnknownFrameError

    def id2frame(self, id_):
        if id_ == self.LOCAL_ENU:
            return "map"
        elif id_ == self.LOCAL_NED:
            return "map_ned"
        elif id_ == self.BODY_FLU:
            return "base_link"
        elif id_ == self.BODY_FRD:
            return "base_link_frd"
        else:
            raise self.UnknownFrameError

    def __transform_pose(self, from_, to, pose):
        if from_ == to:
            ps = PoseStamped()
            ps.header.frame_id = to
            ps.pose = pose
            return ps

        ps = PoseStamped()
        ps.header.frame_id = from_
        ps.pose = pose
        return self.tf_listener.transformPose(to, ps)

    def __transform_twist(self, from_, to, twist):
        """
        Points are not the same to Vectors
        Vectors are no fixed to an origin, then translations are not applied 
        to them (only rotations are applied) 
        
        tf_listener only has methos to transform Point, Pose or Quaternion
        http://wiki.ros.org/tf/TfUsingPython
        
        Transformation has to be done manual with the Homogeneus Transformation Matrix
        H = tf_listener.asMatrix(to, Header(frame_id=from_))
                 _                 _
                | r11  r12  r13  tx |
                | r21  r22  r23  ty |
            H = | r31  r32  r33  tz |
                |_ 0    0    0   s _|
         
            r --> rotations
            t --> translations
            s --> scale
        vin = np.array([twist.linear.x, twist.linear.y, twist.linear.z])
        result = np.dot(vin, H[:3, :3])
        """
        if from_ == to:
            tws = TwistStamped()
            tws.header.frame_id = to
            tws.twist = twist
            return tws

        H = self.tf_listener.asMatrix(to, Header(frame_id=from_))
        lin = np.array([twist.linear.x, twist.linear.y, twist.linear.z])
        ang = np.array([twist.angular.x, twist.angular.y, twist.angular.z])

        lin_out = np.dot(lin, H[:3, :3])
        ang_out = np.dot(ang, H[:3, :3])

        tws = TwistStamped()
        tws.header.frame_id = to
        tws.twist.linear.x = lin_out[0]
        tws.twist.linear.y = lin_out[1]
        tws.twist.linear.z = lin_out[2]
        tws.twist.angular.x = ang_out[0]
        tws.twist.angular.y = ang_out[1]
        tws.twist.angular.z = ang_out[2]
        return tws

    def transform(self, from_, to, input_):
        """
        from_: int or String
        to: int or String 
        input_: Pose() or Twist()
        """
        if isinstance(from_, str):
            from_ = self.get_frame(from_)
        elif isinstance(from_, int):
            from_ = self.id2frame(from_)
        if isinstance(to, str):
            to = self.get_frame(to)
        elif isinstance(to, int):
            to = self.id2frame(to)

        if isinstance(input_, Pose):
            return self.__transform_pose(from_, to, input_)
        if isinstance(input_, Twist):
            return self.__transform_twist(from_, to, input_)

    def enu2ned(self, input_):
        """
        input_: Pose() or Twist()
        """
        return self.transform("enu", "ned", input_)

    def ned2enu(self, input_):
        """
        input_: Pose() or Twist()
        """
        return self.transform("ned", "enu", input_)

    def flu2frd(self, input_):
        """
        input_: Pose() or Twist()
        """
        return self.transform("flu", "frd", input_)

    def frd2flu(self, input_):
        """
        input_: Pose() or Twist()
        """
        return self.transform("frd", "flu", input_)

    def enu2flu(self, input_):
        """
        input_: Pose() or Twist()
        """
        return self.transform("enu", "flu", input_)

    def flu2enu(self, input_):
        """
        input_: Pose() or Twist()
        """
        return self.transform("flu", "enu", input_)

    def enu2frd(self, input_):
        """
        input_: Pose() or Twist()
        """
        return self.transform("enu", "frd", input_)

    def frd2enu(self, input_):
        """
        input_: Pose() or Twist()
        """
        return self.transform("frd", "enu", input_)

    def ned2flu(self, input_):
        """
        input_: Pose() or Twist()
        """
        return self.transform("ned", "flu", input_)

    def flu2ned(self, input_):
        """
        input_: Pose() or Twist()
        """
        return self.transform("flu", "ned", input_)

    def ned2frd(self, input_):
        """
        input_: Pose() or Twist()
        """
        return self.transform("ned", "frd", input_)

    def frd2ned(self, input_):
        """
        input_: Pose() or Twist()
        """
        return self.transform("frd", "ned", input_)


class DroneWrapper:
    def state_cb(self, msg):
        self.state = msg
        rospy.logdebug('State updated')

    def extended_state_cb(self, msg):
        self.extended_state = msg
        rospy.logdebug('Extended State updated')

        self.rqt_extended_state_publisher.publish(self.extended_state)

    def battery_state_cb(self, msg):
        self.battery_state = msg
        rospy.logdebug('Battery State updated')

        self.battery_state_publisher.publish(self.battery_state)

    def pose_stamped_cb(self, msg):
        self.pose_stamped = msg
        rospy.logdebug('Pose updated')

        self.rqt_pose_publisher.publish(self.pose_stamped)

    def vel_body_stamped_cb(self, msg):
        self.vel_body_stamped = msg
        rospy.logdebug('Velocity (body) updated')

        self.rqt_velocity_body_publisher.publish(self.vel_body_stamped)

    def global_position_cb(self, msg):
        self.global_position = msg
        rospy.logdebug('Global position updated')

    def cam_frontal_cb(self, msg):
        self.frontal_image = msg
        rospy.logdebug('Frontal image updated')

        self.rqt_cam_frontal_publisher.publish(self.frontal_image)

    def cam_ventral_cb(self, msg):
        self.ventral_image = msg
        rospy.logdebug('Ventral image updated')

        self.rqt_cam_ventral_publisher.publish(self.ventral_image)

    def stay_armed_stay_offboard_cb(self, event):
        try:
            if self.state.mode != 'OFFBOARD':
                if self.request_mode('OFFBOARD'):
                    rospy.loginfo("OFFBOARD requested")
            elif not self.state.armed:
                if self.arm(True):
                    rospy.loginfo("Vehicle Armed")
        except rospy.ROSTimeMovedBackwardsException:
            pass

    def get_frontal_image(self):
        return self.bridge.imgmsg_to_cv2(self.frontal_image)

    def get_ventral_image(self):
        return self.bridge.imgmsg_to_cv2(self.ventral_image)

    def get_position(self, frame="map"):
        ps = self.frames_tf.transform(self.pose_stamped.header.frame_id, frame, 
                                    self.pose_stamped.pose)
        return np.array([ps.pose.position.x,
                         ps.pose.position.y,
                         ps.pose.position.z])

    def get_velocity(self, frame="flu"):
        tws = self.frames_tf.transform(self.vel_body_stamped.header.frame_id, frame, 
                                    self.vel_body_stamped.twist)
        return np.array([tws.twist.linear.x,
                         tws.twist.linear.y,
                         tws.twist.linear.z])

    def get_yaw_rate(self, frame="flu"):
        tws = self.frames_tf.transform(self.vel_body_stamped.header.frame_id, frame, 
                                    self.vel_body_stamped.twist)
        return tws.twist.angular.z

    def get_orientation(self, frame="map"):
        ps = self.frames_tf.transform(self.pose_stamped.header.frame_id, frame, 
                                    self.pose_stamped.pose)
        return np.array(tf.transformations.euler_from_quaternion([ps.pose.orientation.x,
                                                                  ps.pose.orientation.y,
                                                                  ps.pose.orientation.z,
                                                                  ps.pose.orientation.w]))

    def get_roll(self, frame="map"):
        return self.get_orientation(frame=frame)[0]

    def get_pitch(self, frame="map"):
        return self.get_orientation(frame=frame)[1]

    def get_yaw(self, frame="map"):
        return self.get_orientation(frame=frame)[2]

    def get_landed_state(self):
        return self.extended_state.landed_state

    def get_bat_percentage(self):
        return self.battery_state.percentage

    def param_set(self, param, value):
        if isinstance(value, float):
            val = ParamValue(integer=0, real=value)
        else:
            val = ParamValue(integer=value, real=0.0)

        try:
            resp = self.set_param(param_id=param, value=val)
            rospy.loginfo("ParamSet: %s", resp.success)
        except rospy.ServiceException as e:
            rospy.logwarn("Failed ParamSet: %s", e)

    def param_get(self, param):
        try:
            resp = self.get_param(param_id=param)
            rospy.loginfo("ParamGet: %s", resp.success)
        except rospy.ServiceException as e:
            rospy.logwarn("Failed ParamGet: %s", e)
            return None

        if resp.value.integer != 0:
            return resp.value.integer
        elif resp.value.real != 0.0:
            return resp.value.real
        else:
            return 0

    def arm(self, value=True):
        req = CommandBoolRequest()
        req.value = value
        if self.arm_client(req).success:
            rospy.loginfo('Arming/Disarming successful')
            return True
        else:
            rospy.logwarn('Arming/Disarming unsuccessful')
            return False

    def request_mode(self, mode='OFFBOARD'):
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

    def set_cmd_pos(self, x=0, y=0, z=0, az=0, max_vel=12.0, frame="map"):
        pose = Pose(position=Point(x=x, y=y, z=z),
                    orientation=Quaternion(*tf.transformations.quaternion_from_euler(0, 0, az)))
        pose = self.frames_tf.transform(frame, "map", pose).pose

        self.coord_frame = self.frames_tf.get_frame_id(frame)
        self.setpoint_raw.header.frame_id = self.frames_tf.get_frame(frame)
        self.setpoint_raw.coordinate_frame = self.coord_frame

        self.posx = pose.position.x
        self.posy = pose.position.y
        self.height = pose.position.z
        self.vx = 0
        self.vy = 0
        self.vz = 0

        self.setpoint_raw.position.x = pose.position.x
        self.setpoint_raw.position.y = pose.position.y
        self.setpoint_raw.position.z = pose.position.z
        self.setpoint_raw.yaw = tf.transformations.euler_from_quaternion([pose.orientation.x,
                                                                          pose.orientation.y,
                                                                          pose.orientation.z,
                                                                          pose.orientation.w])[2]

        global CMD
        CMD = 0  # POS
        self.setpoint_raw.type_mask = 3064  # xyz yaw

        self.param_set(param="MPC_XY_VEL_MAX", value=float(max_vel))
        self.setpoint_raw_publisher.publish(self.setpoint_raw)

    def set_cmd_vel(self, vx=0, vy=0, vz=0, az=0, frame="flu"):
        twist = Twist(linear=Vector3(x=vx, y=vy, z=vz),
                      angular=Vector3(x=0, y=0, z=az))
        twist = self.frames_tf.transform(frame, "flu", twist).twist

        self.coord_frame = 8
        # self.coord_frame = self.frames_tf.get_frame_id(frame)
        self.setpoint_raw.header.frame_id = self.frames_tf.get_frame(frame)
        self.setpoint_raw.coordinate_frame = self.coord_frame

        self.setpoint_raw.yaw_rate = twist.angular.z

        self.posx = self.pose_stamped.pose.position.x
        self.posy = self.pose_stamped.pose.position.y
        self.height = self.pose_stamped.pose.position.z
        self.vx = twist.linear.x
        self.vy = twist.linear.y
        self.vz = twist.linear.z

        global CMD
        CMD = 1  # VEL

        if abs(vx) <= EPSILON and abs(vy) <= EPSILON:
            self.is_xy = True
        else:
            self.setpoint_raw.velocity.x = twist.linear.x
            self.setpoint_raw.velocity.y = twist.linear.y

            self.is_xy = False

        if abs(vz) <= EPSILON:
            self.is_z = True
        else:
            self.setpoint_raw.velocity.z = twist.linear.z
            self.is_z = False

        if self.is_xy:
            if self.is_z:
                self.setpoint_raw.type_mask = 2040  # xyz yaw_rate
                # self.setpoint_raw.type_mask = 3064 # xyz yaw
            else:
                self.setpoint_raw.type_mask = 1991  # vx vy vy yaw_rate
                # self.setpoint_raw.type_mask = 3015  # vx vy vz yaw
                # self.setpoint_raw.type_mask = 3036 # x y vz yaw -> NOT SUPPORTED
        else:
            if self.is_z:
                self.setpoint_raw.type_mask = 1987  # vx vy vz z yaw_rate
                # self.setpoint_raw.type_mask = 2019  # vx vy z yaw_rate -> NOT SUPPORTED
                # self.setpoint_raw.type_mask = 3011  # vx vy vz z yaw
                # self.setpoint_raw.type_mask = 3043  # vx vy z yaw -> NOT SUPPORTED
            else:
                self.setpoint_raw.type_mask = 1991  # vx vy vy yaw_rate
                # self.setpoint_raw.type_mask = 3015  # vx vy vz yaw

        self.setpoint_raw_publisher.publish(self.setpoint_raw)

    def set_cmd_mix(self, vx=0, vy=0, z=0, az=0, frame="flu"):
        pose = Pose(position=Point(x=self.pose_stamped.pose.position.x, 
                                   y=self.pose_stamped.pose.position.y, z=z))
        pose = self.frames_tf.transform("map", "map", pose).pose

        twist = Twist(linear=Vector3(x=vx, y=vy, z=0),
                      angular=Vector3(x=0, y=0, z=az))
        twist = self.frames_tf.transform(frame, "flu", twist).twist

        self.coord_frame = 8
        # self.coord_frame = self.frames_tf.get_frame_id(frame)
        self.setpoint_raw.header.frame_id = self.frames_tf.get_frame(frame)
        self.setpoint_raw.coordinate_frame = self.coord_frame

        self.posx = self.pose_stamped.pose.position.x
        self.posy = self.pose_stamped.pose.position.y
        self.height = pose.position.z
        self.vx = twist.linear.x
        self.vy = twist.linear.y
        self.vz = 0

        self.setpoint_raw.position.z = pose.position.z
        self.setpoint_raw.velocity.x = twist.linear.x
        self.setpoint_raw.velocity.y = twist.linear.y
        self.setpoint_raw.yaw_rate = twist.angular.z

        global CMD
        CMD = 2  # MIX
        self.setpoint_raw.type_mask = 1987  # vx vy vz z yaw_rate

        self.setpoint_raw_publisher.publish(self.setpoint_raw)

    def repeat_setpoint_raw(self, event):
        self.setpoint_raw.coordinate_frame = self.coord_frame
        #self.setpoint_raw.header.frame_id = self.frames_tf.get_frame(self.coord_frame)

        self.setpoint_raw.position.x = self.posx
        self.setpoint_raw.position.y = self.posy
        self.setpoint_raw.position.z = self.height
        self.setpoint_raw.velocity.x = self.vx
        self.setpoint_raw.velocity.y = self.vy
        self.setpoint_raw.velocity.z = self.vz

        if CMD == 0:  # POS
            self.setpoint_raw.type_mask = 3064  # xyz yaw
        elif CMD == 1:  # VEL
            if self.is_xy:
                if self.is_z:
                    self.setpoint_raw.type_mask = 2040  # xyz yaw_rate
                else:
                    self.setpoint_raw.type_mask = 1991  # vx vy vy yaw_rate
            else:
                if self.is_z:
                    self.setpoint_raw.type_mask = 1987  # vx vy vz z yaw_rate
                else:
                    self.setpoint_raw.type_mask = 1991  # vx vy vy yaw_rate
        elif CMD == 2:  # MIX
            self.setpoint_raw.type_mask = 1987  # vx vy vz z yaw_rate
        else:
            self.setpoint_raw.type_mask = 3064  # xyz yaw
            print("[CMD error]: Mask set to position control")

        self.setpoint_raw_publisher.publish(self.setpoint_raw)

    def hold_setpoint_raw(self):
        if not self.setpoint_raw_flag:
            self.setpoint_raw_timer = rospy.Timer(rospy.Duration(nsecs=50000000), self.repeat_setpoint_raw)

    def takeoff(self, h=3, precision=0.05):
        if self.extended_state.landed_state == 2 or self.extended_state.landed_state == 3:
            rospy.loginfo('Drone is already flying!')
            return

        self.set_cmd_pos(0, 0, 0, 0)
        self.hold_setpoint_raw()
        self.arm(True)
        self.stay_armed_stay_offboard_timer = rospy.Timer(rospy.Duration(3), self.stay_armed_stay_offboard_cb)
        while True:
            while not (self.state.armed and self.state.mode == 'OFFBOARD'):
                self.rate.sleep()
            rospy.loginfo('Sleeping 1 secs to confirm change')
            rospy.sleep(1)
            if self.state.mode == 'OFFBOARD':
                break
        self.set_cmd_mix(z=h)
        rospy.loginfo('Taking off!!!')
        while True:
            if abs(self.pose_stamped.pose.position.z - h) < precision:
                break
        self.set_cmd_vel()

    # NOT USED
    def take_control(self):
        self.set_cmd_pos(0, 0, 0, 0)
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

    def br_base2map(self, event):
        now = rospy.Time.now()
        if self.last_pub_tf < now:
            my_pose = self.get_position(frame="map")
            my_orientation = self.get_orientation(frame="map")
            my_orientation[2] += radians(-90)
            q = tf.transformations.quaternion_from_euler(*my_orientation)
            self.br.sendTransform(my_pose, 
                                q, now, "base_link", "map")
            self.last_pub_tf = now

    def shutdown(self):
        self.br_timer.shutdown()
        self.setpoint_raw_timer.shutdown()
        self.stay_armed_stay_offboard_timer.shutdown()

    def __init__(self, name='drone', ns='', verbose=False):
        if name != 'rqt':
            if verbose:
                rospy.init_node(name, anonymous=True, log_level=rospy.DEBUG)
            else:
                rospy.init_node(name)
        self.ns = ns

        rospy.on_shutdown(self.shutdown)

        drone_model = rospy.get_param('drone_model', 'iris')  # default --> iris

        self.state = State()
        self.extended_state = ExtendedState()
        self.battery_state = BatteryState()
        self.pose_stamped = PoseStamped(header=Header(frame_id="map"))
        self.vel_body_stamped = TwistStamped()
        self.rate = rospy.Rate(20)
        self.setpoint_raw = PositionTarget()
        self.setpoint_raw_flag = False
        self.vz_factor = 0.4
        self.bridge = CvBridge()

        self.is_z = False
        self.is_xy = False

        self.posx = 0
        self.posy = 0
        self.height = 0
        self.vx = 0
        self.vy = 0
        self.vz = 0

        self.frames_tf = FRAMES()
        self.br = tf.TransformBroadcaster()
        self.br_timer = rospy.Timer(rospy.Duration(nsecs=10000000), self.br_base2map)
        self.last_pub_tf = rospy.Time(0)

        self.setpoint_raw_timer = rospy.Timer(rospy.Duration(nsecs=50000000), self.repeat_setpoint_raw)
        self.setpoint_raw_timer.shutdown()
        self.stay_armed_stay_offboard_timer = rospy.Timer(rospy.Duration(5), self.stay_armed_stay_offboard_cb)
        self.stay_armed_stay_offboard_timer.shutdown()

        rospy.wait_for_service(self.ns + 'mavros/cmd/arming')
        self.arm_client = rospy.ServiceProxy(ns + 'mavros/cmd/arming', CommandBool)
        rospy.wait_for_service(self.ns + 'mavros/set_mode')
        self.mode_client = rospy.ServiceProxy(ns + 'mavros/set_mode', SetMode)
        rospy.wait_for_service(self.ns + 'mavros/cmd/land')
        self.land_client = rospy.ServiceProxy(ns + 'mavros/cmd/land', CommandTOL)
        rospy.wait_for_service(self.ns + '/mavros/param/set')
        self.set_param = rospy.ServiceProxy(self.ns + '/mavros/param/set', ParamSet)
        rospy.wait_for_service(self.ns + '/mavros/param/get')
        self.get_param = rospy.ServiceProxy(self.ns + 'mavros/param/get', ParamGet)

        self.rqt_extended_state_publisher = rospy.Publisher(self.ns + 'drone_wrapper/extended_state', ExtendedState,
                                                            queue_size=1)
        self.battery_state_publisher = rospy.Publisher(self.ns + 'drone_wrapper/battery', BatteryState, queue_size=1)
        self.rqt_pose_publisher = rospy.Publisher(self.ns + 'drone_wrapper/local_position/pose', PoseStamped,
                                                  queue_size=1)
        self.rqt_velocity_body_publisher = rospy.Publisher(self.ns + 'drone_wrapper/local_position/velocity_body',
                                                           TwistStamped, queue_size=1)
        self.rqt_cam_frontal_publisher = rospy.Publisher(self.ns + 'drone_wrapper/cam_frontal/image_raw', Image,
                                                         queue_size=1)
        self.rqt_cam_ventral_publisher = rospy.Publisher(self.ns + 'drone_wrapper/cam_ventral/image_raw', Image,
                                                         queue_size=1)

        rospy.Subscriber(self.ns + 'mavros/state', State, self.state_cb)
        rospy.Subscriber(self.ns + 'mavros/extended_state', ExtendedState, self.extended_state_cb)
        rospy.Subscriber(self.ns + 'mavros/battery', BatteryState, self.battery_state_cb)
        rospy.Subscriber(self.ns + 'mavros/local_position/pose', PoseStamped, self.pose_stamped_cb)
        rospy.Subscriber(self.ns + 'mavros/local_position/velocity_body', TwistStamped, self.vel_body_stamped_cb)
        rospy.Subscriber(self.ns + 'mavros/global_position/global', NavSatFix, self.global_position_cb)

        # cam_frontal_topic = rospy.get_param('cam_frontal_topic', None)
        # cam_frontal_topic = rospy.get_param('cam_frontal_topic', '/iris/cam_frontal/image_raw')

        drone_model = rospy.get_param('drone_model', 'firefly')  # default --> iris

        self.frontal_image = Image()
        self.ventral_image = Image()
        cam_frontal_topic = '/' + drone_model + '/cam_frontal/image_raw'
        cam_ventral_topic = '/' + drone_model + '/cam_ventral/image_raw'
        rospy.Subscriber(cam_frontal_topic, Image, self.cam_frontal_cb)
        rospy.Subscriber(cam_ventral_topic, Image, self.cam_ventral_cb)

        self.setpoint_raw_publisher = rospy.Publisher(self.ns + 'mavros/setpoint_raw/local', PositionTarget,
                                                      queue_size=1)


if __name__ == "__main__":
    drone = DroneWrapper()
    rospy.spin()