#! /usr/bin/env python

import rospy
from mavros_msgs.srv import CommandTOL, SetMode, CommandBool, ParamGet, ParamSet
from mavros_msgs.msg import State, ExtendedState, PositionTarget, ParamValue
from sensor_msgs.msg import BatteryState, NavSatFix, Image, NavSatStatus
from geometry_msgs.msg import PoseStamped, TwistStamped, Pose, Point, Quaternion, Twist, Vector3
from cv_bridge import CvBridge
from tf.transformations import quaternion_from_euler

import cv2

from math import degrees
import threading
import socket
import time


def process_mask(mask):
    if mask == 3064:  # xyz yaw
        return mask
    elif mask == 1991:  # vx vy vz yaw_rate
        return mask
    elif mask == 1987:  # vx vy vz z yaw_rate
        return mask
    else:
        return 1991  # vx vy vz yaw_rate


class TelloConnectionError(Exception):
    pass


# TODO: x and y poses dont update during velocity control
class TelloDriver:
    STILL_ALIVE_RATE = 10  # sec
    PUB_RATE = 100000000  # nsec --> 0.1 secs
    RESP_TIMEOUT = 7  # sec
    CMD_ADDRESS = ('192.168.10.1', 8889)
    STATE_ADDRESS = ('0.0.0.0', 8890)
    VIDEO_ADDRESS = ('0.0.0.0', 11111)
    LOCAL_ADDRESS = ('0.0.0.0', 9000)

    def __init__(self, emergency_disabled=False):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.state_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.__emergency_disabled = emergency_disabled

        self.__responses = []
        self.__state_dict = {}
        self.__running = True  # started as True to recv responses and data
        self.__flight_mode = "MANUAL"  # app
        self.__is_armed = False
        self.__is_flying = False
        self.__is_stream = False

        self.__x = 0.0  # cm
        self.__y = 0.0  # cm
        self.__h = 0.0  # cm
        self.__yaw = 0.0  # degrees
        self.__vx = 0.0  # cm/s
        self.__vy = 0.0  # cm/s
        self.__vz = 0.0  # cm/s
        self.__yaw_rate = 0.0  # degrees/s

        self.state_pub = rospy.Publisher('mavros/state', State, queue_size=10)
        self.ext_state_pub = rospy.Publisher('mavros/extended_state', ExtendedState, queue_size=10)
        self.pose_pub = rospy.Publisher('mavros/local_position/pose', PoseStamped, queue_size=10)
        self.vel_body_pub = rospy.Publisher('mavros/local_position/velocity_body', TwistStamped, queue_size=10)
        self.global_pub = rospy.Publisher('mavros/global_position/global', NavSatFix, queue_size=10)

        self.bridge = CvBridge()
        self.img_pub = rospy.Publisher('tello/cam_frontal/image_raw', Image, queue_size=10)

        self.bat_status_pub = rospy.Publisher('mavros/battery', BatteryState, queue_size=10)  # EXTRA

        rospy.Subscriber('mavros/setpoint_raw/local', PositionTarget, self.setpoint_cb)

        self.takeoff_srv = rospy.Service('mavros/cmd/takeoff', CommandTOL, self.tello_takeoff)  # EXTRA

        self.arm_srv = rospy.Service('mavros/cmd/arming', CommandBool, self.tello_arm)
        self.set_mode_srv = rospy.Service('mavros/set_mode', SetMode, self.tello_set_mode)
        self.land_srv = rospy.Service('mavros/cmd/land', CommandTOL, self.tello_land)
        self.param_set_srv = rospy.Service('mavros/param/set', ParamSet, self.tello_param_set)
        self.param_get_srv = rospy.Service('mavros/param/get', ParamGet, self.tello_param_get)

        self.emergency_srv = rospy.Service('tello/emergency', CommandBool, self.emergency_stop)

        rospy.on_shutdown(self.shutdown)

        self.response_handler = threading.Thread(target=self.rcv_response)
        self.data_handler = threading.Thread(target=self.rcv_data)
        self.video_handler = threading.Thread(target=self.rcv_video)

        try:
            self.sock.bind(self.LOCAL_ADDRESS)
            self.state_sock.bind(self.STATE_ADDRESS)

            self.response_handler.start()

            self.connect()
            self.set_stream(on=True)

            self.data_handler.start()
            self.video_handler.start()
        except Exception as ex:
            rospy.logerr(ex)
            self.shutdown()

        rospy.Timer(rospy.Duration(nsecs=self.PUB_RATE), self.send_data)
        if emergency_disabled:
            rospy.Timer(rospy.Duration(secs=self.STILL_ALIVE_RATE), self.__still_alive)

    def __still_alive(self, event):
        self.__send_cmd("battery?")

    def __send_cmd(self, cmd, blocking=True, timeout=RESP_TIMEOUT):
        print("Sending... {}".format(cmd))  # DEBUG
        self.sock.sendto(cmd.encode(encoding="utf-8"), self.CMD_ADDRESS)

        if blocking:
            start_time = time.time()
            while not self.__responses:
                if time.time() - start_time > timeout:
                    rospy.logwarn("Command {cmd} did not receive a response.".format(cmd=cmd))
                    return False
                time.sleep(0.1)  # waits a bit

            resp = self.__responses.pop(0).decode("utf-8")
            if resp.lower() == "ok":
                return True
            else:
                return False

    def connect(self):
        if self.__send_cmd("command"):
            rospy.loginfo("Tello Connected")

            self.__running = True
            self.__flight_mode = "OFFBOARD"
            return True
        else:
            self.__running = False
            raise TelloConnectionError("Connection failed. Tello not ready.")

    def set_stream(self, on=True):
        if on:
            if self.__send_cmd("streamon"):
                rospy.loginfo("Tello Stream On")
                self.__is_stream = True
        else:
            if self.__send_cmd("streamoff"):
                rospy.loginfo("Tello Stream Off")
                self.__is_stream = False

    def emergency_stop(self, req):
        if req.value:
            resp = self.__send_cmd("emergency")
            return resp, 0
        return False, 1

    def tello_takeoff(self, req):
        # float32 min_pitch  # used by takeoff
        # float32 yaw
        # float32 latitude
        # float32 longitude
        # float32 altitude
        # ---
        # bool success
        # uint8 result

        if self.__send_cmd("takeoff"):
            rospy.loginfo("Taking off!")
            self.__is_flying = True
            return True, 0
        else:
            return False, 1

    def tello_arm(self, req):
        # bool value
        # ----
        # bool success
        # uint8 result
        if req.value:
            rospy.loginfo("Tello Arming")
            self.__is_armed = True
            time.sleep(1)  # waits 1 sec
            tk_req = CommandTOL()
            success, result = self.tello_takeoff(tk_req)  # arming is actually taking off in OFFBOARD flight mode
            return success, result
        else:
            rospy.loginfo("Tello Disarming")
            self.__is_armed = False
            return True, 0

    def tello_set_mode(self, req):
        # uint8 base_mode
        # string custom_mode
        # ----
        # bool mode_sent
        return False

    def tello_land(self, req):
        if self.__send_cmd("land"):
            rospy.loginfo("Landing!")
            self.__is_flying = False

            cmd = CommandBool()
            cmd.value = False
            success, result = self.tello_arm(cmd)
            return success, result
        else:
            return False, 1

    def tello_param_set(self, req):
        # string param_id
        # mavros_msgs/ParamValue value
        # ----
        # bool success
        # mavros_msgs/ParamValue value
        return False, ParamValue(integer=0, real=0.0)

    def tello_param_get(self, req):
        # string param_id
        # ----
        # bool success
        # mavros_msgs/ParamValue value
        return False, ParamValue(integer=0, real=0.0)

    def setpoint_cb(self, msg):
        # uint8 coordinate_frame
        # uint16 type_mask
        # geometry_msgs/Point position
        # geometry_msgs/Vector3 velocity
        # geometry_msgs/Vector3 acceleration_or_force
        # float32 yaw
        # float32 yaw_rate

        frame = msg.coordinate_frame
        offset = False  # TODO, offset not used
        if frame == 1:
            # FRAME_LOCAL_NED = 1 --> Local coordinate frame, Z-down (x: North, y: East, z: Down).
            rospy.logwarn_once("Compass data not known. Using FRD as NED frame.")
            frame = 20
            offset = False
        elif frame == 7:
            # FRAME_LOCAL_OFFSET_NED = 7 --> Offset to the current local frame.
            # Anything expressed in this frame should be added to the current local frame position.
            rospy.logwarn_once("Compass data not known. Using FRD as NED frame.")
            frame = 20
            offset = True
        elif frame == 8:
            # FRAME_BODY_NED = 8 --> DEPRECATED: MAV_FRAME_BODY_FRD: Body fixed frame of reference,
            # Z-down (x: Forward, y: Right, z: Down).
            rospy.logwarn_once("Coordination frame FRAME_BODY_NED deprecated. Using MAV_FRAME_BODY_FRD.")
            frame = 12
            offset = False
        elif frame == 9:
            # FRAME_BODY_OFFSET_NED = 9  --> DEPRECATED
            rospy.logwarn_once("Coordination frame FRAME_BODY_NED deprecated. Using MAV_FRAME_BODY_FRD.")
            frame = 12
            offset = True
        elif frame == 12:
            # MAV_FRAME_BODY_FRD --> Body fixed frame of reference, Z-down (x: Forward, y: Right, z: Down).
            pass
        elif frame == 20:
            # MAV_FRAME_LOCAL_FRD
            pass
        elif frame == 21:
            # MAV_FRAME_LOCAL_FLU
            pass
        else:
            rospy.logerr("Unknown Coordinate Frame. Setpoint skipped.")
            return

        # TODO: overriding frame
        frame = 21
        is_abs = True if (frame == 20 or frame == 21) else False
        is_frd = True if (frame == 12 or frame == 20) else False
        is_vel = False

        mask = process_mask(msg.type_mask)  # filter valid mask, else vel control
        mask = "{0:012b}".format(int(mask))
        if not bool(int(mask[-1])):
            target_x = msg.position.x*100  # cm
            if is_abs:
                x = target_x - self.__x
                self.__x = target_x
            else:
                x = target_x
                self.__x += target_x

            if x > 0:
                self.__send_cmd("forward {}".format(abs(x)), False)  # cm
            elif x < 0:
                self.__send_cmd("back {}".format(abs(x)), False)  # cm
            else:
                pass  # already at target x
                # print("Already at target x")
        if not bool(int(mask[-2])):
            target_y = msg.position.y*100  # cm
            if is_abs:
                y = target_y - self.__y
                self.__y = target_y
            else:
                y = target_y
                self.__y += target_y

            cmd_right = "right" if is_frd else "left"
            cmd_left = "left" if is_frd else "right"
            if y > 0:
                self.__send_cmd(cmd_right + " {}".format(abs(y)), False)  # cm
            elif y < 0:
                self.__send_cmd(cmd_left + " {}".format(abs(y)), False)  # cm
            else:
                pass  # already at target y
                # print("Already at target y")
        if not bool(int(mask[-3])):
            target_z = msg.position.z*100  # cm
            if is_abs:
                z = target_z - self.__h
                # self.__h = target_z  # barometer
            else:
                z = target_z
                # self.__h += target_z  # barometer

            cmd_down = "down" if is_frd else "up"
            cmd_up = "up" if is_frd else "down"
            if z > 0:
                self.__send_cmd(cmd_down + " {}".format(abs(z)), False)  # cm
            elif z < 0:
                self.__send_cmd(cmd_up + " {}".format(abs(z)), False)  # cm
            else:
                pass  # already at target z
                # print("Already at target z")
        if not bool(int(mask[-4])):
            vx = msg.velocity.x*100  # cm/s
            self.__vx = vx
            if vx != 0:
                is_vel = True
        if not bool(int(mask[-5])):
            vy = msg.velocity.y*100  # cm/s
            if is_frd:
                self.__vy = -vy
            else:
                self.__vy = vy
            if vy != 0:
                is_vel = True
        if not bool(int(mask[-6])):
            vz = msg.velocity.z*100  # cm/s
            if is_frd:
                self.__vz = -vz
            else:
                self.__vz = vz
            if vz != 0:
                is_vel = True
        if not bool(int(mask[-7])):
            pass
            rospy.logwarn("AFX target not supported.")
        if not bool(int(mask[-8])):
            pass
            rospy.logwarn("AFY target not supported.")
        if not bool(int(mask[-9])):
            pass
            rospy.logwarn("AFZ target not supported.")
        if not bool(int(mask[-10])):
            pass
            rospy.logwarn("FORCE target not supported.")
        if not bool(int(mask[-11])):
            target_yaw = degrees(msg.yaw)
            if is_abs:
                yaw = target_yaw - self.__yaw
                self.__yaw = target_yaw
            else:
                yaw = target_yaw
                self.__yaw += target_yaw

            if yaw > 0:
                self.__send_cmd("cw {}".format(abs(yaw)), False)  # degrees
            elif yaw < 0:
                self.__send_cmd("ccw {}".format(abs(yaw)), False)  # degrees
            else:
                pass  # already at target yaw
                # print("Already at target yaw")
        if not bool(int(mask[-12])):
            yaw_rate = degrees(msg.yaw_rate)  # degrees/s
            self.__yaw_rate = yaw_rate
            if yaw_rate != 0:
                is_vel = True

        if is_vel:
            self.__send_cmd("rc {} {} {} {}".format(self.__vy, self.__vx, self.__vz, self.__yaw_rate), blocking=False)

        # DEBUG
        print(msg.header.seq)
        print("pos", msg.position.x, msg.position.y, msg.position.z)
        print("vel", msg.velocity.x, msg.velocity.y, msg.velocity.z)
        print("yaw", msg.yaw)
        print("yaw_rate", msg.yaw_rate)

    def rcv_response(self):
        while self.__running:
            try:
                data, address = self.sock.recvfrom(1024)

                address = address[0]
                if address == self.CMD_ADDRESS[0]:
                    self.__responses.append(data)

                # rospy.loginfo("Data {data} received from {address}".format(data=data, address=address))
            except Exception as e:
                if self.__running:
                    print(e)
                break

    def rcv_data(self):
        while self.__running:
            try:
                msg, _ = self.state_sock.recvfrom(1024)
                msg = msg.decode(encoding="utf-8")
                state = msg.strip().split(";")

                for field in state:
                    pair = field.split(":")
                    if len(pair) < 2:
                        continue

                    key = pair[0]
                    value = pair[1]

                    self.__state_dict[key] = value

                # print("states: {}".format(msg))
            except Exception as err:
                print(err)

    def send_data(self, event):
        if not self.__running:
            return

        try:
            bat_percent = int(self.__state_dict["bat"])
        except KeyError:
            bat_percent = float('nan')
            rospy.logwarn("Battery state unknown.")

        try:
            height = float(self.__state_dict["h"])/100  # m
            self.__h = height*100  # cm
        except KeyError:
            height = float('nan')
            rospy.logwarn("Height state unknown.")

        try:
            pitch = float(self.__state_dict["pitch"])
        except KeyError:
            pitch = float('nan')
            rospy.logwarn("Pitch state unknown.")
        try:
            roll = float(self.__state_dict["roll"])
        except KeyError:
            roll = float('nan')
            rospy.logwarn("Roll state unknown.")
        try:
            yaw = float(self.__state_dict["yaw"])
            self.__yaw = yaw  # degrees
        except KeyError:
            yaw = float('nan')
            rospy.logwarn("Yaw state unknown.")
        q = quaternion_from_euler(roll, pitch, yaw)

        try:
            vx = float(self.__state_dict["vgx"])
        except KeyError:
            vx = float('nan')
            rospy.logwarn("Vgx state unknown.")
        try:
            vy = float(self.__state_dict["vgy"])
        except KeyError:
            vy = float('nan')
            rospy.logwarn("Vgy state unknown.")
        try:
            vz = float(self.__state_dict["vgz"])
        except KeyError:
            vz = float('nan')
            rospy.logwarn("Vgz state unknown.")

        try:
            ax = float(self.__state_dict["agx"])
        except KeyError:
            ax = float('nan')
            rospy.logwarn("Agx state unknown.")
        try:
            ay = float(self.__state_dict["agy"])
        except KeyError:
            ay = float('nan')
            rospy.logwarn("Agy state unknown.")
        try:
            az = float(self.__state_dict["agz"])
        except KeyError:
            az = float('nan')
            rospy.logwarn("Agz state unknown.")

        is_armed = self.__is_armed
        # landed_state = 1 if is_on_ground else 2 if is_flying else 0
        landed_state = 2 if self.__is_flying else 1

        state = State(connected=self.__running, armed=is_armed, guided=False, manual_input=False,
                      mode=self.__flight_mode, system_status=0)
        self.state_pub.publish(state)

        ext_state = ExtendedState(vtol_state=0, landed_state=landed_state)
        self.ext_state_pub.publish(ext_state)

        pose = PoseStamped(pose=Pose(position=Point(x=self.__x, y=self.__y, z=height),
                                     orientation=Quaternion(x=float(q[1]), y=float(q[2]), z=float(q[3]),
                                                            w=float(q[0]))))
        self.pose_pub.publish(pose)

        twist = TwistStamped(twist=Twist(linear=Vector3(x=vx, y=vy, z=vz),
                                         angular=Vector3(x=float('nan'), y=float('nan'), z=float('nan'))))
        self.vel_body_pub.publish(twist)

        # Empty, global pos not known
        nav_sat = NavSatFix(status=NavSatStatus(status=-1, service=0), latitude=float('nan'),
                            longitude=float('nan'), altitude=float('nan'), position_covariance=[float('nan')] * 9,
                            position_covariance_type=0)
        self.global_pub.publish(nav_sat)

        bat = BatteryState(voltage=0.0, current=float('nan'), charge=float('nan'),
                           capacity=float('nan'), design_capacity=float('nan'), percentage=bat_percent,
                           power_supply_status=0, power_supply_health=0, power_supply_technology=0, present=True,
                           cell_voltage=[float('nan')], location="0",
                           serial_number="")
        self.bat_status_pub.publish(bat)

    def rcv_video(self):
        cap = cv2.VideoCapture("udp://@{ip}:{port}".format(ip=self.VIDEO_ADDRESS[0], port=str(self.VIDEO_ADDRESS[1])))
        while self.__running:
            try:
                ret, frame = cap.read()
                if ret:
                    img = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                    self.img_pub.publish(self.bridge.cv2_to_imgmsg(img))
            except Exception as err:
                print(err)

        cap.release()
        cv2.destroyAllWindows()

    def shutdown(self):
        if self.__is_flying:
            self.tello_land(CommandTOL())
        if self.__is_stream:
            self.set_stream(on=False)
        self.__running = False
        try:
            # self.sock.close()
            self.sock.shutdown(socket.SHUT_RDWR)
        except Exception as e:
            if e.errno == 107:
                pass
            else:
                raise e
        self.state_sock.close()
        if self.response_handler.is_alive():
            self.response_handler.join()
        if self.data_handler.is_alive():
            self.data_handler.join()
        if self.video_handler.is_alive():
            self.video_handler.join()

        rospy.loginfo("Tello driver shutting down")

    def __del__(self):
        self.shutdown()


def main():
    rospy.init_node('tello_driver')

    try:
        TelloDriver()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
