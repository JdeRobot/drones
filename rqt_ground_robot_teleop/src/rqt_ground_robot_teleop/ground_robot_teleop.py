#!/usr/bin/env python

import os
import rospy
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
from python_qt_binding.QtGui import QIcon, QPixmap, QImage
from python_qt_binding.QtCore import pyqtSignal, Qt

from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
from std_msgs.msg import Bool, Float64
from geometry_msgs.msg import Pose, PoseStamped, Twist, TwistStamped

from teleopWidget import TeleopWidget

class GroundRobotTeleop(Plugin):
	def __init__(self, context):
		super(GroundRobotTeleop, self).__init__(context)
		# Give QObjects reasonable names
		self.setObjectName('GroundRobotTeleop')

		# Process standalone plugin command-line arguments
		from argparse import ArgumentParser
		parser = ArgumentParser()
		# Add argument(s) to the parser.
		parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")
		# args, unknowns = parser.parse_known_args(context.argv())
		# if not args.quiet:
		# 	print 'arguments: ', args
		# 	print 'unknowns: ', unknowns

		# Create QWidget
		self._widget = QWidget()
		# Get path to UI file which should be in the "resource" folder of this package
		ui_file = os.path.join(rospkg.RosPack().get_path(
			'rqt_ground_robot_teleop'), 'resource', 'GroundRobotTeleop.ui')
		# Extend the widget with all attributes and children from UI file
		loadUi(ui_file, self._widget)
		# Give QObjects reasonable names
		self._widget.setObjectName('GroundRobotTeleopUi')
		# Show _widget.windowTitle on left-top of each plugin (when
		# it's set in _widget). This is useful when you open multiple
		# plugins at once. Also if you open multiple instances of your
		# plugin at once, these lines add number to make it easy to
		# tell from pane to pane.
		if context.serial_number() > 1:
			self._widget.setWindowTitle(
				self._widget.windowTitle() + (' (%d)' % context.serial_number()))

		# Add logo
		pixmap = QPixmap(os.path.join(rospkg.RosPack().get_path(
			'rqt_ground_robot_teleop'), 'resource', 'jderobot.png'))
		self._widget.img_logo.setPixmap(pixmap.scaled(121, 121))

		# Set Variables
		self.linear_velocity_scaling_factor = 1
		self.angular_velocity_scaling_factor = 0.5
		
		# Set functions for each GUI Item
		self._widget.stop_button.clicked.connect(self.stop_robot)

		# Add Publishers
		self.twist_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

		#Add global variables
		self.twist_msg = Twist()
		self.stop_icon = QIcon()
		self.stop_icon.addPixmap(QPixmap(os.path.join(rospkg.RosPack().get_path(
			'rqt_ground_robot_teleop'), 'resource', 'stop.png')), QIcon.Normal, QIcon.Off)

		self.play_icon = QIcon()
		self.play_icon.addPixmap(QPixmap(os.path.join(rospkg.RosPack().get_path(
			'rqt_ground_robot_teleop'), 'resource', 'play.png')), QIcon.Normal, QIcon.Off)

		self.bridge = CvBridge()

		self.teleop = TeleopWidget(self, 'set_twist', 311)
		self._widget.teleop_layout.addWidget(self.teleop)
		self.teleop.setVisible(True)

		# Add widget to the user interface
		context.add_widget(self._widget)

		# Add Subscibers
		rospy.Subscriber('camera/rgb/image_raw', Image, self.cam_cb)

	def msg_to_pixmap(self, msg):
		cv_img = self.bridge.imgmsg_to_cv2(msg)
		h, w, _ = cv_img.shape
		bytesPerLine = 3 * w
		q_img = QImage(cv_img.data, w, h, bytesPerLine, QImage.Format_RGB888)
		return QPixmap.fromImage(q_img).scaled(320, 240)
		
	def cam_cb(self, msg):
		self._widget.img_camera.setPixmap(self.msg_to_pixmap(msg))
	
	def stop_robot(self):
		rospy.loginfo('Stopping Robot')
		self.teleop.stop()
		for i in range(5):
			self.twist_msg = Twist()
			self.twist_pub.publish(self.twist_msg)
			rospy.sleep(0.05)

	def set_twist(self, u, v):
		az = -self.linear_velocity_scaling_factor * u
		x = -self.angular_velocity_scaling_factor * v
		self._widget.rot_value.setText('%.2f' % az)
		self._widget.x_value.setText('%.2f' % x)
		self.twist_msg.linear.x = x
		self.twist_msg.angular.z = az
		self.twist_pub.publish(self.twist_msg)

	def shutdown_plugin(self):
		# TODO unregister all publishers here
		pass

	def save_settings(self, plugin_settings, instance_settings):
		# TODO save intrinsic configuration, usually using:
		# instance_settings.set_value(k, v)
		pass

	def restore_settings(self, plugin_settings, instance_settings):
		# TODO restore intrinsic configuration, usually using:
		# v = instance_settings.value(k)
		pass

	#def trigger_configuration(self):
	# Comment in to signal that the plugin has a way to configure
	# This will enable a setting button (gear icon) in each dock widget title bar
	# Usually used to open a modal configuration dialog
