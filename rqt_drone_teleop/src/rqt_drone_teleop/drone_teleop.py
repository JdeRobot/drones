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
from sensorsWidget import SensorsWidget

class DroneTeleop(Plugin):
	def __init__(self, context):
		super(DroneTeleop, self).__init__(context)
		# Give QObjects reasonable names
		self.setObjectName('DroneTeleop')

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
			'rqt_drone_teleop'), 'resource', 'DroneTeleop.ui')
		# Extend the widget with all attributes and children from UI file
		loadUi(ui_file, self._widget)
		# Give QObjects reasonable names
		self._widget.setObjectName('DroneTeleopUi')
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
			'rqt_drone_teleop'), 'resource', 'jderobot.png'))
		self._widget.img_logo.setPixmap(pixmap.scaled(121, 121))

		# Set Variables
		self.play_code_flag = False
		self.takeoff = False
		self._widget.term_out.setReadOnly(True)
		self._widget.term_out.setLineWrapMode(self._widget.term_out.NoWrap)

		# Set functions for each GUI Item
		self._widget.takeoffButton.clicked.connect(self.call_takeoff_land)
		self._widget.playButton.clicked.connect(self.call_play)
		self._widget.stopButton.clicked.connect(self.call_stop)
		self._widget.altdSlider.valueChanged.connect(self.alt_slider_val_changed)
		self._widget.rotationDial.valueChanged.connect(self.rotation_val_changed)

		# Add Publishers
		self.takeoff_pub = rospy.Publisher('gui/takeoff_land', Bool, queue_size=1)
		self.play_stop_pub = rospy.Publisher('gui/play_stop', Bool, queue_size=1)
		self.twist_pub = rospy.Publisher('gui/twist', Twist, queue_size = 1)

		#Add global variables
		self.shared_twist_msg = Twist()
		self.current_pose = Pose()
		self.current_twist = Twist()

		self.bridge = CvBridge()

		self.teleop = TeleopWidget(self)
		self._widget.tlLayout.addWidget(self.teleop)
		self.teleop.setVisible(True)
		self.sensors_widget = SensorsWidget(self)
		self._widget.sensorsCheck.stateChanged.connect(self.show_sensors_widget)

		# Add widget to the user interface
		context.add_widget(self._widget)

		# Add Subscibers
		rospy.Subscriber('iris/cam_frontal/image_raw', Image, self.cam_frontal_cb)
		rospy.Subscriber('iris/cam_ventral/image_raw', Image, self.cam_ventral_cb)
		rospy.Subscriber('interface/filtered_img', Image, self.filtered_img_cb)
		rospy.Subscriber('interface/threshed_img', Image, self.threshed_img_cb)
		rospy.Subscriber('mavros/local_position/pose', PoseStamped, self.pose_stamped_cb)
		rospy.Subscriber('mavros/local_position/velocity_body', TwistStamped, self.twist_stamped_cb)

	def show_sensors_widget(self, state):
		if state == Qt.Checked:
			self.sensors_widget.show()
		else:
			self.sensors_widget.hide()

	def msg_to_pixmap(self, msg):
		cv_img = self.bridge.imgmsg_to_cv2(msg)
		h, w, _ = cv_img.shape
		bytesPerLine = 3 * w
		q_img = QImage(cv_img.data, w, h, bytesPerLine, QImage.Format_RGB888)
		return QPixmap.fromImage(q_img).scaled(320, 240)

	def cam_frontal_cb(self, msg):
		self._widget.img_frontal.setPixmap(self.msg_to_pixmap(msg))
		self.sensors_widget.sensorsUpdate.emit()

	def cam_ventral_cb(self, msg):
		self._widget.img_ventral.setPixmap(self.msg_to_pixmap(msg))

	def threshed_img_cb(self, msg):
		self._widget.img_threshed.setPixmap(self.msg_to_pixmap(msg))

	def filtered_img_cb(self, msg):
		self._widget.img_filtered.setPixmap(self.msg_to_pixmap(msg))

	def pose_stamped_cb(self, msg):
		self.current_pose = msg.pose
	
	def twist_stamped_cb(self, msg):
		self.current_twist = msg.twist

	def call_takeoff_land(self):
		if self.takeoff == True:
			self._widget.takeoffButton.setText("Take Off")
			rospy.loginfo('Landing')
			self._widget.term_out.append('Landing')
			self.takeoff_pub.publish(Bool(False))
			self.takeoff = False
		else:
			self._widget.takeoffButton.setText("Land")
			rospy.loginfo('Taking off')
			self._widget.term_out.append('Taking off')
			self.takeoff_pub.publish(Bool(True))
			self.takeoff = True

	def call_play(self):
		if not self.play_code_flag:
			rospy.loginfo('Executing student code')
			self._widget.term_out.append('Executing student code')
			self.play_stop_pub.publish(Bool(True))
			self.play_code_flag = True
		else:
			rospy.loginfo('Already executing student code')
			self._widget.term_out.append('Already executing student code')

	def call_stop(self):
		if self.play_code_flag:
			rospy.loginfo('Stopping student code')
			self._widget.term_out.append('Stopping student code')
			self.play_stop_pub.publish(Bool(False))
			self.play_code_flag = False
		else:
			rospy.loginfo('Student code not running')
			self._widget.term_out.append('Student code not running')

	def alt_slider_val_changed(self, value):
		value = (1.0/(self._widget.altdSlider.maximum()/2)) * \
                    (value - (self._widget.altdSlider.maximum()/2))
		self._widget.altdValue.setText('%.2f' % value)
		rospy.logdebug('Altitude slider value changed to: %.2f', value)
		self.shared_twist_msg.linear.z = value
		self.twist_pub.publish(self.shared_twist_msg)

	def rotation_val_changed(self, value):
		value = (1.0/(self._widget.rotationDial.maximum()/2)) * \
                    (value - (self._widget.rotationDial.maximum()/2))
		self._widget.rotValue.setText('%.2f' % value)
		rospy.logdebug('Rotational dial value changed to: %.2f', value)
		self.shared_twist_msg.angular.z = value
		self.twist_pub.publish(self.shared_twist_msg)
	
	def setXYValues(self, x, y):
		self._widget.XValue.setText('%.2f' % -y)
		self._widget.YValue.setText('%.2f' % -x)
		rospy.logdebug('Teleop value changed to - x: %.2f y: %.2f', x, y)
		self.shared_twist_msg.linear.x = -y
		self.shared_twist_msg.linear.y = -x
		self.twist_pub.publish(self.shared_twist_msg)

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
