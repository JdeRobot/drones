#!/usr/bin/env python

from PyQt5.QtGui import QImage, QPainter, QPen
from PyQt5.QtCore import pyqtSignal, QPointF, Qt, QPoint
from PyQt5.QtWidgets import QWidget, QGridLayout

import os
import rospy
import rospkg

class TeleopWidget(QWidget):
    stop_sig = pyqtSignal()

    def __init__(self, win_parent, parent_function_name, side):
        super(TeleopWidget, self).__init__()
        self.parent_function = getattr(win_parent, parent_function_name)
        self.line = QPointF(0, 0)
        self.qimage = QImage()
        self.qimage.load(os.path.join(rospkg.RosPack().get_path('rqt_ground_robot_teleop'), 'resource', 'ball.png'))
        self.stop_sig.connect(self.stop)
        self.init_ui(side)

    def init_ui(self, side):
        layout = QGridLayout()
        self.setLayout(layout)
        self.setAutoFillBackground(True)
        p = self.palette()
        p.setColor(self.backgroundRole(), Qt.black)
        self.setPalette(p)
        self.resize(side, side)
        self.setMinimumSize(side, side)

    def stop(self):
        self.line = QPointF(0, 0)
        self.repaint()

    def mouseMoveEvent(self, e):
        if e.buttons() == Qt.LeftButton:
            x = e.x()-self.width()/2
            y = e.y()-self.height()/2
            self.line = QPointF(x, y)
            self.repaint()

    def paintEvent(self, e):
        _width = self.width()
        _height = self.height()
        width = 2
        painter = QPainter(self)
        pen = QPen(Qt.blue, width)
        painter.setPen(pen)
        #Centro del widget
        painter.translate(QPoint(_width/2, _height/2))
        #eje
        painter.drawLine(QPointF(-_width, 0), QPointF(_width, 0))
        painter.drawLine(QPointF(0, -_height), QPointF(0, _height))

        #con el raton
        pen = QPen(Qt.red, width)
        painter.setPen(pen)

        #Comprobamos que el raton este dentro de los limites
        if abs(self.line.x()*2) >= self.size().width():
            if self.line.x() >= 0:
                self.line.setX(self.size().width()/2)
            elif self.line.x() < 0:
                self.line.setX((-self.size().width()/2)+1)

        if abs(self.line.y()*2) >= self.size().height():
            if self.line.y() >= 0:
                self.line.setY(self.size().height()/2)
            elif self.line.y() < 0:
                self.line.setY((-self.size().height()/2)+1)

        painter.drawLine(QPointF(self.line.x(), -_height),
                         QPointF(self.line.x(), _height))

        painter.drawLine(QPointF(-_width, self.line.y()),
                         QPointF(_width, self.line.y()))

        #print "x: %f y: %f" % (self.line.x(), self.line.y())

        v_normalized = (1.0/(self.size().height()/2)) * self.line.y()
        v_normalized = float("{0:.2f}".format(v_normalized))
        w_normalized = (1.0/(self.size().width()/2)) * self.line.x()
        w_normalized = float("{0:.2f}".format(w_normalized))

        #print "v: %f w: %f" % (v_normalized,w_normalized)
        self.parent_function(w_normalized, v_normalized)
        painter.drawImage(self.line.x()-self.qimage.width()/2,
                          self.line.y()-self.qimage.height()/2, self.qimage)
