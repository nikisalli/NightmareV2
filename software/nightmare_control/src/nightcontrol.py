import sys
import rviz
import rospy
import os
import pyqtgraph as pg
import time
import numpy as np

from pyqtgraph import PlotWidget, plot
from std_msgs.msg import Float32
from PyQt5 import QtCore
from PyQt5 import QtGui
from PyQt5 import QtWidgets

from radial_bar import RadialBar

current_buf = []
for _ in range(200):
    current_buf.append(0)
    
time_buf = []
for i in range(200):
    time_buf.append(i)

buf = [current_buf, time_buf]

def listener():
    rospy.init_node('nightcontrol_listener')
    rospy.Subscriber("/nightmare/battery/voltage", Float32, callbackV)
    rospy.Subscriber("/nightmare/battery/current", Float32, callbackC)

def callbackV(data):
    r = rospy.Rate(100)
    voltage = round(data.data,2)
    myviz.batteryVWidget.value = voltage
    r.sleep()
    
    
def callbackC(data):
    r = rospy.Rate(50)
    global buf
    current = round(data.data,2)   
    buf[0].append(current)
    r.sleep()

class MyViz( QtWidgets.QWidget ):
    def __init__(self):
        QtWidgets.QWidget.__init__(self)

        # rviz widget
        self.frame = rviz.VisualizationFrame()
        self.frame.setSplashPath( "" )
        self.frame.initialize()
        reader = rviz.YamlConfigReader()
        config = rviz.Config()
        path = os.path.dirname(os.path.realpath(__file__))
        reader.readFile( config, path + "/nightmare_config.rviz" )
        self.frame.load( config )
        self.setWindowTitle( config.mapGetChild( "Title" ).getValue() )
        self.frame.setMenuBar( None )
        self.frame.setStatusBar( None )
        self.frame.setHideButtonVisibility( False )
        self.manager = self.frame.getManager()
        self.grid_display = self.manager.getRootDisplayGroup().getDisplayAt( 0 )
        
        # voltage gauge widget
        self.batteryVWidget = RadialBar()
        self.batteryVWidget.width = 250
        self.batteryVWidget.height = 250
        self.batteryVWidget.progressColor = QtGui.QColor(0,255,193)
        self.batteryVWidget.foregroundColor = QtGui.QColor(15,15,15)
        self.batteryVWidget.dialWidth = 15
        self.batteryVWidget.suffixText = "V"
        self.batteryVWidget.textFont = QtGui.QFont("Halvetica", 12)
        self.batteryVWidget.textColor = QtGui.QColor(0,255,193)
        self.batteryVWidget.setFixedSize(250,250)
        
        # graph textbox
        self.textbox = QtWidgets.QLabel()
        self.textbox.setFixedSize(80,40)
        self.textbox.text = "0.0A"
        self.textbox.setStyleSheet('QLabel#textbox {color: (0,255,193)}')
        
        # graph widget
        self.graph = pg.PlotWidget()
        self.graph.setBackground(None)
        self.graph.setYRange(0,10,0)
        self.graph.setFixedSize(400,300)
        self.graph.setMouseEnabled(False, False)
        self.graph.setTitle("battery current", color=(0,255,193), size='20')
        self.graph.setContentsMargins(0, 10, 30, 10)
        
        layout = QtWidgets.QHBoxLayout()
        
        vlayout = QtWidgets.QVBoxLayout()
        vlayout.addWidget( self.batteryVWidget, 0, QtCore.Qt.AlignHCenter )
        vlayout.addWidget( self.graph )
        
        layout.addLayout(vlayout)
        layout.addWidget( self.frame )
        
        self.setLayout(layout)
    
    def onTopButtonClick( self ):
        print("lol")

    def grab_data( self , buf):
        pen = pg.mkPen(color=(0,255,193), width=4)
        self.graph.plot(buf[1], buf[0][-200:], clear=True, pen=pen)
    
if __name__ == '__main__':
    app = QtWidgets.QApplication( sys.argv )
    
    myviz = MyViz()
    myviz.show()
    
    timer = QtCore.QTimer()
    timer.timeout.connect(lambda: myviz.grab_data(buf))
    timer.start(10)
    
    listener()
    sys.exit(app.exec_())
    rospy.spin()