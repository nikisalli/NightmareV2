import sys
import rviz
import rospy
import threading
import os
from std_msgs.msg import Float32
from PyQt5 import QtCore
from PyQt5 import QtGui
from PyQt5 import QtWidgets
from PyQt5 import QtQml
from PyQt5 import QtQuick
from PyQt5 import QtQuickWidgets

from radial_bar import RadialBar

global voltage
voltage = 6.9

def listener():
    rospy.init_node('nightcontrol_listener')
    rospy.Subscriber("/nightmare/battery/voltage", Float32, callback)

def callback(data):
    global voltage
    voltage = round(data.data,2)
    
    
class MyClass(QtCore.QObject):
    randomValueChanged = QtCore.pyqtSignal(float)
    #self.randomValueChanged.emit(v)
    def __init__(self, parent=None):
        super(MyClass, self).__init__(parent)
        self.m_randomValue = 0
    
    @QtCore.pyqtProperty(float, notify=randomValueChanged)
    def randomValue(self):
        return self.m_randomValue
    
    @randomValue.setter
    def randomValue(self, v):
        if self.m_randomValue == v:
            return
        self.m_randomValue = v
        self.randomValueChanged.emit(v)
    
    def random_value(self):
        self.randomValue = voltage
        
class MyViz( QtWidgets.QWidget ):
    def __init__(self, parent=None):
        QtWidgets.QMainWindow.__init__(self, parent)
        
        # rviz widget
        self.frame = rviz.VisualizationFrame()
        self.frame.setSplashPath( "" )
        self.frame.initialize()
        reader = rviz.YamlConfigReader()
        config = rviz.Config()
        path = dir_path = os.path.dirname(os.path.realpath(__file__))
        reader.readFile( config, path + "/nightmare_config.rviz" )
        self.frame.load( config )
        self.setWindowTitle( config.mapGetChild( "Title" ).getValue() )
        self.frame.setMenuBar( None )
        self.frame.setStatusBar( None )
        self.frame.setHideButtonVisibility( False )
        self.manager = self.frame.getManager()
        self.grid_display = self.manager.getRootDisplayGroup().getDisplayAt( 0 )        
        
        # voltage gauge widget
        self.batteryVWidget = QtQuickWidgets.QQuickWidget()
        self.batteryVWidget.setResizeMode(QtQuickWidgets.QQuickWidget.SizeRootObjectToView)
        
        # layout
        layout = QtWidgets.QHBoxLayout()
        
        layout.addWidget( self.frame )
        layout.addWidget( self.batteryVWidget )
        
        self.setLayout( layout )
        
if __name__ == '__main__':
    app = QtWidgets.QApplication( sys.argv )
    app.setAttribute(QtCore.Qt.AA_DontCreateNativeWidgetSiblings)
    
    myviz = MyViz()
    
    QtQml.qmlRegisterType(RadialBar, "SDK", 1,0, "RadialBar")
    batteryWidget = MyClass()
    
    myviz.batteryVWidget.rootContext().setContextProperty("batteryWidget",batteryWidget)
    path = dir_path = os.path.dirname(os.path.realpath(__file__))
    myviz.batteryVWidget.setSource(QtCore.QUrl.fromLocalFile(path + '/qml_voltage_widget.qml'))
    
    timer = QtCore.QTimer()
    timer.timeout.connect(batteryWidget.random_value)
    timer.start(1)
    
    myviz.resize( 500, 500 )
    myviz.show()
    
    listener()
    sys.exit(app.exec_())
    rospy.spin()