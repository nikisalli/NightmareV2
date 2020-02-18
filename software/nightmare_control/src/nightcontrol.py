import sys
import rviz
import rospy
from python_qt_binding import QtCore
from python_qt_binding import QtGui
from python_qt_binding import QtWidgets
from python_qt_binding import QtQml
from python_qt_binding import QtQuick
from python_qt_binding import QtQuickWidgets

from radial_bar import RadialBar

voltage = 7.2

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
        reader.readFile( config, "/home/nik/Desktop/NightmareV2/software/nightmare_control/src/nightmare_config.rviz" )
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
        
        #hlayout = QtWidgets.QHBoxLayout()
        layout.addWidget( self.frame )
        
        #layout.addWidget( hlayout )
        layout.addWidget( self.batteryVWidget )
        self.setLayout( layout )
        
if __name__ == '__main__':
    app = QtWidgets.QApplication( sys.argv )
    app.setAttribute(QtCore.Qt.AA_DontCreateNativeWidgetSiblings)
    
    myviz = MyViz()
    
    QtQml.qmlRegisterType(RadialBar, "SDK", 1,0, "RadialBar")
    batteryWidget = MyClass()
    
    myviz.batteryVWidget.rootContext().setContextProperty("batteryWidget",batteryWidget)
    myviz.batteryVWidget.setSource(QtCore.QUrl.fromLocalFile('/home/nik/Desktop/NightmareV2/software/nightmare_control/src/qml_voltage_widget.qml'))
    
    timer = QtCore.QTimer()
    timer.timeout.connect(batteryWidget.random_value)
    timer.start(1)
    
    myviz.resize( 500, 500 )
    myviz.show()

    app.exec_()
