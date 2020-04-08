from PyQt5 import QtGui
from PyQt5 import QtCore
from PyQt5 import QtWidgets
import sys
from enum import Enum

class Direction(Enum):
    Left = 0
    Right = 1
    Up = 2
    Down = 3

class Joystick(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super(Joystick, self).__init__(parent)
        self.setMinimumSize(300, 300)
        self.movingOffset = QtCore.QPointF(0, 0)
        self.grabCenter = False
        self.__maxDistance = 150
        self.color = QtGui.QColor(0,255,193)
        self.x = 0
        self.y = 0
        
    def paintEvent(self, event):
        painter = QtGui.QPainter(self)
        bounds = QtCore.QRectF(-self.__maxDistance, -self.__maxDistance, self.__maxDistance * 2, self.__maxDistance * 2).translated(self._center())
        painter.drawEllipse(bounds)
        painter.setBrush(QtGui.QBrush())
        painter.setPen(QtGui.QPen(self.color, 5, QtCore.Qt.SolidLine))
        painter.drawEllipse(self._centerEllipse())

    def _centerEllipse(self):
        return QtCore.QRectF(-20, -20, 40, 40).translated(QtCore.QPointF(self.width()/2 + self.x, self.height()/2 + self.y))

    def _center(self):
        return QtCore.QPointF(self.width()/2, self.height()/2)
    
    def setPos(self, x1, y1):
        self.x = x1
        self.y = y1
        #print(str(round(self.x, 2)) + " " + str(round(self.y, 2)))
        self.update()
    
    """def _boundJoystick(self, point):
        limitLine = QtCore.QLineF(self._center(), point)
        if (limitLine.length() > self.__maxDistance):
            limitLine.setLength(self.__maxDistance)
        return limitLine.p2()

    def joystickDirection(self):
        if not self.grabCenter:
            return 0
        normVector = QtCore.QLineF(self._center(), self.movingOffset)
        currentDistance = normVector.length()
        angle = normVector.angle()

        distance = min(currentDistance / self.__maxDistance, 1.0)
        if 45 <= angle < 135:
            return (Direction.Up, distance)
        elif 135 <= angle < 225:
            return (Direction.Left, distance)
        elif 225 <= angle < 315:
            return (Direction.Down, distance)
        return (Direction.Right, distance)

    def mouseReleaseEvent(self, event):
        self.grabCenter = False
        self.movingOffset = QtCore.QPointF(0, 0)
        self.update()

    def mouseMoveEvent(self, event):
        if self.grabCenter:
            print("Moving")
            self.movingOffset = self._boundJoystick(event.pos())
            self.update()
        print(self.joystickDirection())"""