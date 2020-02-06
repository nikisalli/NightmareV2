import numpy as np
import math
import time
import sys
from threading import Thread
from PyQt5.QtWidgets import QApplication, QWidget, QSlider, QVBoxLayout
from PyQt5 import QtCore, QtGui, uic

cx = 6.5
fm = 13
tb = 17

x = 0
y  = 10
z   = -10

app = QApplication([])
window = QWidget()
layout = QVBoxLayout()
sliderx = QSlider(QtCore.Qt.Horizontal)
slidery = QSlider(QtCore.Qt.Horizontal)
sliderz = QSlider(QtCore.Qt.Horizontal)

class program_exec(Thread):
    def __init__(self):
        Thread.__init__(self)
        self.daemon = True
        self.start()
    def run(self):
        while 1:
            if(x == 0):
                x=0.0001

            if(y == 0):
                y=0.0001

            if(z == 0):
                z=0.0001

            L1 = math.sqrt(x**2 + y**2)
            gama = - (np.arctan(x / y) / math.pi * 180)
            L = math.sqrt((L1 - cx)**2 + (-z)**2)
            beta = - (np.cos((tb**2 + fm**2 - L**2) / (2 * tb * fm)) / math.pi * 180) + 180
            alpha1 = np.cos(-z / L) / math.pi * 180
            alpha2 = np.cos((fm**2 + L**2 - tb**2) / (2 * fm * L)) / math.pi * 180
            alpha = - (alpha1 + alpha2) + 90
    
            L2 = cx + np.cos(math.radians(alpha))*fm + np.cos(math.radians((beta)+alpha))*tb
            x1 = np.sin(math.radians(-gama))*L2
            y1 = np.cos(math.radians(-gama))*L2
            z1 = -(np.sin(math.radians(alpha))*fm+np.sin(math.radians((beta)+alpha))*tb)
  
            print(str(x) ,str(x1), str(y), str(y1), str(z), str(z1), str(alpha), str(beta), str(gama))

class qtrun(Thread):
    def __init__(self):
        Thread.__init__(self)
        self.daemon = True
        self.start()
    def run(self):
        sliderx.minimum = -100
        sliderx.maximum = 100
        sliderx.setValue = 0

        slidery.minimum = -100
        slidery.maximum = 100
        slidery.setValue = 0

        sliderz.minimum = -100
        sliderz.maximum = 100
        sliderz.setValue = 0

        layout.addWidget(sliderx)
        layout.addWidget(slidery)
        layout.addWidget(sliderz)
        window.setLayout(layout)
        window.show()
        app.exec_()
    
def main(): 
    qtrun()
    program_exec()
    
if __name__ == '__main__':
    main()