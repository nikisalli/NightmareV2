#!/usr/bin/env python

import rospy
import serial
from std_msgs.msg import Float32
from threading import Thread

robot = serial.Serial('/dev/ttyUSB0', 460800)
imu = serial.Serial('/dev/ttyACM0', 115200)

robot.isOpen()

def fmap(x, in_min, in_max, out_min, out_max):
  return (float)(x - in_min) * (out_max - out_min) / (float)(in_max - in_min) + out_min

class robot_listener(Thread):
    def __init__(self):
        Thread.__init__(self)
        self.daemon = True
        self.start()
    def run(self):
        while 1:
            if(ord(robot.read()) == 0xAA):
                robot.read()
                bytes = []
                for _ in range(2):
                    bytes.append(ord(robot.read()))
                robot_listener.voltage = fmap(bytes[0],0,255,5,10)
                robot_listener.current = fmap(bytes[1],0,255,0,15)

class imu_listener(Thread):
    def __init__(self):
        Thread.__init__(self)
        self.daemon = True
        self.start()
    def run(self):
        while 1:
            if(ord(imu.read()) == 0xBB):
                imu.read()
                bytes = []
                for _ in range(3):
                    bytes.append(ord(imu.read()))
                imu_listener.roll = fmap(bytes[0],0,255,-180,180)
                imu_listener.pitch = fmap(bytes[1],0,255,-180,180)
                imu_listener.yaw = fmap(bytes[2],0,255,-180,180)

def nightmare_node():
    bvol = rospy.Publisher('nightmare/batt_voltage', Float32, queue_size=10)
    bcur = rospy.Publisher('nightmare/batt_current', Float32, queue_size=10)
    nroll = rospy.Publisher('nightmare/body_roll', Float32, queue_size=10)
    npitch = rospy.Publisher('nightmare/body_pitch', Float32, queue_size=10)
    nyaw = rospy.Publisher('nightmare/body_yaw', Float32, queue_size=10)

    rospy.init_node('nightmare', anonymous=True)

    rate = rospy.Rate(100) # 100hz

    while not rospy.is_shutdown():

        #rospy.loginfo(imu_listener.roll)
        #rospy.loginfo(imu_listener.pitch)
        #rospy.loginfo(imu_listener.yaw)

        bvol.publish(robot_listener.voltage)
        bcur.publish(robot_listener.current)
        nroll.publish(imu_listener.roll)
        npitch.publish(imu_listener.pitch)
        nyaw.publish(imu_listener.yaw)

        rate.sleep()

if __name__ == '__main__':
    try:
        robot_listener()
        imu_listener()
        nightmare_node()
    except rospy.ROSInterruptException:
        pass