#!/usr/bin/env python
# license removed for brevity
import rospy
import serial
from std_msgs.msg import Float32

robot = serial.Serial('/dev/ttyUSB0', 460800)
robot.isOpen()

def fmap(x, in_min, in_max, out_min, out_max):
  return (float)(x - in_min) * (out_max - out_min) / (float)(in_max - in_min) + out_min

def talker():
    bvol = rospy.Publisher('battery_voltage', Float32, queue_size=10)
    bcur = rospy.Publisher('battery_current', Float32, queue_size=10)
    rospy.init_node('nightmare', anonymous=True)
    rate = rospy.Rate(20) # 20hz
    while not rospy.is_shutdown():
        if(ord(robot.read()) == 0xAA):
            robot.read()
            bytes = []
            for i in range(2):
                bytes.append(ord(robot.read()))
            voltage = fmap(bytes[0],0,255,5,10)
            current = fmap(bytes[1],0,255,0,15)
        rospy.loginfo(voltage)
        rospy.loginfo(current)
        bvol.publish(voltage)
        bcur.publish(current)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass