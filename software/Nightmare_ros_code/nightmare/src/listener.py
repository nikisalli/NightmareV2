#!/usr/bin/env python
# license removed for brevity
import rospy
import serial
from std_msgs.msg import Float32

robot = serial.Serial('/dev/ttyUSB0', 460800, timeout=0.1)

def talker():
    pub = rospy.Publisher('battery_voltage', Float32, queue_size=10)
    rospy.init_node('nightmare', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        if(robot.read() == 0x55):
            robot.read()
            bytes = []
            for i in range(1):
                bytes[i] = robot.read()
            voltage = bytes[1] / 10.0
            rospy.loginfo(voltage)
            pub.publish(voltage)
            rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass