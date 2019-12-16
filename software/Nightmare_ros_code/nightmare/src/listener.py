#!/usr/bin/env python

import rospy
import serial
import numpy as np
import tf
import math
from std_msgs.msg import Float32
from threading import Thread
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
from tf.broadcaster import TransformBroadcaster

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

class nightmare_node():
    def __init__(self):
        self.degrees2rad = math.pi/180.0

        # Get values from launch file
        self.rate = rospy.get_param('~rate', 100.0)  # the rate at which to publish the transform
        # Static transform between sensor and fixed frame: x, y, z, roll, pitch, yaw
        # <rosparam param="static_transform">[0, 0, 0, 0, 0, 0]</rosparam>
        self.static_transform = rospy.get_param('~static_transform', [0, 0, 0, 0, 0, 0])
        self.topic_name = rospy.get_param('~topic_name', "/imu")
        self.fixed_frame = rospy.get_param('~fixed_frame', "world")
        self.frame_name = rospy.get_param('~frame_name', "imu")
        self.publish_transform = rospy.get_param('~publish_transform', False)

        self.pub_imu = rospy.Publisher("nightmare/imu", Imu, queue_size=1)
        self.odomBroadcaster_imu = TransformBroadcaster()

        self.imu_msg = Imu()
        #self.imu_msg.orientation_covariance[0] = -1
        #self.imu_msg.angular_velocity_covariance[0] = -1
        #self.imu_msg.linear_acceleration_covariance[0] = -1

        self.current_time = rospy.get_time()
        self.last_time = rospy.get_time()

        self.seq = 0

        rospy.on_shutdown(self.shutdown_node)
        rate = rospy.Rate(100) # 100hz

        rospy.loginfo("Ready for publishing imu")

        while not rospy.is_shutdown():
            self.current_time = rospy.get_time()

            if self.publish_transform:
                quaternion = tf.transformations.quaternion_from_euler(self.static_transform[3]*self.degrees2rad,
                                                                      self.static_transform[4]*self.degrees2rad,
                                                                      self.static_transform[5]*self.degrees2rad)

                self.odomBroadcaster_imu.sendTransform(
                    (self.static_transform[0], self.static_transform[1], self.static_transform[2]),
                    (quaternion[0], quaternion[1], quaternion[2], quaternion[3]),
                    rospy.Time.now(), self.frame_name, self.fixed_frame
                )
            
            self.publish_imu()
            rate.sleep()

    def publish_imu(self):
        self.imu_msg = Imu()

        self.imu_msg.orientation_covariance = [0.01 ,0    ,0
                                              ,0    ,0.01 ,0
                                              ,0    ,0    ,0.01]

        quaternion = tf.transformations.quaternion_from_euler(imu_listener.roll*self.degrees2rad,
                                                              imu_listener.pitch*self.degrees2rad,
                                                              imu_listener.yaw*self.degrees2rad)

        self.imu_msg.orientation.x = quaternion[0] # x
        self.imu_msg.orientation.y = quaternion[1] # y
        self.imu_msg.orientation.z = quaternion[2] # z
        self.imu_msg.orientation.w = quaternion[3] # w

        self.imu_msg.header.stamp = rospy.Time.now()
        self.imu_msg.header.frame_id = self.frame_name
        self.imu_msg.header.seq = self.seq

        self.pub_imu.publish(self.imu_msg)
        self.seq += 1

    def shutdown_node(self):
        rospy.loginfo("Turning off node: robot_imu_publisher")

if __name__ == '__main__':
    robot_listener()
    imu_listener()

    rospy.loginfo('Starting RobotImuPublisherNode')
    rospy.init_node('sensor_imu_publisher')

    try:
        obj_temp = nightmare_node()
    except rospy.ROSInterruptException:
        pass