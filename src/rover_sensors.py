#! /usr/bin/env python3

import rospy
import time

import sys
import time
import math
import numpy as np
import spidev
import tf

import argparse
import sys
import navio.mpu9250
import navio.util
from ahrs.filters import Madgwick

from std_msgs.msg import Float32
from geometry_msgs.msg import Vector3Stamped
from sensor_msgs.msg import Imu

navio.util.check_apm()

class roverSensors():
    def __init__(self):

        self.imu = navio.mpu9250.MPU9250()
        if self.imu.testConnection():
            print("Connection established: True")
        else:
            sys.exit("Connection established: False")

        self.imu.initialize()

        self.imu_pub = rospy.Publisher("imu",Imu,queue_size=10)
        self.eul_pub = rospy.Publisher("euler",Vector3Stamped,queue_size=10)
        self.time_out = rospy.get_time()
        self.wheel_timer = rospy.get_time()
        self.dt = 0
        self.time = rospy.get_time()
        self.old_time = rospy.get_time()

        self.madgwick = Madgwick()
        #self.quat = np.zeros((1, 4))      # Allocation of quaternions
        self.quat = np.array([1.0, 0.0, 0.0, 0.0])
        self.accel = np.array([1,3])
        self.gyro = np.array([1,3])
        self.mag = np.array([1,3])

        self.send_timer = rospy.Timer(rospy.Duration(0.02), self.mainLoop)


    def mainLoop(self,msg):
        accel_data, gyro_data, mag_data = self.imu.getMotion9()
        now = rospy.get_rostime()
        self.time = rospy.get_time()
        self.dt = self.time - self.old_time

        self.accel = np.array([accel_data[1],-accel_data[0],accel_data[2]])
        self.gyro = np.array([gyro_data[1],-gyro_data[0],gyro_data[2]])
        self.mag = np.array([mag_data[1],-mag_data[0],mag_data[2]])

        self.madgwick.Dt = self.dt
        self.quat = self.madgwick.updateMARG(self.quat, gyr=self.gyro, acc=self.accel,mag=self.mag)
        q = (self.quat[1],self.quat[2],self.quat[3],self.quat[0])
        euler = tf.transformations.euler_from_quaternion(q)

        eul_msg = Vector3Stamped()
        eul_msg.header.frame_id = 'imu_link'
        eul_msg.header.stamp.secs= now.secs
        eul_msg.header.stamp.nsecs = now.nsecs
        eul_msg.vector.x = euler[0]
        eul_msg.vector.y = euler[1]
        eul_msg.vector.z = euler[2]
        self.eul_pub.publish(eul_msg)
        # todo: Transform data from x-right y-forward z-up # TODO: to x-forward- y-left z-up
        # should be a 90degree rotation about z
        imu_msg = Imu()
        imu_msg.header.frame_id = 'imu_link'
        imu_msg.header.stamp.secs = now.secs
        imu_msg.header.stamp.nsecs = now.nsecs
        imu_msg.linear_acceleration.x = self.accel[0]
        imu_msg.linear_acceleration.y = self.accel[1]
        imu_msg.linear_acceleration.z = self.accel[2]

        imu_msg.angular_velocity.x = self.gyro[0]
        imu_msg.angular_velocity.y = self.gyro[1]
        imu_msg.angular_velocity.z = self.gyro[2]

        imu_msg.orientation.w = self.quat[0]
        imu_msg.orientation.x = self.quat[1]
        imu_msg.orientation.y = self.quat[2]
        imu_msg.orientation.z = self.quat[3]

        self.imu_pub.publish(imu_msg)

        self.old_time = self.time


if __name__ == '__main__':
    rospy.init_node('rover_sensors_node')

    mySensors = roverSensors()

    rospy.spin()
