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

from std_msgs.msg import Float32, Float32MultiArray
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
        self.time_out = rospy.get_time()
        self.wheel_timer = rospy.get_time()
        self.dt = 0
        self.time = rospy.get_time()
        self.old_time = rospy.get_time()

        self.madgwick = Madgwick()
        self.quat = np.zeros((1, 4))      # Allocation of quaternions
        self.quat = [1.0, 0.0, 0.0, 0.0]


        self.send_timer = rospy.Timer(rospy.Duration(0.02), self.mainLoop)


    def mainLoop(self,msg):
        accel_data, gyro_data, mag_data = self.imu.getMotion9()
        self.time = rospy.get_time()
        self.dt = self.time - self.old_time

        self.madgwick.Dt = self.dt
        self.quat = self.madgwick.updateMARG(self.quat, gyr=gyro_data, acc=accel_data,mag=mag_data)
        q = (self.quat[1],self.quat[2],self.quat[3],self.quat[0])
        euler = tf.transformations.euler_from_quaternion(q)


        imu_msg = Imu()
        imu_msg.linear_acceleration.x = accel_data[0]
        imu_msg.linear_acceleration.y = accel_data[1]
        imu_msg.linear_acceleration.z = accel_data[2]

        imu_msg.angular_velocity.x = gyro_data[0]
        imu_msg.angular_velocity.y = gyro_data[1]
        imu_msg.angular_velocity.z = gyro_data[2]

        imu_msg.orientation.w = self.quat[0]
        imu_msg.orientation.x = self.quat[1]
        imu_msg.orientation.y = self.quat[2]
        imu_msg.orientation.z = self.quat[3]

        self.imu_pub.publish(imu_msg)
        print(euler)
        self.old_time = self.time


if __name__ == '__main__':
    rospy.init_node('rover_sensors_node')

    mySensors = roverSensors()

    rospy.spin()
