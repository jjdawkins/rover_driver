#! /usr/bin/env python3

import time
import rospy
import sys
import time
import math
import numpy as np
import spidev

import argparse
import sys
import navio.mpu9250
import navio.util

import RPi.GPIO as GPIO

from std_msgs.msg import Float32, Float32MultiArray
from sensor_msgs.msg import Imu

navio.util.check_apm()

class roverSensors():
    def __init__(self):

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(18, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.add_event_detect(18, GPIO.FALLING, callback=self.wheelTickCallBack, bouncetime=2)
        self.str_pub = rospy.Publisher("odometry",Float32MultiArray,queue_size=10)
        self.spd_pub = rospy.Publisher("wheel_spd",Float32,queue_size=10)
        #self.imu_sub = rospy.Subscriber("/mavros/imu/data",Imu,self.imuCallBack)
        #self.rc_sub = rospy.Subscriber("/mavros/rc/in",RCIn,self.rcCallBack)
        self.wheel_speed = 0
        self.time_out = rospy.get_time()
        self.wheel_timer = rospy.get_time()
        self.dt=100
        self.ax = 0
        self.ay = 0
        self.gz = 0
        self.rc_str = 0
        self.rc_thr = 0
        self.send_timer = rospy.Timer(rospy.Duration(0.04), self.sendCallBack)


    def rcCallBack(self,msg):
        self.rc_str = msg.channels[0]
        self.rc_thr = msg.channels[1]

    def imuCallBack(self,msg):
        self.ax = msg.linear_acceleration.x
        self.ay = msg.linear_acceleration.y
        self.gz = msg.angular_velocity.z

    def wheelTickCallBack(self,channel):
        self.dt = rospy.get_time()-self.wheel_timer
        print(" in Ts %f",1/self.dt)
        self.wheel_timer=rospy.get_time()

    def sendCallBack(self,msg):
        str_msg = Float32MultiArray()

        if(rospy.get_time()-self.wheel_timer > 0.2):
            self.wheel_speed = 0
        else:
            curr_ws = 1/self.dt
            if(curr_ws<200):
                self.wheel_speed = curr_ws
            else:
                self.wheel_speed = self.wheel_speed


        #str_msg.data.append(self.rc_str)
        #str_msg.data.append(self.vl53.range)
        #str_msg.data.append(self.rc_str)
        #str_msg.data.append(self.wheel_speed)
        #str_msg.data.append(self.ax)
        #str_msg.data.append(self.ay)
        #str_msg.data.append(self.gz)

        #print("Range: {0}mm".format(vl53.range))
        #str_cmd = self.str_ang + 1.500
        spd_msg = Float32()
        spd_msg.data = self.wheel_speed
        self.spd_pub.publish(spd_msg)
        #self.str_pub.publish(str_msg)


if __name__ == '__main__':
    rospy.init_node('rover_sensors_node')

    myOdom = roverSensors()

    rospy.spin()
