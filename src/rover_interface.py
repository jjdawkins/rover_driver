#! /usr/bin/env python3

import rospy
import sys
import time
import navio.pwm
import navio.util
import navio.leds

import math
import numpy as np

from ackermann_msgs.msg import *
from std_msgs.msg import Empty, String, Header, Float64
from sensor_msgs.msg import Joy

STR_PWM_CH = 0
THR_PWM_CH = 2
SERVO_MIN = 1.250 #ms
SERVO_MAX = 1.750 #ms
navio.util.check_apm()

class RoverInterface():

    def __init__(self):

        self.str_pwm = navio.pwm.PWM(0)
        self.str_pwm.initialize()
        self.str_pwm.set_period(50)
        self.str_pwm.enable()

        self.thr_pwm = navio.pwm.PWM(2)
        self.thr_pwm.initialize()
        self.thr_pwm.set_period(50)
        self.thr_pwm.enable()

        self.led = navio.leds.Led()

        self.max_str_angle = rospy.get_param('max_str_angle',0.5)
        self.max_thr = rospy.get_param('max_thr_cmd',0.2)
        #self.min_thr = rospy.get_param('min_thr_cmd',-0.2)
        self.time_out = 0
        self.auto_mode = False
        self.armed = False
        self.str_cmd = 0
        self.thr_cmd = 0
        self.spd_cmd = 0

        #self.str_trim = rospy.get_param('str_trim',)
        self.max_speed = rospy.get_param('max_speed',2)

        self.acker_sub = rospy.Subscriber('acker_cmd',AckermannDriveStamped,self.ackerCallBack)
        self.joy_sub = rospy.Subscriber('/joy',Joy,self.joyCallBack)

        # self.vesc_spd_pub = rospy.Publisher()
        # self.vesc_pwm_pub = rospy.Publisher()

    def ackerCallBack(self,msg):
        rospy.loginfo("acker_Callback")
        if(self.auto_mode):
            self.str_cmd = msg.drive.steering_angle/self.max_str_angle

            self.spd_cmd = msg.drive.speed
            #rospy.loginfo("Str_PWM: %f, Thr_PWM: %f",str_cmd,thr_cmd)

        self.time_out = rospy.get_time();

    def joyCallBack(self,msg):
        #rospy.loginfo("Joy_Callback")

        if(msg.buttons[0]): # A button
            self.armed = not self.armed
            rospy.loginfo("Armed %d",self.armed)

        if(msg.buttons[2]): # X button
            self.auto_mode = False
            rospy.loginfo("Auto Mode Disable")


        if(msg.buttons[3]): # Y button on game pad
            self.auto_mode = True
            rospy.loginfo("Auto Mode Enable")

        self.thr_cmd = msg.axes[1]
        self.str_cmd = msg.axes[3]



    def sendCommand(self):
        thr_cmd = 0
        spd_cmd = 0
        str_cmd = 0
        #rospy.loginfo("%f",(rospy.get_time()-self.time_out))

        if(self.armed):
            self.led.setColor('Green')
        elif(self.armed and self.auto):
            self.led.setColor('Yellow')
        else:
            self.led.setColor('Red')

        if(self.auto_mode):
            if((rospy.get_time()-self.time_out) < 0.3):
                spd_cmd = self.spd_cmd
            else:
                spd_cmd = 0
        else:
            thr_cmd = self.thr_cmd

        str_cmd = self.str_cmd*0.4 + 1.500
        #str_cmd = 0.200*math.sin(rospy.get_time()) + 1.500
        #print("Str_PWM: %f, Thr_PWM: %f",str_cmd,thr_cmd)
        self.str_pwm.set_duty_cycle(str_cmd)

        if(self.armed):
            # Change here to publish to VESC control topic (PWM or speed if PWM is not compatible)
            self.thr_pwm.set_duty_cycle(thr_cmd)
        elif(self.armed and self.auto):
            # ADD CODE HERE TO PUBLISH TO VESC SPEED CONTROL TOPIC
        else:
            # change here to publish a speed of zero to the vesc speed topic
            self.thr_pwm.set_duty_cycle(1.5)



if __name__ == '__main__':
    rospy.init_node('Rover_Interface_Node')

    my_rover = RoverInterface()
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():

        #str_cmd = 0.200*math.sin(rospy.get_time()) + 1.500
        my_rover.sendCommand()
        rate.sleep()

    #send_timer = rospy.Timer(rospy.Duration(0.05), sendCallBack)

    #rospy.spin()
