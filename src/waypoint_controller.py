#! /usr/bin/env python3

import rospy
import math
import numpy as np

from ackermann_msgs.msg import *
from std_msgs.msg import Empty, String, Header, Float64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Quaternion, Pose
#from tf.transformations import euler_from_quaternion, quaternion_from_euler
#import tf2
#from tf_conversions import quaternion_to_euler


class roverWaypointControl:
    def __init__(self):

        self.max_str_angle = rospy.get_param('max_str_angle',0.6)
        self.max_speed = rospy.get_param('max_speed',2)
        self.desSpeed = rospy.get_param('desired_speed',0.2)
        self.dist_threshold = rospy.get_param('waypoint_distance_threshold',0.3) # default to 10 cm
        self.L = rospy.get_param('wheel_base',0.2) # 0.2 meters
        self.KpSpd = rospy.get_param('Kp_spd',1)
        self.KiSpd = rospy.get_param('Ki_spd',0.05)
        self.KpYaw = rospy.get_param('Kp_yaw',0.5)

        self.acker_pub = rospy.Publisher('acker_cmd',AckermannDriveStamped,queue_size=5)
        self.wp_sub = rospy.Subscriber('waypoint',Pose,self.desPoseCallback)
        self.path_sub = rospy.Subscriber('path',Path,self.pathCallback)
        self.qtm_sub = rospy.Subscriber('odom',Odometry,self.odomCallback)
        self.send_timer = rospy.Timer(rospy.Duration(0.1), self.sendCallBack)

        self.ackMsg = AckermannDriveStamped()
        self.twistMsg = Twist()
        self.yawRate = 0.0 # u2 variable
        self.xdes = 2.0
        self.ydes = 2.0
        self.xpos = 0.0
        self.ypos = 0.0
        self.theta = 0.0
        self.speed = 0.0
        self.dist = 0.0
        self.spdErrInt = 0
        self.wp_list = []
        self.wp_ind = 0

    def wptCallback(self,msg):
        self.xdes = msg.position.x
        self.ydes = msg.position.y

    def pathCallback(self,msg):
        self.xdes = msg.position.x # Modify for path
        self.ydes = msg.position.y

    def odomCallback(self,msg):
        orientation_q = msg.pose.pose.orientation # extract quaternion from pose message
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w] # put quaternion elements into an array
        euler = tf.transformations.euler_from_quaternion(orientation_list)
        #(roll, pitch, yaw) = euler_from_quaternion (orientation_list) # convert quaternion to Euler angles
        speedx = msg.twist.twist.linear.x # parse x-component of inertial speed from odometry message
        speedy = msg.twist.twist.linear.y # parse y-component of inertial speed from odometry message
        self.speed = math.sqrt(speedx*speedx + speedy*speedy)
        self.xpos = msg.pose.pose.position.x # parse x-component of position from odometry message
        self.ypos = msg.pose.pose.position.y # parse y-component of position from odometry message
        self.theta = euler[2]

    def sendCallBack(self,msg):
        # Steering controller (using yaw rate speed kinematics)
        q = [0,0,0,1]
        euler = tf.transformations.euler_from_quaternion(q)

        steerAng = Float64()
        x_err = self.xdes-self.xpos
        y_err = self.ydes-self.ypos
        self.dist = math.sqrt(x_err*x_err + y_err*y_err)
        # calculate how far I am from waypoint
        #self.dist = math.sqrt((self.xdes-self.xpos)**2 + (self.ydes-self.ypos)**2)
        if(self.dist < self.dist_threshold):
            wp_ind = wp_ind + 1
            desired_speed = 0

        theta_des = math.atan2(y_err,x_err)
        theta_error = (theta_des-self.theta)
        if theta_error>math.pi:
            theta_error = theta_error-2*math.pi
        elif theta_error<-math.pi:
            theta_error = theta_error+2*math.pi

        self.yawRate = self.KpYaw*theta_error
        if self.speed<0.1: # if speed less than 10 cm/s, use speed invariant turn angle
            steerAng = math.atan(self.yawRate*self.L/0.1)
        else: # if speed greater than 10 cm/s, use speed dependent steering angle
            steerAng = math.atan(self.yawRate*self.L/self.speed)

        if abs(steerAng)>self.max_str_angle:
            steerAng = math.copysign(self.max_str_angle,steerAng)



        # if vehicle is within distance threshold, stop
        if self.dist<self.dist_threshold:
            self.thr_cmd = 0.0 # stop if within threshold of waypoint


        # speed controller (Using PI controller on acceleration)
        self.spdErr = self.desSpeed - self.speed

        # update speed error integral term
        self.spdErrInt = self.spdErrInt + self.spdErr*0.1 # 10 Hz sampling rate
        # saturate speed error integral term at 2
        #if self.spdErrInt > 2:
        #    self.spdErrInt = 2.0
        if(self.spdErrInt > 0.2):
            self.spdErrInt = 0.2
        elif (self.spdErrInt < -0.2):
            self.spdErrInt = -0.2
        else:
            self.spdErrInt = self.spdErrInt;

        # PWM duty cycle for throttle control
        self.thr_cmd = self.KpSpd*self.spdErr + self.KiSpd*self.spdErrInt # PI controller

        # Saturate speed command at 20% duty cycle
        if self.thr_cmd > 0.2:
            self.thr_cmd = 0.2
        elif self.thr_cmd < -0.2:
            self.thr_cmd = -0.2

        # package ackermann message
        self.ackMsg.drive.steering_angle = steerAng
        self.ackMsg.drive.acceleration = 0*self.thr_cmd

        rospy.loginfo("%f,%f,%f,%f",self.speed,self.dist,self.thr_cmd,self.theta)
        # publish ackermann message
        self.acker_pub.publish(self.ackMsg)



if __name__ == '__main__':
    rospy.init_node('waypoint_controller')

    myRover = roverWaypointControl()

    rospy.spin()
