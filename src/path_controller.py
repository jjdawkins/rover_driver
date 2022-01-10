#! /usr/bin/env python3

import rospy
import math
import numpy as np

from ackermann_msgs.msg import *
from std_msgs.msg import Empty, String, Header, Float64
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist, Quaternion, Pose
#from tf.transformations import euler_from_quaternion, quaternion_from_euler
import tf
#from tf_conversions import quaternion_to_euler


class roverPathControl:
    def __init__(self):

        self.ackMsg = AckermannDriveStamped()
        self.twistMsg = Twist()
        self.yawRate = 0.0 # u2 variable
        self.xdes = 2.0
        self.ydes = 2.0
        self.pose = np.zeros([1,3])
        self.xpos = 0.0
        self.ypos = 0.0
        self.xerr = 0.0
        self.yerr = 0.0
        self.yaw = 0.0
        self.speed = 0.0
        self.dist = 0.0
        self.spdErrInt = 0
        self.path = []
        self.wp_n = 0
        self.wp_ind = 0
        self.yaw_des = 0
        self.xvel_b = 0
        self.yvel_b = 0
        self.xvel_g = 0
        self.yvel_g = 0
        self.mission = False
        self.path_track = 0
        self.dt = 0.1
        self.s = 0

        self.max_str_angle = rospy.get_param('max_str_angle',0.75)
        self.max_thr = rospy.get_param('max_throttle',0.2)
        self.cruise_spd = rospy.get_param('cruise_speed',1.0)
        self.dist_threshold = rospy.get_param('waypoint_distance_threshold',0.4) # default to 10 cm
        self.L = rospy.get_param('wheel_base',0.2) # 0.2 meters
        self.KpSpd = rospy.get_param('Kp_spd',1.0)
        self.KiSpd = rospy.get_param('Ki_spd',0.05)
        self.KpYaw = rospy.get_param('Kp_yaw',1.7)
        self.K_ct = rospy.get_param('Kct',2.0)

        self.acker_pub = rospy.Publisher('acker_cmd',AckermannDriveStamped,queue_size=5)
        self.wp_sub = rospy.Subscriber('waypoint',Pose,self.wptCallBack)
        self.path_sub = rospy.Subscriber('path',Path,self.pathCallBack)
        self.qtm_sub = rospy.Subscriber('odom',Odometry,self.odomCallBack)
        self.set_spd_sub = rospy.Subscriber('set_speed',Float64,self.setSpeedCallBack)


        self.send_timer = rospy.Timer(rospy.Duration(self.dt), self.sendCallBack)

    def setSpeedCallBack(self,msg):
        if(abs(msg.data) > self.max_spd):
            self.cruise_spd = self.max_spd
        else:
            self.cruise_spd = abs(msg.data)

    def wptCallBack(self,msg):
        self.xdes = msg.position.x
        self.ydes = msg.position.y
        self.mission = True

    def pathCallBack(self,msg):

        #print("%d",len(msg.poses))
        self.wp_n = len(msg.poses)
        self.path = np.zeros([self.wp_n,3])
        for i in range(self.wp_n):

            orientation_q = msg.poses[i].pose.orientation # extract quaternion from pose message
            orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w] # put quaternion elements into an array
            euler = tf.transformations.euler_from_quaternion(orientation_list)
            self.yaw = euler[2]
            self.path[i,0] = msg.poses[i].pose.position.x
            self.path[i,1] = msg.poses[i].pose.position.y
            self.path[i,2] = euler[2]

        dx = np.diff(self.path[:,0])
        dy = np.diff(self.path[:,1])
        r = np.sqrt(dx*dx + dy*dy)


        self.path_s = np.cumsum(r)
        self.path_l = self.path_s[np.size(r)-1]
        print(self.path_l)
        self.mission = True


    def findClosestPoint(self,path_pose,pose):
        diff = path_pose - pose
        rng = np.sqrt(diff[:,0]*diff[:,0]+diff[:,1]*diff[:,1])
        index = np.argmin(rng)
        return index


    def findPathLocation(self,path_s,car_s):
        diff = path_s - car_s
        index = np.argmin(np.abs(diff))
        return index


    def odomCallBack(self,msg):
        orientation_q = msg.pose.pose.orientation # extract quaternion from pose message
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w] # put quaternion elements into an array
        euler = tf.transformations.euler_from_quaternion(orientation_list)
        self.yaw = euler[2]

        #(roll, pitch, yaw) = euler_from_quaternion (orientation_list) # convert quaternion to Euler angles
        self.xvel_g = msg.twist.twist.linear.x # parse x-component of inertial speed from odometry message
        self.yvel_g = msg.twist.twist.linear.y # parse y-component of inertial speed from odometry message

        self.xvel_b = math.cos(self.yaw)*self.xvel_g + math.sin(self.yaw)*self.yvel_g
        self.yvel_b = -math.sin(self.yaw)*self.xvel_g + math.cos(self.yaw)*self.yvel_g

        #self.speed = math.sqrt(speedx*speedx + speedy*speedy)
        self.pose[0,0] = msg.pose.pose.position.x
        self.pose[0,1] = msg.pose.pose.position.y
        self.pose[0,2] = self.yaw
        self.xpos = msg.pose.pose.position.x # parse x-component of position from odometry message
        self.ypos = msg.pose.pose.position.y # parse y-component of position from odometry message

    def wrapToPi(self,ang):
        while(ang > math.pi):
            ang = ang - 2*math.pi

        while(ang < -math.pi):
            ang = ang + 2*math.pi

        return ang

    def sendCallBack(self,msg):
        # Steering controller (using yaw rate speed kinematics)

        if(self.mission):

            if(self.s > self.path_l): # If we reach the end of the path start back at zero
                self.s = 0

            self.s = self.s + self.cruise_spd*self.dt
            ind = self.findPathLocation(self.path_s, self.s)
            #ind = self.findClosestPoint(self.path,self.pose)


            steerAng = Float64()

            #if(self.wp_ind == self.wp_n): # If we reached the last waypoint start over
            #    self.wp_ind = 0

            self.xdes = self.path[ind,0]
            self.ydes = self.path[ind,1]
            self.yaw_des = self.path[ind,2]
            R = 2
            C = 2*math.pi*R
            T = C/self.cruise_spd

            #rospy.loginfo("des_pos %f,%f,%f",self.xdes,self.ydes,self.yaw_des)

            #self.xdes = 4*math.cos(2*math.pi*rospy.get_time()/T)
            #self.ydes = R*math.sin(2*math.pi*rospy.get_time()/T) + 4
            #self.yaw_des = 2*math.pi*rospy.get_time()/T


            self.xerr = self.xdes-(self.pose[0,0]+0*(self.L/2)*math.cos(self.yaw))
            self.yerr = self.ydes-(self.pose[0,1]+0*(self.L/2)*math.sin(self.yaw))

# Rotate Global Error Vector into the Body Frame
            self.xerr_b = math.cos(self.yaw)*self.xerr + math.sin(self.yaw)*self.yerr
            self.yerr_b = -math.sin(self.yaw)*self.xerr + math.cos(self.yaw)*self.yerr

            #Stanley Controller
            #yaw_error = (self.yaw-self.yaw_des)
            yaw_error = self.wrapToPi(self.yaw_des-self.yaw)


            if abs(self.xvel_b)<0.1: # if speed less than 10 cm/s, use speed invariant turn angle
                str_ang = yaw_error
            else: # if speed greater than 10 cm/s, use speed dependent steering angle
                str_ang = self.KpYaw*yaw_error + math.atan(self.K_ct*self.yerr_b/self.xvel_b)

            if abs(str_ang)>self.max_str_angle:
                str_ang = math.copysign(self.max_str_angle,str_ang)


            self.desSpeed = self.cruise_spd
            # speed controller (Using PI controller on acceleration)
            self.spdErr = self.desSpeed - self.xvel_b

            # update speed error integral term
            self.spdErrInt = self.spdErrInt + self.spdErr*0.1 # 10 Hz sampling rate
            # saturate speed error integral term at 2
            #if self.spdErrInt > 2:
            #    self.spdErrInt = 2.0
            if(self.spdErrInt > 0.05):
                self.spdErrInt = 0.05
            elif (self.spdErrInt < -0.05):
                self.spdErrInt = -0.05
            else:
                self.spdErrInt = self.spdErrInt;

            # PWM duty cycle for throttle control
            self.thr_cmd = self.KpSpd*self.spdErr + self.KiSpd*self.spdErrInt # PI controller

            # Saturate speed command at 20% duty cycle
            if self.thr_cmd > self.max_thr:
                self.thr_cmd = self.max_thr

            if self.thr_cmd < -self.max_thr:
                self.thr_cmd = -self.max_thr


            # package ackermann message
            self.ackMsg.drive.steering_angle = str_ang
            self.ackMsg.drive.acceleration = self.thr_cmd

            rospy.loginfo("X_err %f CT_err %f Psi_Err %f",self.xerr_b, self.yerr_b,yaw_error)
            # publish ackermann message
            self.acker_pub.publish(self.ackMsg)
        else:
            rospy.loginfo("Waiting for Mission")
            rospy.sleep(1)


if __name__ == '__main__':
    rospy.init_node('path_controller')

    myRover = roverPathControl()

    rospy.spin()
