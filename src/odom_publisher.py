#! /usr/bin/env python3

import rospy
import math
import numpy as np
import tf2_ros
import tf_conversions


from std_msgs.msg import Float32, Float64, Bool, Empty
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TransformStamped
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import Imu
from vesc_msgs.msg import VescStateStamped


class ODOM_Interface:
    def __init__(self):
        self.time_out = 0.0
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.roll_rate = 0.0
        self.pitch_rate = 0.0
        self.yaw_rate = 0.0
        self.dt = 0.05
        self.quat = (0,0,0,1)
        self.speed = 0.0
        self.encoder_constant=rospy.get_param('~encoder_constant',3092.53)


        self.qtm_sub = rospy.Subscriber("/sensors/core",VescStateStamped,self.speedCallback) # TODO: potentially modify
        self.imu_sub = rospy.Subscriber("imu",Imu,self.IMUCallback) # TODO: specify which topic the IMU is on
        self.reset_sub = rospy.Subscriber("odom_reset",Empty,self.resetCallBack)
        self.odom_broadcaster = tf2_ros.TransformBroadcaster()
        self.odom_pub = rospy.Publisher('odom',Odometry,queue_size=1)

        self.odom_timer = rospy.Timer(rospy.Duration(self.dt),self.odomTFCallback)


    def resetCallBack(self,msg):
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.speed = 0.0
        self.yaw_rate = 0.0

    def speedCallback(self,msg):
        self.speed = msg.state.speed/self.encoder_constant # v=omega*r, r=0.05 wheel radius
        #speed = Float64()
        #speed.data = speedavg*0.05 # v=omega*r, r=0.05 wheel radius

        # publish ackermann message
        # self.speed_pub.publish(speed)

    def IMUCallback(self,msg):
        self.roll_rate = msg.angular_velocity.x
        self.pitch_rate =msg.angular_velocity.y
        self.yaw_rate = msg.angular_velocity.z
        oriQuat = msg.orientation
        self.quat = (oriQuat.x, oriQuat.y, oriQuat.z, oriQuat.w)
        roll, pitch, yaw = tf_conversions.transformations.euler_from_quaternion(self.quat)
        self.yaw = yaw

    def odomTFCallback(self,msg):
        self.x = self.x + self.speed*math.cos(self.yaw)*self.dt
        self.y = self.y + self.speed*math.sin(self.yaw)*self.dt
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = self.quat[0]
        t.transform.rotation.y = self.quat[1]
        t.transform.rotation.z = self.quat[2]
        t.transform.rotation.w = self.quat[3]
        self.odom_broadcaster.sendTransform(t)

        Od = Odometry()
        Od.header.stamp = rospy.Time.now()
        Od.header.frame_id = "odom"
        Od.child_frame_id = "base_link"
        Od.pose.pose.position.x = self.x
        Od.pose.pose.position.y = self.y
        Od.pose.pose.position.z = 0.0
        Od.pose.pose.orientation.x = self.quat[0]
        Od.pose.pose.orientation.y = self.quat[1]
        Od.pose.pose.orientation.z = self.quat[2]
        Od.pose.pose.orientation.w = self.quat[3]
        Od.twist.twist.linear.x = self.speed
        Od.twist.twist.angular.x = self.roll_rate
        Od.twist.twist.angular.y = self.pitch_rate
        Od.twist.twist.angular.z = self.yaw_rate
        self.odom_pub.publish(Od)
        print(Od)


#        print("sent odom")

if __name__ == '__main__':
    rospy.init_node('Odom_publisher')

    myCar = ODOM_Interface()

    rospy.spin()
