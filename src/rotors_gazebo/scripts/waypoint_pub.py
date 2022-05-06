#!/usr/bin/env python
from random import random
from unittest.mock import seal
from matplotlib.transforms import Transform
import rospy
import numpy as np
import math
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from geometry_msgs.msg import Quaternion, Transform, Vector3
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import random


class Waypoints():
    def __init__(self) -> None:
        self.sub_ag = rospy.Subscriber("/pelican/odometry_sensor1/odometry", Odometry, self.odom_cb)
        self.pub_ag = rospy.Publisher('/pelican/command/trajectory', MultiDOFJointTrajectory, queue_size=1)
        self.sub_ob = rospy.Subscriber("/firefly/odometry_sensor1/odometry", Odometry, self.odom_hb_cb)
        self.pub_ob = rospy.Publisher('/firefly/command/trajectory', MultiDOFJointTrajectory, queue_size=1)
        self.pos = [0.0, 0.0, 0.0]
        self.pos_ob = [2.0, 0.0, 0.0]
        self.orientation = [0.0, 0.0, 0.0]
        self.or_ob = [0.0, 0.0, 0.0]
        self.rad = 3.0
        self.center = [2.0, 0.0]
        self.center_ob = [0.0, 10.0]
        self.theta = 3*np.pi/2
        self.theta_d = np.pi/20
        self.goal = [self.rad*np.cos(self.theta+self.theta_d)+self.center[0], self.rad*np.sin(self.theta+self.theta_d)+self.center[1], 2.0]
        self.theta = self.theta+self.theta_d
        self.goal_ob = [3.0, 0.0, 2.0]
        self.goal_dist = 1.0
        self.theta_ob = 3*np.pi/2
        self.rad = 3.0*(self.goal_dist*2-1)

    def odom_hb_cb(self, msg):
        self.pos_ob[0] = msg.pose.pose.position.x
        self.pos_ob[1] = msg.pose.pose.position.y
        self.pos_ob[2] = msg.pose.pose.position.z
        self.center[0] = msg.pose.pose.position.x
        self.center[1] = msg.pose.pose.position.y
        (r, p, y) = euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])        
        self.or_ob[0] = r
        self.or_ob[1] = p
        self.or_ob[2] = y
        #print("pose = ", self.pos_ob)


    def odom_cb(self, msg):
        (r, p, y) = euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])        
        self.orientation[0] = r
        self.orientation[1] = p
        self.orientation[2] = y
        self.pos[0] = msg.pose.pose.position.x
        self.pos[1] = msg.pose.pose.position.y
        self.pos[2] = msg.pose.pose.position.z
    
    def talker_ag(self):
        rate = rospy.Rate(10) # 10hz
        tm = MultiDOFJointTrajectory()
        tm.joint_names.append("base_link")
        #while not rospy.is_shutdown():
        pts = MultiDOFJointTrajectoryPoint()
        trf = Transform()
        if np.sqrt(np.sum((np.array(self.pos)-np.array(self.goal))**2))<(4.0):
            self.goal = [self.rad*np.cos(self.theta+self.theta_d)+self.center[0], self.rad*np.sin(self.theta+self.theta_d)+self.center[1], 4.0]
            self.theta = self.theta+self.theta_d
        trf.translation.x = self.goal[0]
        trf.translation.y = self.goal[1]
        trf.translation.z = self.goal[2] #+ (random.random()-0.5)*2
        if self.orientation[2]<0.0:
            self.orientation[2] = self.orientation[2] + 2*np.pi
        new_or = self.theta+np.pi
        if new_or>2*np.pi:
            new_or = new_or-2*np.pi

        #print(np.rad2deg(self.orientation[2]), np.rad2deg(new_or))
        pitch = np.arcsin(np.clip((self.pos[2]-self.pos_ob[2])/np.sqrt(np.sum((np.array(self.pos)-np.array(self.goal))**2)), -1.0, 1.0))
        #(x,y,z,w) = quaternion_from_euler(0, pitch, new_or)
        (x,y,z,w) = quaternion_from_euler(0.0, 0.0, new_or)
        trf.rotation.x = x
        trf.rotation.y = y
        trf.rotation.z = z
        trf.rotation.w = w
        pts.transforms.append(trf)
        tm.points.append(pts)
        self.pub_ag.publish(tm)
        rate.sleep()

    def talker_ob(self):
        rate = rospy.Rate(10) # 10hz
        tm = MultiDOFJointTrajectory()
        tm.joint_names.append("base_link")
        #while not rospy.is_shutdown():
        pts = MultiDOFJointTrajectoryPoint()
        trf = Transform()
        #if np.sqrt(np.sum((np.array(self.pos_ob)-np.array(self.goal_ob))**2))<0.9:
        #print("yoyouo")
        #self.goal_ob = [self.pos_ob[0]+self.goal_dist, 0.0, 2.0]
        if np.sqrt(np.sum((np.array(self.pos_ob)-np.array(self.goal_ob))**2))<(4.0):
            self.goal_ob = [10.0*np.cos(self.theta_ob+self.theta_d)+self.center_ob[0], 10.0*np.sin(self.theta_ob+self.theta_d)+self.center_ob[1], 2.0]
            self.theta_ob = self.theta_ob+self.theta_d

        #print("Goal = ", self.goal_ob)
        trf.translation.x = self.goal_ob[0]
        trf.translation.y = self.goal_ob[1]
        trf.translation.z = self.goal_ob[2]
        pts.transforms.append(trf)
        tm.points.append(pts)
        self.pub_ob.publish(tm)
        rate.sleep()




if __name__ == '__main__':
    try:
        rospy.init_node('waypoint_publisher', anonymous=True)
        obj = Waypoints()
        while not rospy.is_shutdown():
            obj.talker_ob()
            obj.talker_ag()
    except rospy.ROSInterruptException:
        pass