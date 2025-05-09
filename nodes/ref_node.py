#!/usr/bin/env python

import rospy
import numpy as np
from nav_msgs.msg import Odometry
import math
import time

class Ref_Wrapper():
    def __init__(self):

        # send real_wp to simulator
        self.reference_pub = rospy.Publisher("test_reference", Odometry, queue_size= 1)
        self.time = 0
        self.reference_rate = rospy.Rate(50)
        task = rospy.get_param("~task")
        self.task = task

        while True:
            self.run_reference()
            self.reference_rate.sleep()

    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
        qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2)
        qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2)
        qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)

        return np.array([qw, qx, qy, qz])

    def run_reference(self):
        
        self.time = self.time + (1 / self.rate)
        
        if self.task == "circle_3d":
            
            deg = 2 * math.pi / 10 * self.time
            radius = 2    
            ref.pose.pose.position.x, ref.pose.pose.position.y, ref.pose.pose.position.z = [radius * math.cos(deg), radius * math.sin(deg), 0]
            ref.pose.pose.orientation.w, ref.pose.pose.orientation.x, ref.pose.pose.orientation.y, ref.pose.pose.orientation.z = self.euler_to_quaternion(0, 0, 0)
            ref.twist.twist.linear.x, ref.twist.twist.linear.y, ref.twist.twist.linear.z = [0.0, 0.0, 0.0]
            ref.twist.twist.angular.x, ref.twist.twist.angular.y, ref.twist.twist.angular.z = [0.0, 0.0, 0.0]

            self.reference_pub.publish(ref)
        
        else:
            rospy.logerr("Task Error")

def main():

    rospy.init_node("reference_test_node")
    Ref_Wrapper()

if __name__ == '__main__':
    main()
