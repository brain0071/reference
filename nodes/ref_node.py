#!/usr/bin/env python

import rospy
import numpy as np
# from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import math
import time

class Test_Ref_Wrapper():
    def __init__(self):

        # send real_wp to simulator
        self.reference_pub = rospy.Publisher("test_reference", 
                                           Odometry, queue_size= 1)

        self.time = 0
        self.rate = 50
        self.reference_rate = rospy.Rate(self.rate)
        self.radius = 2
                
        task = rospy.get_param("~task")
        self.task = task
        self.last_pos = [self.radius, 0, 0]
    
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

            deg = 2 * math.pi / 60 * self.time
            print(self.time)
            print(deg)
            
            high = 2 
            
            # [self.radius 0 0]
            ref_x = self.radius * math.cos(deg)
            ref_y = self.radius * math.sin(deg)
            ref_z = 0 - high / 60 * self.time
            
            vel_x = ref_x - self.last_pos[0] / (1 / self.rate)
            vel_y = ref_y - self.last_pos[1] / (1 / self.rate)
            vel_z = ref_z - self.last_pos[2] / (1 / self.rate)
            
            self.last_pos = [ref_x, ref_y, ref_z]            
            
            ref_roll = 0
            ref_pitch = 0
            ref_yaw = 0

            ref_att= self.euler_to_quaternion(ref_roll, ref_pitch, ref_yaw)
            
            rate = [0, 0, 0]
            
            rospy.loginfo("ref_x: %f, ref_y: %f, ref_z: %f", ref_x, ref_y, ref_z)
            rospy.loginfo("vel_x: %f, vel_y: %f, vel_z: %f", vel_x, vel_y, vel_z)
            rospy.loginfo("qw: %f, qx: %f, qy: %f, qz: %f", ref_att[0], ref_att[1], ref_att[2], ref_att[3])
            rospy.loginfo("rate_x: %f, rate_y: %f, rate_z: %f", rate[0], rate[1], rate[2])
            
            ref = Odometry()          
            ref.pose.pose.position.x  = ref_x
            ref.pose.pose.position.y = ref_y
            ref.pose.pose.position.z = ref_z
            ref.pose.pose.orientation.w = ref_att[0]
            ref.pose.pose.orientation.x = ref_att[1]
            ref.pose.pose.orientation.y = ref_att[2]
            ref.pose.pose.orientation.z = ref_att[3]
            ref.twist.twist.linear.x = vel_x
            ref.twist.twist.linear.y = vel_y
            ref.twist.twist.linear.z = vel_z
            ref.twist.twist.angular.x = rate[0]
            ref.twist.twist.angular.y = rate[1]
            ref.twist.twist.angular.z = rate[2]
            self.reference_pub.publish(ref)
    
        else:
            rospy.logerr("Task Error")

def main():

    rospy.init_node("reference_test_node")
    

    Test_Ref_Wrapper()

if __name__ == '__main__':
    main()
