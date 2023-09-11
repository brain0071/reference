#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
import math


class Test_Ref_Wrapper():
    def __init__(self):

        # send real_wp to simulator
        self.reference_pub = rospy.Publisher("test_reference", 
                                           PoseStamped, queue_size= 1)
       
        self.yaw_max = 30
        self.roll_max = 30
        self.pitch_max = 15

        self.time = 0
        self.rate = 10
        self.reference_rate = rospy.Rate(10)
        
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
         
        
        # ref_roll = self.roll_max * math.sin((math.pi / 1) * self.time)
        ref_roll = 0
        
        ref_pitch = self.pitch_max * math.sin(0.1*( math.pi / 1) * self.time)
        # ref_pitch = 0

        # ref_yaw = self.yaw_max * math.sin((math.pi / 1) * self.time)
        ref_yaw = 0

        ref_att= self.euler_to_quaternion(math.radians(ref_roll),  math.radians(ref_pitch), 
                                                math.radians(ref_yaw))
        rospy.loginfo("qw: %f, qx: %f, qy: %f, qz: %f", ref_att[0], ref_att[1], ref_att[2], ref_att[3])
        ref = PoseStamped()
        ref.pose.position.x = 0
        
        ref.pose.position.y = 0
        
        ref.pose.position.z = 0
        
        ref.pose.orientation.w = ref_att[0]

        ref.pose.orientation.x = ref_att[1]
        
        ref.pose.orientation.y = ref_att[2]
        
        ref.pose.orientation.z = ref_att[3]

        self.reference_pub.publish(ref)

    
def main():

    rospy.init_node("reference_test_node")
    

    Test_Ref_Wrapper()

if __name__ == '__main__':
    main()
