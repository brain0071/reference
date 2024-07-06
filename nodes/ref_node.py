#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
import math
import time

class Test_Ref_Wrapper():
    def __init__(self):

        # send real_wp to simulator
        self.reference_pub = rospy.Publisher("test_reference", 
                                           PoseStamped, queue_size= 1)
       
        self.time = 0
        self.rate = 10
        self.reference_rate = rospy.Rate(10)
        
        task = rospy.get_param("~task")
        self.task = task
        
        self.ref_x = 3
        self.ref_y = 2
        self.ref_z = 0.3

        while True:
            self.run_reference()
            self.reference_rate.sleep()

    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
        qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2)
        qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2)
        qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)

        return np.array([qw, qx, qy, qz])

    
    # task 1:
    # point trakcing: square (0.5m)
    # task 2: 2d circle (diameter 1m)
    # task 3: 3d circle (diameter 1m, high 0.5m ~ 0.7m)

    def run_reference(self):
        
        self.time = self.time + (1 / self.rate)
        
        if self.task == "point": 
            
            ref_roll = 0
            ref_pitch = 0
            ref_yaw = 0
                    

        elif self.task == "roll":
            
            ref_roll = 45
            ref_pitch = 0
            ref_yaw = 0       
                 
        elif self.task == "roll_sin":
            
            # T = 10s
            deg = 2 * math.pi / 10 * self.time
            
            roll = 45
            ref_roll = roll * math.sin(deg)
            ref_pitch = 0
            ref_yaw = 0
            
        elif self.task == "pitch":
            
            ref_roll = 0
            ref_pitch = 45
            ref_yaw = 0    
            
        elif self.task == "pitch_sin":       
            
            # T = 10s
            deg = 2 * math.pi / 10 * self.time
            
            pitch = 45
            ref_pitch = pitch * math.sin(deg)
            ref_roll = 0
            ref_yaw = 0
        
        elif self.task == "yaw":
            
            ref_roll = 0
            ref_pitch = 0
            ref_yaw = 45   
            
        elif self.task == "yaw_sin":       
            
            # T = 10s
            deg = 2 * math.pi / 10 * self.time
            yaw = 45
            ref_yaw = yaw * math.sin(deg)
            ref_roll = 0
            ref_pitch = 0
        
        else:
            rospy.logerr("Task Error")
        
        ref_att= self.euler_to_quaternion(math.radians(ref_roll),  math.radians(ref_pitch), 
                                                math.radians(ref_yaw))
        
        rospy.loginfo("ref: %f, ref_y: %f, ref_z: %f", self.ref_x, self.ref_y, self.ref_z)
        rospy.loginfo("qw: %f, qx: %f, qy: %f, qz: %f", ref_att[0], ref_att[1], ref_att[2], ref_att[3])
        ref = PoseStamped()            
        ref.pose.position.x = self.ref_x
        ref.pose.position.y = self.ref_y
        ref.pose.position.z = self.ref_z
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
