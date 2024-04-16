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
        
        if self.task == "square": 
            delta_time = 60
            # real
            # length = 0.5
            # sim 
            length = 0.1
            if self.time < 1 * delta_time:
                
                ref_x = 0
                ref_y = 0
                ref_yaw = 0

            elif self.time > 1 * delta_time and self.time < 2 * delta_time:
                
                # ref_x = -length
                # ref_y = 0
                
                 
                ref_x = 0.2
                ref_y = -0.2
                
                # test
                ref_yaw = 90
            
            elif self.time > 2 * delta_time and self.time < 3 * delta_time:
                
                # ref_x = 0
                # ref_y = 0

                ref_x = length
                ref_y = length
                ref_yaw = -180

                # ref_x = length 
                # ref_y = 0
                # ref_yaw = -45
            
            elif self.time > 3 * delta_time and self.time <  4 * delta_time:
            
                ref_x = -length
                ref_y = length

                # ref_x = length
                # ref_y = -length

                ref_yaw = -90

            elif self.time > 4 * delta_time:
                
                ref_x = -length
                ref_y = -length
                
                # ref_x = 0
                # ref_y = 0
                
                ref_yaw = -90
    
            else:
                rospy.logerr("Time Error")
        
            ref_roll = 0
            ref_pitch = 0
            
            # real
            ref_z = 0.6
            # sim
            # ref_z = 0       
            ref_att= self.euler_to_quaternion(math.radians(ref_roll),  math.radians(ref_pitch), 
                                                math.radians(ref_yaw))
            
            rospy.loginfo("ref: %f, ref_y: %f, ref_z: %f", ref_x, ref_y, ref_z)
            rospy.loginfo("qw: %f, qx: %f, qy: %f, qz: %f", ref_att[0], ref_att[1], ref_att[2], ref_att[3])
            ref = PoseStamped()            
            ref.pose.position.x = ref_x
            ref.pose.position.y = ref_y
            ref.pose.position.z = ref_z
            ref.pose.orientation.w = ref_att[0]
            ref.pose.orientation.x = ref_att[1]
            ref.pose.orientation.y = ref_att[2]
            ref.pose.orientation.z = ref_att[3]
            
            self.reference_pub.publish(ref)

        elif self.task == "circle_2d":
            
            deg = 2 * math.pi / 60 * self.time
            
            radius = 0.1

            ref_x = radius * math.cos(deg)
            ref_y = radius * math.sin(deg)
            
            # ref_z = 0
            
            # real
            ref_z = 0.6
            
            ref_roll = 0
            ref_pitch = 0
            
            # ref_yaw = deg
            ref_yaw = 0
            
            ref_att= self.euler_to_quaternion(ref_roll, ref_pitch, ref_yaw)
            rospy.loginfo("ref: %f, ref_y: %f, ref_z: %f", ref_x, ref_y, ref_z)
            rospy.loginfo("qw: %f, qx: %f, qy: %f, qz: %f", ref_att[0], ref_att[1], ref_att[2], ref_att[3])
            ref = PoseStamped()            
            ref.pose.position.x = ref_x
            ref.pose.position.y = ref_y
            ref.pose.position.z = ref_z
            ref.pose.orientation.w = ref_att[0]
            ref.pose.orientation.x = ref_att[1]
            ref.pose.orientation.y = ref_att[2]
            ref.pose.orientation.z = ref_att[3]
            self.reference_pub.publish(ref)

        elif self.task == "circle_3d":

            deg = 2 * math.pi / 60 * self.time
            print(self.time)
            print(deg)
            radius = 2
            high = 0.6 

            ref_x = radius * math.cos(deg)
            ref_y = radius * math.sin(deg)
            ref_z = 0 - high / 60 * self.time
            
            ref_roll = 0
            ref_pitch = 0
            ref_yaw = deg

            ref_att= self.euler_to_quaternion(ref_roll, ref_pitch, ref_yaw)
            
            rospy.loginfo("ref: %f, ref_y: %f, ref_z: %f", ref_x, ref_y, ref_z)
            rospy.loginfo("qw: %f, qx: %f, qy: %f, qz: %f", ref_att[0], ref_att[1], ref_att[2], ref_att[3])
            
            ref = PoseStamped()            
            ref.pose.position.x = ref_x
            ref.pose.position.y = ref_y
            ref.pose.position.z = ref_z
            ref.pose.orientation.w = ref_att[0]
            ref.pose.orientation.x = ref_att[1]
            ref.pose.orientation.y = ref_att[2]
            ref.pose.orientation.z = ref_att[3]
            self.reference_pub.publish(ref)
        
        elif self.task == "grasp":
            pass
        
        else:
            rospy.logerr("Task Error")

def main():

    rospy.init_node("reference_test_node")
    

    Test_Ref_Wrapper()

if __name__ == '__main__':
    main()
