import rclpy
from rclpy.node import Node
import numpy as np

from nav_msgs.msg import Odometry
import math
import time

class Reference(Node):

    def __init__(self):

        super().__init__('reference_node')
        self.reference_pub = self.create_publisher(Odometry, "reference", 10)
        self.time = 0
        self.last_pos = [0.0, 0.0, 0.0]
        self.rate = 50
        self.dt = 1 / self.rate
        self.create_timer(self.dt, self.run_reference)

    def run_reference(self):
        
        self.time = self.time + (1 / self.rate)
        deg = 2 * math.pi / 60 * self.time
        roll = 0.0
        pitch = 0.0
        yaw = 0.0
        att = self.euler_to_quaternion(roll, pitch, yaw)
            
        x, y, z = 1.0 * math.cos(deg), 1.0 * math.sin(deg), 0.2
        pos = [x, y, z]
        vel = [(x - self.last_pos[0]) / self.dt , (y - self.last_pos[1]) / self.dt, (z - self.last_pos[2]) / self.dt]
        rate = [0.0, 0.0, 0.0]
        self.last_pos = pos
  
        ref = Odometry()          
        ref.pose.pose.position.x, ref.pose.pose.position.y, ref.pose.pose.position.z = pos
        ref.pose.pose.orientation.w, ref.pose.pose.orientation.x, ref.pose.pose.orientation.y, ref.pose.pose.orientation.z = att
        ref.twist.twist.linear.x, ref.twist.twist.linear.y, ref.twist.twist.linear.z = vel
        ref.twist.twist.angular.x, ref.twist.twist.angular.y, ref.twist.twist.angular.z = rate
        self.reference_pub.publish(ref)

    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
        qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2)
        qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2)
        qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
        return np.array([qw, qx, qy, qz])

def main(args=None):
    rclpy.init(args=args)
    reference = Reference()
    rclpy.spin(reference)
    reference.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
        

