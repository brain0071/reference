import numpy as np
import math

def euler_to_quaternion(roll, pitch, yaw):
        qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
        qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2)
        qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2)
        qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
        print("qw:", qw, "qx:", qx, "qy:", qy, "qz:", qz)
        return np.array([qw, qx, qy, qz])

print("roll: 0, pitch: 0, yaw: 0")
euler_to_quaternion(math.radians(0),  math.radians(0), math.radians(0))

print("roll: 0, pitch: 0, yaw: 90")
euler_to_quaternion(math.radians(0),  math.radians(0), math.radians(90))

print("roll: 0, pitch: 0, yaw: 180")
euler_to_quaternion(math.radians(0),  math.radians(0), math.radians(180))

print("roll: 0, pitch: 0, yaw: 270")
euler_to_quaternion(math.radians(0),  math.radians(0), math.radians(270))


