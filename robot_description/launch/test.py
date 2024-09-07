import numpy as np # Scientific computing library
 
# Project: Calculating Rotation Matrices
# Author: Addison Sears-Collins
# Date created: August 1, 2020
 
# Servo (joint) angles in degrees
servo_0_angle = 90 # Joint 1
servo_1_angle = 0 # Joint 2
 
# Convert servo angles from degrees to radians
servo_0_angle = np.deg2rad(servo_0_angle)
servo_1_angle = np.deg2rad(servo_1_angle)
 
# Define the first rotation matrix.
# This matrix helps convert frame 1 to frame 0.
# There is only rotation around the z axis of servo_0.
rot_mat_0_1 = np.array([[np.cos(servo_0_angle), -np.sin(servo_0_angle), 0],
                        [np.sin(servo_0_angle), np.cos(servo_0_angle), 0],
                        [0, 0, 1]]) 
 
# Define the second rotation matrix.
# This matrix helps convert the 
# end-effector frame (frame 2) to frame 1.
# There is only rotation around the z axis of servo_1 (joint 2).
rot_mat_1_2 = np.array([[np.cos(servo_1_angle), -np.sin(servo_1_angle), 0],
                        [np.sin(servo_1_angle), np.cos(servo_1_angle), 0],
                        [0, 0, 1]]) 
 
# Calculate the rotation matrix that converts the 
# end-effector frame to the servo_0 frame.
rot_mat_0_2 = rot_mat_0_1 @ rot_mat_1_2
 
# Display the rotation matrix
print(rot_mat_0_2)