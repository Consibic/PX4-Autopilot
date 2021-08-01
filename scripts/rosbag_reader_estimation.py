#!/usr/bin/env python2

from __future__ import division, print_function, absolute_import

# Import ROS libraries
import roslib
import rospy
import rosbag
import numpy as np
from matplotlib import pyplot as plt
from tf.transformations import euler_from_quaternion


bag_loc = '/home/consibic/Documents/rosbag_orig_controller/single_circular_estimation.bag'
list_topics = ['/gazebo_ground_truth_payload', '/gazebo_estimate_payload_pose']
bag = rosbag.Bag(bag_loc)

gt_tr_list = [[], [], []]  # x, y, z
gt_rot_list = [[], [], []]  # r, p, y
est_tr_list = [[], [], []]
est_rot_list = [[], [], []]

for topic, msg, t in bag.read_messages(topics=list_topics):
    if topic == '/gazebo_ground_truth_payload':
        gt_tr_list[0].append(msg.pose.pose.position.x)
        gt_tr_list[1].append(msg.pose.pose.position.y)
        gt_tr_list[2].append(msg.pose.pose.position.z)
        gt_qtn = msg.pose.pose.orientation
        gt_qtn_list = [gt_qtn.x, gt_qtn.y, gt_qtn.z, gt_qtn.w]
        gt_euler = euler_from_quaternion(gt_qtn_list)   
        gt_euler_x = gt_euler[0]
        gt_euler_x_u = gt_euler[0] - np.pi / 2
        if gt_euler_x_u < -np.pi:
            gt_euler_x_u += 2 * np.pi
        gt_rot_list[0].append(gt_euler_x_u)
        gt_rot_list[1].append(gt_euler[1])
        gt_rot_list[2].append(gt_euler[2])
    else:
        est_tr_list[0].append(msg.transform.translation.x)
        est_tr_list[1].append(msg.transform.translation.y)
        est_tr_list[2].append(msg.transform.translation.z)
        est_qtn = msg.transform.rotation
        est_qtn_list = [est_qtn.x, est_qtn.y, est_qtn.z, est_qtn.w]
        est_euler = euler_from_quaternion(est_qtn_list)   
        est_euler_z = est_euler[2]
        est_euler_z_u = est_euler[2] + np.pi / 2
        if est_euler_z_u > np.pi:
            est_euler_z_u -= 2 * np.pi 
        est_rot_list[0].append(est_euler[0])
        est_rot_list[1].append(est_euler[1])
        est_rot_list[2].append(est_euler[2])

print(est_tr_list[0])
print(np.shape(gt_tr_list))
print(np.shape(est_tr_list))

'''
plt.figure()
plt.title('Yaw of Zigzag Path')
plt.plot(gt_rot_list[2], label='ground truth')
plt.plot(est_rot_list[2], label='estimation')
plt.legend()
plt.show()
'''

tr_diff_list = [[], [], []]
rot_diff_list = [[], [], []]
for i, item in enumerate(gt_tr_list[0]):
    tr_diff_list[0].append(est_tr_list[0][i] - item)
    tr_diff_list[1].append(est_tr_list[1][i] - gt_tr_list[1][i])
    tr_diff_list[2].append(est_tr_list[2][i] - gt_tr_list[2][i])
    if abs(est_rot_list[0][i] - gt_rot_list[0][i]) > abs(est_rot_list[0][i] + gt_rot_list[0][i]):
        rot_diff_list[0].append(est_rot_list[0][i] + gt_rot_list[0][i])
    else:
        rot_diff_list[0].append(est_rot_list[0][i] - gt_rot_list[0][i])
    if abs(est_rot_list[1][i] - gt_rot_list[1][i]) > abs(est_rot_list[1][i] + gt_rot_list[1][i]):
        rot_diff_list[1].append(est_rot_list[1][i] + gt_rot_list[1][i])
    else:
        rot_diff_list[1].append(est_rot_list[1][i] - gt_rot_list[1][i])
    if abs(est_rot_list[2][i] - gt_rot_list[2][i]) > abs(est_rot_list[2][i] + gt_rot_list[2][i]):
        rot_diff_list[2].append(est_rot_list[2][i] + gt_rot_list[2][i])
    else:
        rot_diff_list[2].append(est_rot_list[2][i] - gt_rot_list[2][i])

plt.figure()
plt.title('Yaw Difference of Zigzag Path')
plt.plot(rot_diff_list[2])
plt.show()
