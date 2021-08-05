#!/usr/bin/env python2

from __future__ import division, print_function, absolute_import

# Import ROS libraries
import roslib
import rospy
import rosbag
import numpy as np

# Import class that computes the desired positions
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import TransformStamped, Twist
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from std_msgs.msg._Empty import Empty


class ROSBagNode(object):

    def __init__(self):
        """Initialize the ROSControllerNode class"""

        # subscriber
        self.drone_pos_topic = '/gazebo_ground_truth_UAV0'
	self.drone_pos_msg = Odometry()
	self.payload_pos_topic = '/gazebo_ground_truth_payload'
        self.payload_pos_msg = Odometry()
        self.payload_est_topic = '/gazebo_estimate_payload_pose'
        self.payload_est_msg = TransformStamped()
        self.payload_est_rel_topic = '/gazebo_estimate_payload_pose_camera'
        self.payload_est_rel_msg = TransformStamped()
	self.image_topic = '/iris_0/bottom/image_raw'
	self.image_msg = Image()
        self.sub_drone = rospy.Subscriber(self.drone_pos_topic, Odometry, self.get_drone_pos)
        self.sub_payload = rospy.Subscriber(self.payload_pos_topic, Odometry, self.get_payload_pos)
        self.sub_payload_est = rospy.Subscriber(self.payload_est_topic, TransformStamped, self.get_payload_est)
        self.sub_payload_est_rel = rospy.Subscriber(self.payload_est_rel_topic, TransformStamped, self.get_payload_est_rel)
        self.sub_img = rospy.Subscriber(self.image_topic, Image, self.get_image)

        # run the rosbag writer at 20 Hz
        self.data_loop_frequency = 20.

        # run the rosbag writer at the given frequency
        self.rate = rospy.Rate(self.data_loop_frequency)
        self.time_stamp = 0
        self.bag_loc = '/home/consibic/Documents/rosbag_orig_controller/single_zigzag_no_wind.bag'
	self.bag = rosbag.Bag(self.bag_loc, 'w')

    def get_drone_pos(self, msg):
        self.drone_pos_msg = msg

    def get_payload_pos(self, msg):
        self.payload_pos_msg = msg

    def get_payload_est(self, msg):
        self.payload_est_msg = msg

    def get_payload_est_rel(self, msg):
        self.payload_est_rel_msg = msg

    def get_image(self, msg):
        self.image_msg = msg


if __name__ == '__main__':
    rospy.init_node("ros_bag", disable_signals=True)
    drone = ROSBagNode()
    counter = 0
    try:
        while not rospy.is_shutdown():
            drone.bag.write(drone.drone_pos_topic, drone.drone_pos_msg)
            drone.bag.write(drone.payload_pos_topic, drone.payload_pos_msg)
            drone.bag.write(drone.payload_est_topic, drone.payload_est_msg)
            drone.bag.write(drone.payload_est_rel_topic, drone.payload_est_rel_msg)
            # if counter == 100:
            #     drone.bag.write(drone.image_topic, drone.image_msg)
            #     counter = 0
            # else:
            #     counter += 1
            drone.rate.sleep() 
    finally:
        drone.bag.close()

    print('end')


