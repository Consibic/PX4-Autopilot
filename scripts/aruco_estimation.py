#!/usr/bin/env python

# A basic video display window for the tutorial "Up and flying with the AR.Drone and ROS | Getting Started"
# https://github.com/mikehamer/ardrone_tutorials_getting_started

# This display window listens to the drone's video feeds and updates the display at regular intervals
# It also tracks the drone's status and any connection problems, displaying them in the window's status bar
# By default it includes no control functionality. The class can be extended to implement key or mouse listeners if required

# Import the ROS libraries, and load the manifest file which through <depend package=... /> will give us access to the project dependencies
import roslib
import rospy
import cv2
import math
import os
import copy

# Import the two types of messages we're interested in
from sensor_msgs.msg import Image    	 # for receiving the video feed

# 2017-03-22 Import libraries from OpenCV
# OpenCV Bridge http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import TransformStamped, Twist
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_matrix, quaternion_from_euler, euler_from_matrix, euler_matrix
import numpy as np

class DroneVideoDisplay():
	
	def __init__(self):
		
		# Subscribe to the drone's video feed, calling self.ReceiveImage when a new frame is received
		self.subVideoBottom   = rospy.Subscriber('/iris_0/bottom/image_raw/', Image, self.ReceiveImageBottom,queue_size=1)

                # subscribe to /vicon/ARDroneCarre/ARDroneCarre for position and attitude feedback
                self.vicon_topic = '/gazebo_ground_truth_UAV0'
                self._vicon_msg = Odometry()
                self.sub_vicon = rospy.Subscriber(self.vicon_topic, Odometry, self._vicon_callback)

                # subscribe to the averaged payload pose estimations
                self._payload_pose_msg = TransformStamped()
                self.sub_payload_pose = rospy.Subscriber('/sensor_msgs/Image/ave_payload_pose', TransformStamped, self._payload_pose_callback)
		
		# convert ROS images to OpenCV images
		self.bridge = CvBridge()

		# set to enable or disable CV image processing
		self.processImages = True	

                '''********************Project Setup Parameters Below*************************'''

                # enable (True) or disable (False) RANSAC outlier rejection
                self.RANSAC_enabled = False

                # set up sample counter
                self.count = 0
                # set the frequency of how often the images will be processed (once every this many frames) 
                self.process_freq = 4  

                # declare properties from the intrinsic camera matrix
                self.fx = 604.62
                self.fy = 604.62
                self.cx = 320.5
                self.cy = 180.5 
                # build the instrinsic camera matrix (K)
                self.K = np.array([[self.fx, 0.0, self.cx], [0.0, self.fy, self.cy], [0.0, 0.0, 1.0]])
                # declare the distortion properties (k1, k2, T1, T2, k3)
                self.dist = np.array([0.0, 0.0, 0.0, 0.0, 0.0]) 

                # declare the rotation and translations between the body (vehicle) and camera frames
                self.C_cb = np.array([[0.0, -1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])  
                self.r_bc = np.array([[0.0], [0.0175], [-0.03]])  
                # build transformation matrix from the body to the camera frame
                self.T_cb = np.zeros((4,4))
                self.T_cb[0:3,0:3] = self.C_cb 
                self.T_cb[0:3,[3]] = self.r_bc
                self.T_cb[3,3] = 1.
                # build transformation matrix from the camera to the body frame
                self.T_bc = np.zeros((4,4)) 
                self.T_bc[0:3,0:3] = np.transpose(self.C_cb)
                self.T_bc[0:3,[3]] = np.dot((-1*np.transpose(self.C_cb)), (self.r_bc)) 
                self.T_bc[3,3] = 1.

                # declare empty array to store time stamps (element 1 is secs, element 2 is nsecs)
                self.time = np.zeros(2)

                # set the ArUco dictionary as 6x6 (250)
                self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)   
                # set up the ArUco detector with default parameters
                self.aruco_params = cv2.aruco.DetectorParameters_create()     
                # set the real-world ArUco marker size in meters
                self.aruco_size = 0.2 

                # set up transformation matrices from the payload to the marker frames
                self.T_mp = np.zeros((24, 4, 4))
                # marker ID 0 rotation and translation: 
                self.T_mp[[0],0:4,0:4] = np.eye(4)
                self.T_mp[[0],0:3,[3]] = np.array([0.125, -0.125, -0.25]) 
                # marker ID 1 rotation and translation:
                self.T_mp[[1],0:4,0:4] = np.eye(4)
                self.T_mp[[1],0:3,[3]] = np.array([-0.125, -0.125, -0.25]) 
                # marker ID 2 rotation and translation:
                self.T_mp[[2],0:4,0:4] = np.eye(4)
                self.T_mp[[2],0:3,[3]] = np.array([0.125, 0.125, -0.25]) 
                # marker ID 3 rotation and translation:
                self.T_mp[[3],0:4,0:4] = np.eye(4)
                self.T_mp[[3],0:3,[3]] = np.array([-0.125, 0.125, -0.25])  
                # marker ID 4 rotation and translation: 
                self.T_mp[[4],0:4,0:4] = [[0,0,-1,0],[0,1,0,0],[1,0,0,0],[0,0,0,1]]
                self.T_mp[[4],0:3,[3]] = np.array([0.125, -0.125, -0.25]) 
                # marker ID 5 rotation and translation:
                self.T_mp[[5],0:4,0:4] = [[0,0,-1,0],[0,1,0,0],[1,0,0,0],[0,0,0,1]]
                self.T_mp[[5],0:3,[3]] = np.array([-0.125, -0.125, -0.25]) 
                # marker ID 6 rotation and translation:
                self.T_mp[[6],0:4,0:4] = [[0,0,-1,0],[0,1,0,0],[1,0,0,0],[0,0,0,1]]
                self.T_mp[[6],0:3,[3]] = np.array([0.125, 0.125, -0.25]) 
                # marker ID 7 rotation and translation:
                self.T_mp[[7],0:4,0:4] = [[0,0,-1,0],[0,1,0,0],[1,0,0,0],[0,0,0,1]]
                self.T_mp[[7],0:3,[3]] = np.array([-0.125, 0.125, -0.25])  
                # marker ID 8 rotation and translation: 
                self.T_mp[[8],0:4,0:4] = [[-1,0,0,0],[0,1,0,0],[0,0,-1,0],[0,0,0,1]]
                self.T_mp[[8],0:3,[3]] = np.array([0.125, -0.125, -0.25]) 
                # marker ID 9 rotation and translation:
                self.T_mp[[9],0:4,0:4] = [[-1,0,0,0],[0,1,0,0],[0,0,-1,0],[0,0,0,1]]
                self.T_mp[[9],0:3,[3]] = np.array([-0.125, -0.125, -0.25]) 
                # marker ID 10 rotation and translation:
                self.T_mp[[10],0:4,0:4] = [[-1,0,0,0],[0,1,0,0],[0,0,-1,0],[0,0,0,1]]
                self.T_mp[[10],0:3,[3]] = np.array([0.125, 0.125, -0.25]) 
                # marker ID 11 rotation and translation:
                self.T_mp[[11],0:4,0:4] = [[-1,0,0,0],[0,1,0,0],[0,0,-1,0],[0,0,0,1]]
                self.T_mp[[11],0:3,[3]] = np.array([-0.125, 0.125, -0.25])  
                # marker ID 12 rotation and translation: 
                self.T_mp[[12],0:4,0:4] = [[0,0,1,0],[0,1,0,0],[-1,0,0,0],[0,0,0,1]]
                self.T_mp[[12],0:3,[3]] = np.array([0.125, -0.125, -0.25]) 
                # marker ID 13 rotation and translation:
                self.T_mp[[13],0:4,0:4] = [[0,0,1,0],[0,1,0,0],[-1,0,0,0],[0,0,0,1]]
                self.T_mp[[13],0:3,[3]] = np.array([-0.125, -0.125, -0.25]) 
                # marker ID 14 rotation and translation:
                self.T_mp[[14],0:4,0:4] = [[0,0,1,0],[0,1,0,0],[-1,0,0,0],[0,0,0,1]]
                self.T_mp[[14],0:3,[3]] = np.array([0.125, 0.125, -0.25]) 
                # marker ID 15 rotation and translation:
                self.T_mp[[15],0:4,0:4] = [[0,0,1,0],[0,1,0,0],[-1,0,0,0],[0,0,0,1]]
                self.T_mp[[15],0:3,[3]] = np.array([-0.125, 0.125, -0.25])  
                # marker ID 16 rotation and translation: 
                self.T_mp[[16],0:4,0:4] = [[0,-1,0,0],[0,0,-1,0],[1,0,0,0],[0,0,0,1]]
                self.T_mp[[16],0:3,[3]] = np.array([0.125, -0.125, -0.25]) 
                # marker ID 17 rotation and translation:
                self.T_mp[[17],0:4,0:4] = [[0,-1,0,0],[0,0,-1,0],[1,0,0,0],[0,0,0,1]]
                self.T_mp[[17],0:3,[3]] = np.array([-0.125, -0.125, -0.25]) 
                # marker ID 18 rotation and translation:
                self.T_mp[[18],0:4,0:4] = [[0,-1,0,0],[0,0,-1,0],[1,0,0,0],[0,0,0,1]]
                self.T_mp[[18],0:3,[3]] = np.array([0.125, 0.125, -0.25]) 
                # marker ID 19 rotation and translation:
                self.T_mp[[19],0:4,0:4] = [[0,-1,0,0],[0,0,-1,0],[1,0,0,0],[0,0,0,1]]
                self.T_mp[[19],0:3,[3]] = np.array([-0.125, 0.125, -0.25])  
                # marker ID 20 rotation and translation: 
                self.T_mp[[20],0:4,0:4] = [[0,1,0,0],[0,0,1,0],[1,0,0,0],[0,0,0,1]]
                self.T_mp[[20],0:3,[3]] = np.array([0.125, -0.125, -0.25]) 
                # marker ID 21 rotation and translation:
                self.T_mp[[21],0:4,0:4] = [[0,1,0,0],[0,0,1,0],[1,0,0,0],[0,0,0,1]]
                self.T_mp[[21],0:3,[3]] = np.array([-0.125, -0.125, -0.25]) 
                # marker ID 22 rotation and translation:
                self.T_mp[[22],0:4,0:4] = [[0,1,0,0],[0,0,1,0],[1,0,0,0],[0,0,0,1]]
                self.T_mp[[22],0:3,[3]] = np.array([0.125, 0.125, -0.25]) 
                # marker ID 23 rotation and translation:
                self.T_mp[[23],0:4,0:4] = [[0,1,0,0],[0,0,1,0],[1,0,0,0],[0,0,0,1]]
                self.T_mp[[23],0:3,[3]] = np.array([-0.125, 0.125, -0.25])  

                # create TransformStamped class to store marker poses for publishing 
                self._marker_pose = TransformStamped() 

                # set up ROS publisher to publish marker poses in the inertial frame
                self.pub_marker_pose = rospy.Publisher('/gazebo_estimate_marker_pose', TransformStamped, queue_size = 32)
                # set up ROS publisher to publish marker poses in the camera frame
                self.pub_marker_pose_camera = rospy.Publisher('/gazebo_estimate_marker_pose_camera', TransformStamped, queue_size = 32)

                # set up ROS publisher to publish payload poses in the inertial frame
                self.pub_payload_pose = rospy.Publisher('/gazebo_estimate_payload_pose', TransformStamped, queue_size = 32)
                # set up ROS publisher to publish payload poses in the camera frame
                self.pub_payload_pose_camera = rospy.Publisher('/gazebo_estimate_payload_pose_camera', TransformStamped, queue_size = 32)

                # set up ROS publisher to publish average payload poses in the inertial frame
                self.pub_ave_payload_pose = rospy.Publisher('/gazebo_estimate_payload_avg_pose', TransformStamped, queue_size = 32)

                # set up ROS publisher to publish payload velocity in the inertial frame
                self.pub_payload_vel = rospy.Publisher('/gazebo_estimate_payload_vel', TransformStamped, queue_size = 32)

                # sut up publisher to publish ROS images of processed images
                self.pub_image_processed = rospy.Publisher('/sensor_msgs/Image/processed', Image, queue_size = 1) 

                '''************************ End of Project Setup Parameters *************************'''
			
        """method to receive the bottom camera images, call the image processing methods and then publish the results if any obstacles or landmarks are detected"""
	def ReceiveImageBottom(self,data):

		try:
                    # backup the vicon pose the moment the image is received 
                    image_pose = self._vicon_msg
                    # Save the ros image for processing by the display thread
                    self.image = data 

                    # If image processing is enabled and this is a cycle we wish to perform processing on
                    if self.processImages and (self.count % self.process_freq) == 0:
                        try:
                            # convert from ROS image to OpenCV image
                            cv_image = self.bridge.imgmsg_to_cv2(self.image, desired_encoding="bgr8")

                           
                            # call method to find the ids, rotation and translation of all markers found in the image in the camera frame
                            self.find_markers(cv_image)  
                            # if at least 1 ArUco marker was found
                            if len(self.ids) > 0:
                                # print divider for this image
                                print('---------------------------------------------------------')
                                # compute the rotation and transformation matrices from the body to the inertial frame
                                self.pose_to_transform_mat(image_pose)
                                # reset array which contains estimated marker positions in the inertial frame (each row is a marker)
                                # the first 3 columns are (x,y,z) position and the 4th column is marker ID 
                                self.p_a = np.zeros((0,4)) 
                                # reset array which contains estimated payload poses in the inertial frame (each row is from a marker)
                                # the columns are (x,y,z) position, followed by roll, pitch & yaw angles and then the marker ID 
                                self.T_ip_j = np.zeros((0,7)) 
                                # loop through every marker found 
                                for i in range(len(self.tvec)):
                                    # use the marker's rotation and translation from the camera frame to estimate marker & payload pose in the inertial frame
                                    self.camera_to_inertial_frame(self.rvec[i], self.tvec[i], self.ids[i]) 
                                                
                                # if RANSAC outlier rejection is enabled and at least 3 markers found
                                # if self.RANSAC_enabled and len(self.ids) >= 3:
                                    # TODO run RANSAC algorithm
                                    # print('RANSAC!') 

                                # take the average of the inlier measurements and publish the result
                                self.average_payload_pose()
   
                        except CvBridgeError as e:
                           print "Image conversion failed: %s" % e

		finally:
			pass
                # increment sample counter for the next loop
                self.count += 1

        # vicon system callback method
        def _vicon_callback(self, msg):
            self._vicon_msg = msg

        # payload pose callback method  
        def _payload_pose_callback(self, msg):
            
            pose_prev = self._payload_pose_msg
            self._payload_pose_msg = msg

            # compute delta time (dt) in seconds since last sample 
            dt = (float(self._payload_pose_msg.header.stamp.secs) - float(pose_prev.header.stamp.secs)) + ((float(self._payload_pose_msg.header.stamp.nsecs) - float(pose_prev.header.stamp.nsecs))/10**9)
            # get position differences between the current and previous time stamp
            dx = self._payload_pose_msg.transform.translation.x - pose_prev.transform.translation.x
            dy = self._payload_pose_msg.transform.translation.y - pose_prev.transform.translation.y
            dz = self._payload_pose_msg.transform.translation.z - pose_prev.transform.translation.z    

            # create message for the payload velocity using the pose message as a template
            payload_vel_msg = copy.deepcopy(self._payload_pose_msg)
            # marker linear velocity in meters/sec
            payload_vel_msg.transform.translation.x = dx/dt
            payload_vel_msg.transform.translation.y = dy/dt
            payload_vel_msg.transform.translation.z = dz/dt
            # marker angular velocity (0's as place holder)
            payload_vel_msg.transform.rotation.x = 0
            payload_vel_msg.transform.rotation.y = 0
            payload_vel_msg.transform.rotation.z = 0
            payload_vel_msg.transform.rotation.w = 0

            # TODO put the velocities through an exponential moving average filter before publishing 

            # publish the payload velocity's transformStamped message to the ROS topic
            self.pub_payload_vel.publish(payload_vel_msg) 
         
        '''***************************************************************************************************''' 
        '''**********************Project Target Detection Code in the Method Below****************************''' 
        '''***************************************************************************************************'''

        """method to take an image, identify all ArUco markers and then determine their IDs, rotations & translations wrt. the camera frame """
        def find_markers(self, img):

            # detect all ArUco markers in the image and return their 2D coordinates and IDs
            corners, self.ids, rejected = cv2.aruco.detectMarkers(img, self.aruco_dict, parameters=self.aruco_params)

            # if at least one marker was found   
            if len(corners) > 0:
                # draw a border around the detected marker along with the ID number
                cv2.aruco.drawDetectedMarkers(img, corners, self.ids)
                
                # solve PNP to get the rotation and translation vectors for all detected markers
                self.rvec, self.tvec = cv2.aruco.estimatePoseSingleMarkers(corners, self.aruco_size, self.K, self.dist)  

                # loop through each marker detection
                for i in range(len(self.rvec)): 
                    # plot the coordinate frame axes on each detected marker
                    cv2.aruco.drawAxis(img, self.K, self.dist, self.rvec[i], self.tvec[i], self.aruco_size/2.)
                    # and print basic detection results in the camera frame for debugging (tvec is in m)
                    # print('Marker ' + str(self.ids[i]) + ' Found at: ' + str(self.tvec[i])) 
  
            # else print that no markers were found to the terminal and return empty results
            else:
                # print('No Markers Detected.')  
                self.ids = []
                self.rvec = []
                self.tvec = []      

            # use bridge to convert processed image back to a format which can be published to a ROS topic
            img_msg = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
            # publish the processed image to the ROS topic
            self.pub_image_processed.publish(img_msg)

            return

        """method to take a transformStamped pose message, compute the corresponding transformation matrix & extract the time stamp"""   
        def pose_to_transform_mat(self, pose_angle):

            # get 3D attitude (in quaternions)
            x = pose_angle.pose.pose.orientation.x
            y = pose_angle.pose.pose.orientation.y
            z = pose_angle.pose.pose.orientation.z
            w = pose_angle.pose.pose.orientation.w

            # convert quaternions to a transformation matrix from the body (vehicle) to the inertial frame   
            self.T_ib = quaternion_matrix([x, y, z, w])

            # extract the translation from the inertial frame to the body frame and insert to transformation matrix
            self.T_ib[0,3] = pose_angle.pose.pose.position.x
            self.T_ib[1,3] = pose_angle.pose.pose.position.y
            self.T_ib[2,3] = pose_angle.pose.pose.position.z

            # get time stamp (sec, nano_sec) of this vicon pose measurement
            self.time[0] = pose_angle.header.stamp.secs
            self.time[1] = pose_angle.header.stamp.nsecs

            return

        """method to convert ArUco marker detections (rotation & translation vactors) from the camera frame to the inertial frame"""
        def camera_to_inertial_frame(self, r_vec, t_vec, marker_id):
            
            # if the marker's ID is within the known range, proceed
            if marker_id < len(self.T_mp): 
  
                """conversion from T_cm to T_im"""

                # initialize the 4x4 transformation matrix from the marker to the camera frame
                T_cm = np.eye(4)
                # insert the translation vector from the camera to the marker frame wrt. the camera frame (r_mc) 
                T_cm[0:3,[3]] = np.array([[t_vec[0][0]], [t_vec[0][1]], [t_vec[0][2]]]) 
                # convert the rotation vector to a rotation matrix from the marker to the camera frame (C_cm)
                T_cm[0:3,0:3], _ = cv2.Rodrigues(r_vec[0])   
 
                # compound transformation matrices to convert the marker pose from the camera to the inertial frame
                T_bm = np.dot(self.T_bc, T_cm)
                T_im = np.dot(self.T_ib, T_bm)    
                # extract the payload markers's roll, pitch & yaw from the rotation matrix
                euler = euler_from_matrix(T_im[0:3,0:3], 'sxyz') 

                # print the marker location (in the inertial frame) to the terminal for debugging   
                #print('\nMarker ' + str(marker_id[0]) + ' Found at: x: ' + str(T_im[0,3]) + ' y: ' + str(T_im[1,3]) + ' z: ' + str(T_im[2,3]))         
                # print the marker's attitude (in the inertial frame) to the terminal for debugging 
                #print('Marker: roll: ' + str(euler[0]) + ' pitch: '  + str(euler[1]) + ' yaw: ' + str(euler[2]))   
 
                # append the marker position and ID to the backed up list for this image
                self.p_a = np.vstack((self.p_a, np.array([T_im[0,3], T_im[1,3], T_im[2,3], marker_id])))

                # convert the marker's euler angles to quaternions
                quaternion = quaternion_from_euler(euler[0], euler[1], euler[2], 'sxyz')

                # wrap up all results in a transform stamped message:
                # time stamp:
                self._marker_pose.header.stamp.secs = self.time[0]
                self._marker_pose.header.stamp.nsecs = self.time[1]
                # image number:   
                self._marker_pose.header.frame_id = str(self.count)
                # marker ID:
                self._marker_pose.child_frame_id = str(marker_id[0])  
                # marker position in meters
                self._marker_pose.transform.translation.x = T_im[0,3]
                self._marker_pose.transform.translation.y = T_im[1,3]
                self._marker_pose.transform.translation.z = T_im[2,3]
                # marker attitude in quaternions
                self._marker_pose.transform.rotation.x = quaternion[0]
                self._marker_pose.transform.rotation.y = quaternion[1]
                self._marker_pose.transform.rotation.z = quaternion[2]
                self._marker_pose.transform.rotation.w = quaternion[3] 

                # publish the marker's pose to a ROS topic
                self.pub_marker_pose.publish(self._marker_pose)  

                # compute and publish the pose of marker wrt the camera (drone)
                marker_pose_camera = TransformStamped()
                marker_pose_camera.header.stamp.secs = self.time[0]
                marker_pose_camera.header.stamp.nsecs = self.time[1]
                # image number:   
                marker_pose_camera.header.frame_id = str(self.count)
                # marker ID:
                marker_pose_camera.child_frame_id = str(marker_id[0])  
                # marker position in meters
                marker_pose_camera.transform.translation.x = T_bm[0,3]
                marker_pose_camera.transform.translation.y = T_bm[1,3]
                marker_pose_camera.transform.translation.z = T_bm[2,3]
                # marker attitude in quaternions
                euler_camera = euler_from_matrix(T_bm[0:3, 0:3], 'sxyz')   
                quaternion_camera = quaternion_from_euler(euler_camera[0], euler_camera[1], euler_camera[2], 'sxyz')  
                marker_pose_camera.transform.rotation.x = quaternion_camera[0]
                marker_pose_camera.transform.rotation.y = quaternion_camera[1]
                marker_pose_camera.transform.rotation.z = quaternion_camera[2]
                marker_pose_camera.transform.rotation.w = quaternion_camera[3]  
		
                self.pub_marker_pose_camera.publish(marker_pose_camera) 
 
                """conversion from T_im to T_ip"""
                
                # build the transformation matrix from the payload to the inertial frame
                T_ip = np.dot(T_im, self.T_mp[marker_id[0]])
                # extract the payload's roll, pitch & yaw from the rotation matrix
                euler = euler_from_matrix(T_ip[0:3,0:3], 'sxyz')

                # print the marker location (in the inertial frame) to the terminal for debugging   
                #print('\nEstimated Payload Pose from Marker ' + str(marker_id[0]))
                #print('x: ' + str(T_ip[0,3]) + ' y: ' + str(T_ip[1,3]) + ' z: ' + str(T_ip[2,3]))         
                # print the marker's attitude (in the inertial frame) to the terminal for debugging 
                #print('roll: ' + str(euler[0]) + ' pitch: '  + str(euler[1]) + ' yaw: ' + str(euler[2])) 

                # append the payload pose estimation and ID to the backed up list for this image
                self.T_ip_j = np.vstack((self.T_ip_j, np.array([T_ip[0,3], T_ip[1,3], T_ip[2,3], euler[0], euler[1], euler[2], marker_id]))) 

                # convert the payload's euler angles to quaternions
                quaternion = quaternion_from_euler(euler[0], euler[1], euler[2], 'sxyz')

                # copy the marker pose's transformStamped message to set up the payload pose message
                payload_pose = copy.deepcopy(self._marker_pose)

                # overwrite the position with the vector from the inertial frame to the payload frame
                payload_pose.transform.translation.x = T_ip[0,3]
                payload_pose.transform.translation.y = T_ip[1,3]
                payload_pose.transform.translation.z = T_ip[2,3]
                # overwrite the rotation with the quaternion from the inertial frame to the payload frame
                payload_pose.transform.rotation.x = quaternion[0]
                payload_pose.transform.rotation.y = quaternion[1]
                payload_pose.transform.rotation.z = quaternion[2]
                payload_pose.transform.rotation.w = quaternion[3] 

                # publish the payload pose's transformStamped message to the ROS topic
                self.pub_payload_pose.publish(payload_pose)  

                # publish payload pose wrt camera
                # get 3D attitude (in quaternions)
                x_camera = marker_pose_camera.transform.rotation.x
                y_camera = marker_pose_camera.transform.rotation.y
                z_camera = marker_pose_camera.transform.rotation.z
                w_camera = marker_pose_camera.transform.rotation.w

                # convert quaternions to a transformation matrix from the marker to the payload   
                T_im_camera = quaternion_matrix([x_camera, y_camera, z_camera, w_camera])

                # extract the translation from the marker to the payload frame and insert to transformation matrix
                T_im_camera[0,3] = marker_pose_camera.transform.translation.x
                T_im_camera[1,3] = marker_pose_camera.transform.translation.y
                T_im_camera[2,3] = marker_pose_camera.transform.translation.z

                # build the transformation matrix from the payload to the inertial frame
                T_ip_camera = np.dot(T_im_camera, self.T_mp[marker_id[0]])

                # extract the payload's roll, pitch & yaw from the rotation matrix
                euler_camera = euler_from_matrix(T_ip_camera[0:3,0:3], 'sxyz')
                # convert the marker's euler angles to quaternions
                quaternion_camera = quaternion_from_euler(euler_camera[0], euler_camera[1], euler_camera[2], 'sxyz')

                # copy the marker pose's transformStamped message to set up the payload pose message
                payload_pose_camera = copy.deepcopy(marker_pose_camera)

                # overwrite the position with the vector from the inertial frame to the payload frame
                payload_pose_camera.transform.translation.x = T_ip_camera[0,3]
                payload_pose_camera.transform.translation.y = T_ip_camera[1,3]
                payload_pose_camera.transform.translation.z = T_ip_camera[2,3]
                # overwrite the rotation with the quaternion from the inertial frame to the payload frame
                payload_pose_camera.transform.rotation.x = quaternion_camera[0]
                payload_pose_camera.transform.rotation.y = quaternion_camera[1]
                payload_pose_camera.transform.rotation.z = quaternion_camera[2]
                payload_pose_camera.transform.rotation.w = quaternion_camera[3] 

                # publish the payload pose's transformStamped message to the ROS topic
                self.pub_payload_pose_camera.publish(payload_pose_camera)  

            # else, this marker ID is not defined for the given payload, reject this detection as an outlier 
            else:
                print('Marker ID ' + str(marker_id[0]) + ' found, however this marker ID is undefined for this payload.')

            return
		
        """method to take the average pose from a set of measurements from the same image then publish the result"""
        def average_payload_pose(self):

            # copy the marker pose transformStamped message to set up the payload pose message
            payload_pose = copy.deepcopy(self._marker_pose)

            # overwrite the marker ID with the number of inliers:
            self._marker_pose.child_frame_id = str(len(self.T_ip_j))  
            # overwrite the position with the vector from the inertial frame to the payload frame
            payload_pose.transform.translation.x = np.mean(self.T_ip_j[:,0])
            payload_pose.transform.translation.y = np.mean(self.T_ip_j[:,1])
            payload_pose.transform.translation.z = np.mean(self.T_ip_j[:,2])
           
            # convert the payload's euler angles to quaternions
            euler = np.array([np.mean(self.T_ip_j[:,3]), np.mean(self.T_ip_j[:,4]), np.mean(self.T_ip_j[:,5])])
            quaternion = quaternion_from_euler(euler[0], euler[1], euler[2], 'sxyz')

            # overwrite the rotation with the quaternion from the inertial frame to the payload frame
            payload_pose.transform.rotation.x = quaternion[0]
            payload_pose.transform.rotation.y = quaternion[1]
            payload_pose.transform.rotation.z = quaternion[2]
            payload_pose.transform.rotation.w = quaternion[3] 

            # publish the average payload pose's transformStamped message to the ROS topic
            self.pub_ave_payload_pose.publish(payload_pose)  

            # print the average pose to the terminal for debugging
            # print('Estimated Payload Pose:')
            # print('X: ' + str(payload_pose.transform.translation.x) + ' Y: ' + str(payload_pose.transform.translation.y) + ' Z: ' + str(payload_pose.transform.translation.z))
            # print('Roll: ' + str(euler[0]) + ' Pitch: ' + str(euler[1]) + ' Yaw: ' + str(euler[2]))

            return  

        '''************************ End of Modification for Project Code *************************'''

if __name__=='__main__':
	import sys
	rospy.init_node('aruco_estimation')
	display = DroneVideoDisplay()

        rospy.spin()
	rospy.signal_shutdown('Great Flying!')
	sys.exit(status)
