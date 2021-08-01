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
import csv

# Import the two types of messages we're interested in
from sensor_msgs.msg import Image    	 # for receiving the video feed

# We need to use resource locking to handle synchronization between GUI thread and ROS topic callbacks
from threading import Lock

# The GUI libraries
from PySide import QtCore, QtGui
# from aer1217_ardrone_simulator.srv import *

# 2017-03-22 Import libraries from OpenCV
# OpenCV Bridge http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import TransformStamped, Twist
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_matrix, quaternion_from_euler, euler_from_matrix, euler_matrix
import numpy as np

# Some Constants
CONNECTION_CHECK_PERIOD = 250 #ms
GUI_UPDATE_PERIOD = 20 #ms
DETECT_RADIUS = 4 # the radius of the circle drawn when a tag is detected


class DroneVideoDisplay(QtGui.QMainWindow):
	def __init__(self):
		# Construct the parent class
		super(DroneVideoDisplay, self).__init__()

		# Setup our very basic GUI - a label which fills the whole window and holds our image
		self.setWindowTitle('Video Feed')
		self.imageBox = QtGui.QLabel(self)
		self.setCentralWidget(self.imageBox) 
		
		# Subscribe to the drone's video feed, calling self.ReceiveImage when a new frame is received
		self.subVideoBottom = rospy.Subscriber('/iris_0/bottom/image_raw/', Image, self.ReceiveImageBottom,queue_size=1)
		self.subVideoFront = rospy.Subscriber('/iris_0/front/image_raw/', Image, self.ReceiveImageFront,queue_size=1)

                # subscribe to drone's position and attitude feedback
                self.vicon_topic = '/gazebo_ground_truth_UAV0'
                self._vicon_msg = Odometry()
                self.sub_vicon = rospy.Subscriber(self.vicon_topic, Odometry, self._vicon_callback)

                # subscribe to individual ArUco marker pose estimations
                self.marker_topic = '/gazebo_estimate_marker_pose'
                self._marker_msg = TransformStamped()
                self.sub_marker_pose = rospy.Subscriber(self.marker_topic, TransformStamped, self._marker_callback)

                # subscribe to individual ArUco marker pose estimations wrt the drone
                self.marker_topic_camera = '/gazebo_estimate_marker_pose_camera'
                self._marker_msg_camera = TransformStamped()
                self.sub_marker_pose_camera = rospy.Subscriber(self.marker_topic_camera, TransformStamped, self._marker_callback_camera)

                # 0 = front camera first, 1 = bottom camera first  
		self.camera = 1 
		
		# Holds the image frame received from the drone and later processed by the GUI
		self.image = None
		self.imageLock = Lock()

		self.tags = []
		self.tagLock = Lock()
				
		# Holds the status message to be displayed on the next GUI update
		self.statusMessage = ''

		# Tracks whether we have received data since the last connection check
		# This works because data comes in at 50Hz but we're checking for a connection at 4Hz
		self.communicationSinceTimer = False
		self.connected = False

		# A timer to check whether we're still connected
		self.connectionTimer = QtCore.QTimer(self)
		self.connectionTimer.timeout.connect(self.ConnectionCallback)
		self.connectionTimer.start(CONNECTION_CHECK_PERIOD)
		
		# A timer to redraw the GUI
		self.redrawTimer = QtCore.QTimer(self)
		self.redrawTimer.timeout.connect(self.RedrawCallback)
		self.redrawTimer.start(GUI_UPDATE_PERIOD)
		
		# 2017-03-22 convert ROS images to OpenCV images
		self.bridge = CvBridge()

		# 2017-03-31 Lab 4 processing images variables
		self.processImages = True  #false by default		
		self.cv_output = None
		self.cv_img = None
		
		# rospy.Service('/ardrone/togglecam',ToggleCam,self.ToggleFrontBottomCamera)

                '''********************Project Setup Parameters Below*************************'''

                # use these variables to enable (True) or disable (False) marker identification
                self.find_markers_enabled = True

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
                self.r_bc = np.array([[0.0], [0.0125], [-0.025]])   
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
                self.T_mp = np.zeros((4,4,4))
                # marker ID 0 rotation and translation: 
                self.T_mp[[0],0:4,0:4] = np.eye(4)
                self.T_mp[[0],0:3,[3]] = np.array([4.5/19., -4.5/19., -0.005]) 
                # marker ID 1 rotation and translation:
                self.T_mp[[1],0:4,0:4] = np.eye(4)
                self.T_mp[[1],0:3,[3]] = np.array([-4.5/19., -4.5/19., -0.005]) 
                # marker ID 2 rotation and translation:
                self.T_mp[[2],0:4,0:4] = np.eye(4)
                self.T_mp[[2],0:3,[3]] = np.array([4.5/19., 4.5/19., -0.005]) 
                # marker ID 3 rotation and translation:
                self.T_mp[[3],0:4,0:4] = np.eye(4)
                self.T_mp[[3],0:3,[3]] = np.array([-4.5/19., 4.5/19., -0.005])  

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

                # sut up publisher to publish ROS images of processed images
                self.pub_image_processed = rospy.Publisher('/sensor_msgs/Image/processed', Image, queue_size = 1) 

                '''************************ End of Project Setup Parameters *************************'''

	# Called every CONNECTION_CHECK_PERIOD ms, if we haven't received anything since the last callback, will assume we are having network troubles and display a message in the status bar
	def ConnectionCallback(self):
		self.connected = self.communicationSinceTimer
		self.communicationSinceTimer = False

	def RedrawCallback(self):
		if self.image is not None:
			# We have some issues with locking between the display thread and the ros messaging thread due to the size of the image, so we need to lock the resources
			self.imageLock.acquire()
			try:			
					# Convert the ROS image into a QImage which we can display
					if self.processImages == False:
						image = QtGui.QPixmap.fromImage(QtGui.QImage(self.image.data, self.image.width, self.image.height, QtGui.QImage.Format_RGB888))						
					# display processed image when processing is enabled
					else:
						if self.cv_output is not None:					
							# convert from openCV output cv_output image back to ROS image (Optional for visualization purposes)
							img_msg = self.bridge.cv2_to_imgmsg(self.cv_output, encoding="bgr8")
							# convert to QImage to be displayed
							image = QtGui.QPixmap.fromImage(QtGui.QImage(img_msg.data, img_msg.width, img_msg.height, QtGui.QImage.Format_RGB888))
						else:
							image = QtGui.QPixmap.fromImage(QtGui.QImage(self.image.data, self.image.width, self.image.height, QtGui.QImage.Format_RGB888))		

			finally:
				self.imageLock.release()

			# We could  do more processing (eg OpenCV) hself.p_aere if we wanted to, but for now lets just display the window.
			image = image.scaledToWidth(480)
			self.resize(image.width(),image.height())
			self.imageBox.setPixmap(image)

		# Update the status bar to show the current drone status & battery level
		if self.image is None:
			self.statusBar().showMessage("Simulator not started")
		else:
			if self.camera == 0:
				self.statusBar().showMessage("Displaying front camera")
			if self.camera == 1:
				self.statusBar().showMessage("Displaying bottom camera")
			
        """method to receive the bottom camera images, call the image processing methods and then publish the results if any obstacles or landmarks are detected"""
	def ReceiveImageBottom(self, data):
		if self.camera == 1:
			# Indicate that new data has been received (thus we are connected)
			self.communicationSinceTimer = True
			# We have some issues with locking between the GUI update thread and the ROS messaging thread due to the size of the image, so we need to lock the resources
			self.imageLock.acquire()
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

                                        # if ArUco marker finding is enabled
                                        if self.find_markers_enabled:
                                            # call method to find the ids, rotation and translation of all markers found in the image in the camera frame
                                            self.find_markers(cv_image)  
                                            # if at least 1 ArUco marker was found
                                            if len(self.ids) > 0:
                                                # print divider for this image
                                                # print('---------------------------------------------------------')
                                                # compute the rotation and transformation matrices from the body to the inertial frame
                                                self.pose_to_transform_mat(image_pose) 
                                                # loop through every marker found 
                                                for i in range(len(self.tvec)):
                                                    # convert the marker's rotation and translation from the camera frame to the inertial frame
                                                    self.camera_to_inertial_frame(self.rvec[i], self.tvec[i], self.ids[i]) 
   
                                    except CvBridgeError as e:
                                        print "Image conversion failed: %s" % e
				# 2017-03-22 we do not recommend saving images in this function as it might cause huge latency
			finally:
				self.imageLock.release()
                        # increment sample counter for the next loop
                        self.count += 1

	def ReceiveImageFront(self,data):
		if self.camera == 0:
			# Indicate that new data has been received (thus we are connected)
			self.communicationSinceTimer = True

			# We have some issues with locking between the GUI update thread and the ROS messaging thread due to the size of the image, so we need to lock the resources
			self.imageLock.acquire()
			try:
				self.image = data # Save the ros image for processing by the display thread
				# 2017-03-22 we do not recommend saving images in this function as it might cause huge latency
			finally:
				self.imageLock.release()

        # vicon system callback method
        def _vicon_callback(self, msg):
            self._vicon_msg = msg
         
        # ArUco marker pose callback method
        def _marker_callback(self, msg):
            self._marker_msg = msg
            # call method to process the marker pose estimations as they are received 
            self.marker_to_payload_frame(msg, True)

        # ArUco marker pose wrt the drone callback method
        def _marker_callback_camera(self, msg):
            self._marker_msg_camera = msg
            # call method to process the marker pose estimations as they are received 
            self.marker_to_payload_frame(msg, False)

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

        """method to convert a coordinate frame (a point with a rotation) from the camera frame to the inertial frame"""
        def camera_to_inertial_frame(self, r_vec, t_vec, marker_id):
              
            # convert the translation vector to a (4x1) matrix for multiplication with the transformation matrices 
            r_pc = np.array([[t_vec[0][0]], [t_vec[0][1]], [t_vec[0][2]], [1]]) 
 
            # apply transformation matrices to convert the marker location from the camera to the inertial frame
            r_pi = np.dot(self.T_ib, (np.dot(self.T_bc, r_pc)))    
            # print the marker location (in the inertial frame) to the terminal for debugging   
            # print('Marker ' + str(marker_id[0]) + ' Found at: x: ' + str(r_pi[0,0]) + ' y: ' + str(r_pi[1,0]) + ' z: ' + str(r_pi[2,0]))          
 
            # convert the rotation vector to a rotation matrix from the payload to the camera frame
            C_cp, _ = cv2.Rodrigues(r_vec[0])   

            # compound the three rotation matrices to get the conversion from the payload to the inertial frame
            C_ip = np.dot((self.T_ib[0:3,0:3]), (np.dot(np.transpose(self.C_cb), C_cp))) 

            # extract the payload markers's roll, pitch & yaw from the rotation matrix
            euler = euler_from_matrix(C_ip, 'sxyz')       
            # print the marker's attitude (in the inertial frame) to the terminal for debugging 
            # print('Marker: roll: ' + str(euler[0]) + ' pitch: '  + str(euler[1]) + ' yaw: ' + str(euler[2])) 

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
            self._marker_pose.transform.translation.x = r_pi[0,0]
            self._marker_pose.transform.translation.y = r_pi[1,0]
            self._marker_pose.transform.translation.z = r_pi[2,0]
            # marker attitude in quaternions
            self._marker_pose.transform.rotation.x = quaternion[0]
            self._marker_pose.transform.rotation.y = quaternion[1]
            self._marker_pose.transform.rotation.z = quaternion[2]
            self._marker_pose.transform.rotation.w = quaternion[3]  

            # publish the result to a ROS topic
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
            marker_pose_camera.transform.translation.x = r_pc[0,0]
            marker_pose_camera.transform.translation.y = r_pc[1,0]
            marker_pose_camera.transform.translation.z = r_pc[2,0]
            # marker attitude in quaternions
            euler_camera = euler_from_matrix(C_cp, 'sxyz')   
            quaternion_camera = quaternion_from_euler(euler_camera[0], euler_camera[1], euler_camera[2], 'sxyz')  
            marker_pose_camera.transform.rotation.x = quaternion_camera[0]
            marker_pose_camera.transform.rotation.y = quaternion_camera[1]
            marker_pose_camera.transform.rotation.z = quaternion_camera[2]
            marker_pose_camera.transform.rotation.w = quaternion_camera[3]  
		
            self.pub_marker_pose_camera.publish(marker_pose_camera) 

            return

        '''method to process the individual ArUco marker pose estimations''' 
        def marker_to_payload_frame(self, marker_pose, wrt_inertial):
           # wrt_inertial: True: the marker pose is wrt inertial; False: the marker pose is wrt camera (drone)
       
           # get the ID of the detected marker 
           marker_id = int(marker_pose.child_frame_id)

           # if the marker's ID is within the known range
           if marker_id < len(self.T_mp): 

               # get 3D attitude (in quaternions)
               x = marker_pose.transform.rotation.x
               y = marker_pose.transform.rotation.y
               z = marker_pose.transform.rotation.z
               w = marker_pose.transform.rotation.w

               # convert quaternions to a transformation matrix from the marker to the payload   
               T_im = quaternion_matrix([x, y, z, w])

               # extract the translation from the marker to the payload frame and insert to transformation matrix
               T_im[0,3] = marker_pose.transform.translation.x
               T_im[1,3] = marker_pose.transform.translation.y
               T_im[2,3] = marker_pose.transform.translation.z

               # build the transformation matrix from the payload to the inertial frame
               T_ip = np.dot(T_im, self.T_mp[marker_id])

               # extract the payload's roll, pitch & yaw from the rotation matrix
               euler = euler_from_matrix(T_ip[0:3,0:3], 'sxyz')
               # convert the marker's euler angles to quaternions
               quaternion = quaternion_from_euler(euler[0], euler[1], euler[2], 'sxyz')

               # copy the marker pose's transformStamped message to set up the payload pose message
               payload_pose = marker_pose
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
               if wrt_inertial:
                   self.pub_payload_pose.publish(payload_pose) 
               else:
                   self.pub_payload_pose_camera.publish(payload_pose) 

           return 

		
        '''************************ End of Modification for Project Code *************************'''
		        
	# 2017-03-31 Lab 4 code ================================
	# the codes written here serve as a guideline, it is not required that you use the code. Feel free to modify.

	def EnableImageProcessing(self):  # called from KeyboardController Key_P
		self.processImages = True

	def DisableImageProcessing(self): # called from KeyboardController Key_P
		self.processImages = False

	def ToggleFrontBottomCamera(self,req):
		if (self.camera == 1):
			self.camera = 0
			return "Displaying front camera"
		else:
			self.camera = 1
			return "Displaying bottom camera"
	

if __name__=='__main__':
	import sys
	rospy.init_node('aruco_estimation')
	app = QtGui.QApplication(sys.argv)
	display = DroneVideoDisplay()
	display.show() 
	status = app.exec_()
	rospy.signal_shutdown('Great Flying!')
	sys.exit(status)
