#!/usr/bin/python

import cv2
import rospy
import numpy as np
from openpose_ros_msgs.msg import *
from sensor_msgs.msg import CameraInfo


class Detections(object):

	def __init__(self):

		self.humans = [None]*3
		self.K = None

		rospy.Subscriber("/camera/depth/camera_info", CameraInfo, self.get_caminfo)
		rospy.Subscriber("/openpose_ros/human_list3D", OpenPoseHumanList3D, self.get_points)

	def get_caminfo(self, msg):
		self.K = msg.K

	def get_points(self, msg):

		for ID, person in enumerate(msg.human_list):
			if ID > 2:
				continue

			self.humans[ID] = Human()

			if (self.humans[ID].body_part[0]):
				for i, body_part in enumerate(person.body_key_points_with_prob):
					if body_part.prob > 0.0:
						key_pt = Points(body_part, self.K)
						self.humans[ID].body_keys[i] = key_pt

			if (self.humans[ID].body_part[1]):
				for i, body_part in enumerate(person.left_hand_key_points_with_prob):
					if body_part.prob > 0.0:
						key_pt = Points(body_part, self.K)
						self.humans[ID].left_hand_keys[i] = key_pt

			if (self.humans[ID].body_part[2]):
				for i, body_part in enumerate(person.right_hand_key_points_with_prob):
					if body_part.prob > 0.0:
						key_pt = Points(body_part, self.K)
						self.humans[ID].right_hand_keys[i] = key_pt
			
			if (self.humans[ID].body_part[3]):
				for i, body_part in enumerate(person.face_key_points_with_prob):
					if body_part.prob > 0.0:
						key_pt = Points(body_part, self.K)
						self.humans[ID].face_keys[i] = key_pt

class Human(object):

	def __init__(self):

		self.left_hand_keys = [None]*21
		self.right_hand_keys = [None]*21
		self.body_keys = [None]*25
		self.face_keys = [None]*70
		self.body_part = [False, True, False, False]  #[Body, Left, Right, Face]
	

class Points(object):

	def __init__(self, pointwithprob, K_mat):

		self.x_pixel = pointwithprob.x
		self.y_pixel = pointwithprob.y

		if(pointwithprob.z > 100.0):
			self.z = pointwithprob.z/1000.0
		else:
			self.z = pointwithprob.z
		
		self.x = None
		self.y = None
		self.prob = pointwithprob.prob

		self.fx = K_mat[0]
		self.fy = K_mat[4]
		self.cx = K_mat[2]
		self.cy = K_mat[5]

		self.project_pixels()

	def project_pixels(self):

		if (self.fx == None or self.fy == None or self.cx == None or self.cy == None):
			print("Invalid Camera Parameters!!!")
			exit(-1)
	
		else:
			self.x = (self.x_pixel - self.cx) * (self.z / (self.fx))
			self.y = (self.y_pixel - self.cy) * (self.z / (self.fy))


if __name__ == "__main__":

	rospy.init_node('openpose_py_parser', anonymous=True)
	rate = rospy.Rate(100)
	det1 = Detections()

	while not rospy.is_shutdown():

		if (det1.humans[0] != None):
			print(det1.humans[0].left_hand_keys[0].x)

		rate.sleep()

