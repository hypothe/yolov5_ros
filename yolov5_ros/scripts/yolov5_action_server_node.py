#! /usr/bin/env python3

import rospy
import rospkg

import actionlib

from darknet_ros_msgs.msg import (
	CheckForObjectsAction,
	CheckForObjectsResult,
	BoundingBox,
	BoundingBoxes
)

import os
import sys
import cv2
from cv_bridge import CvBridge, CvBridgeError
# yolov5 detection
import numpy
import torch


class YoloDetector(object):
	def __init__(self, yolo_name, weight_path):
		self.yolo_name = yolo_name
		self.weight = weight_path

		try:
			self.model = torch.hub.load(self.yolo_name, 'custom', path = self.weight)  # local repo
			rospy.loginfo("Model loaded")
		except Exception as e:
			rospy.logerr("Error loading model from torch hub: ", e)
			rospy.shutdown()

	def detect(self, image):
		results = self.model(image, size=640)
		return results.pandas().xyxy[0].values

class YoloAction():

	def __init__(self, action_name, model_name, weight_name):

		self.action_name = action_name
		self.model_name = model_name
		self.weight_name = weight_name

		self.bridge = CvBridge()
		rospack = rospkg.RosPack()
		yolov5_path = os.path.dirname(rospack.get_path('yolov5_ros'))
		print(yolov5_path)

		wg1, wg2 = rospy.get_param("yolo/local/weight_path"), rospy.get_param("yolo/model/weight")
		self.weight_path = os.path.normpath(os.path.join(yolov5_path, wg1, wg2))
		print(self.weight_path)
		
		self.as_ = actionlib.SimpleActionServer(self.action_name, CheckForObjectsAction, execute_cb=self.detectCB, auto_start = False)

		self.yolo = YoloDetector(self.model_name, self.weight_path)

		self.as_.start()

	def detectCB(self, goal):
		rospy.loginfo("Goal received")
		
		try:
			self.image = self.bridge.imgmsg_to_cv2(goal.image, 'bgr8')
		except CvBridgeError as e:
			rospy.logerr("%s: image conversion to cv2 failed" % e)
			self.as_.set_aborted(text="Image conversion failed")
			return

		# Save the image and inform the subprocess that it's ready
		
		rospy.logdebug("Starting detection")

		results = self.yolo.detect(self.image)
		# parse detected bounding boxes
		bbs = BoundingBoxes()
		for ii, line in enumerate(results):
			bb = BoundingBox()
			bb.id = ii
			bb.xmin, bb.ymin, bb.xmax, bb.ymax = [int(round(x)) for x in line[:4]]
			bb.probability = line[4]
			bb.Class = line[-1]
			bbs.bounding_boxes.append(bb)
		
		result = CheckForObjectsResult()
		result.id = goal.id
		result.bounding_boxes = bbs
		self.as_.set_succeeded(result, text="Detection complete")



if __name__ == '__main__':
	rospy.init_node('yolov5_as')
	action_name = rospy.get_param("yolo/actions/yolo/topic")
	#model_name = "ultralytics/yolov5"
	model_name = rospy.get_param('yolo/model/name', 'ultralytics/yolov5')
	# weight_name = "yolov5s_sciroc.pt"
	#weight_name = "best.pt"
	weight_name = rospy.get_param('yolo/model/weight', 'best.pt')
	as_ = YoloAction(action_name, model_name, weight_name)
	rospy.spin()