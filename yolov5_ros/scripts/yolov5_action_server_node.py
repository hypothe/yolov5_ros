#! /usr/bin/env python

# apt-get install python-pip
# pip install subprocess32

import rospy

import actionlib

from darknet_ros_msgs.msgs import (
	CheckForObjectsAction,
	BoundingBox,
	BoundingBoxes
)

import subprocess32 as subprocess
import signal
import os
from cv_bridge import CvBridge, CvBridgeError

class YoloAction(objdect):

	def __init__(self, action_name, model_name, weight_name):

		self.action_name = action_name
		self.model_name = model_name
		self.weight_name = weight_name

		self.bridge = CvBridge()
		# TODO: take this from param server
		self.yolo_path = "ultralytics/yolov5"
		# TODO: take this from param server
		self.image_path = "../../yolov5/images/det.jpg"

		self.as_ = actionlib.SimpleActionServer(self.action_name, CheckForObjectsAction, auto_start = False)
		self.as_.register_goal_callback(self.detectCB)
		# self.as_.register_preempt_callback(self.preemptCB)
		
		self.yolo = subprocess.run(["python3", self.yolo_path, os.getpid()], check=True, input=PIPE, output=PIPE)
		out, _ = self.yolo.communicate()
		rospy.loginfo("Subprocess started ", out)
		# the subprocess informs this one once the results are ready
		signal.signal(signal.SIGUSR2, self.detectedImage)

		self.as_.start()

		# launch the python3 script
		

	def detectCB(self, goal):
		# retrieve image from goal
		# write on the pipe:
		## is there a way to automatically link the receival of a message on the pipe
		## to carrying out something?

		# TODO: split the callback saving the image in it and calling the subprocess
		# Then, in another function, iterate periodically (clock?) querying the state
		# of the detection (cool, but unnecessary)
		# 
		try:
			self.image = bridge.imgmsg_to_cv2(goal.image, encoding='passthrough')
		except CvBridgeError as e:
			rospy.logerr("%s: image conversion to cv2 failed" % e)

		# Save the image and inform the subprocess that it's ready
		cv2.imwrite(self.image_path, self.image)
		self.yolo.send_signal(signal.SIGUSR1)


	def detectedImage(self, signum, frame):
		detection, _ = self.yolo.communicate()
		rospy.loginfo("Received detection: \n%s" % detection)

		# TODO: parse detection and insert it into the result
		result = CheckForObjectsAction()
		# fill result
		self.as_.set_succeeded(result, text="Detection complete")


if __name__ == '__main__':
	rospy.init('yolov5_as')
	# TODO: take these values from param server
	action_name = rospy.get_name()+'/check_for_objects'
	model_name = "yolov5s_sciroc.yaml"
	weight_name = "yolov5s_sciroc.pt"
	as_ = YoloAction(actoin_name, model_name, weight_name)
	rospy.spin()