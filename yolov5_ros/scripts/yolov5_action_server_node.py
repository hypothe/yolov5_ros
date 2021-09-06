#! /usr/bin/env python

# apt-get install python-pip
# pip install subprocess32

import rospy

import actionlib

from darknet_ros_msgs.msg import (
	CheckForObjectsAction,
	BoundingBox,
	BoundingBoxes
)

import signal
import os
import sys
import select
import cv2
from cv_bridge import CvBridge, CvBridgeError

if os.name == 'posix' and sys.version_info[0] < 3:
	import subprocess32 as subprocess
else:
	import subprocess

os.environ["PYTHONUNBUFFERED"] = "1"

class YoloAction():

	def __init__(self, action_name, model_name, weight_name):

		self.action_name = action_name
		self.model_name = model_name
		self.weight_name = weight_name

		self.bridge = CvBridge()
		# TODO: take this from param server
		#self.yolo_path = "../../yolov5/scripts/yolov5_detect.py"
		self.yolo_path = "/home/user/ws/src/yolov5_ros/yolov5/scripts/yolov5_detect.py"
		# TODO: take this from param server
		self.image_path = "../../yolov5/images/det.jpg"

		self.as_ = actionlib.SimpleActionServer(self.action_name, CheckForObjectsAction, execute_cb=self.detectCB, auto_start = False)
		

		self.yolo = subprocess.Popen(["python3", self.yolo_path, self.weight_name], bufsize=0, stdout=subprocess.PIPE)
		#out, _ = self.yolo.communicate()
		out = self.yolo.stdout.readline()
		rospy.loginfo("Subprocess started %s" % out)
		# rospy.loginfo("Subprocess call")
		# the subprocess informs this one once the results are ready
		signal.signal(signal.SIGUSR2, self.detectedImage)
		# signal.siginterrupt(signal.SIGUSR2, False)


		self.as_.start()

		# launch the python3 script
	def __del__(self):
		self.yolo.kill()

		

	def detectCB(self, goal):
		rospy.loginfo("Goal received")
		# TODO: split the callback saving the image in it and calling the subprocess
		# Then, in another function, iterate periodically (clock?) querying the state
		# of the detection (cool, but unnecessary)
		
		# ##########
		# TMP: local test
		# try:
		#	self.image = bridge.imgmsg_to_cv2(goal.image, encoding='passthrough')
		# except CvBridgeError as e:
		#	rospy.logerr("%s: image conversion to cv2 failed" % e)
		self.image = cv2.imread(self.image_path)

		# Save the image and inform the subprocess that it's ready
		cv2.imwrite(self.image_path, self.image)

		self.got_detection = False
		self.detection = ''
		print("Sending and waiting for detection")
		self.yolo.send_signal(signal.SIGUSR1)

		# the check on self.got_detection is needed to know the the program un-paused due to the correct signal
		#while not rospy.is_shutdown() and not self.got_detection:
		#	if self.as_.is_preempt_requested():
		#		rospy.loginfo('%s: Preempted' % self.action_name)
		#		self.as_.set_preempted()
		#		success = False
		#		break
		#	else:
		#		print("Waiting for detection")
		#		signal.pause()
		#		print(self.got_detection)
		#	r.sleep()
		#print("out of the loooop")
		r = rospy.Rate(100)
		while not self.got_detection:
			if self.as_.is_preempt_requested():
				rospy.loginfo('%s: Preempted' % self.action_name)
				self.as_.set_preempted()
				success = False
				return
			
			r.sleep()

		# Not preempted, signal received
		print("'bout to start %s %s" % (type(self.yolo.stdout), self.yolo.stdout.fileno()))
		ready_stdout = [self.yolo.stdout.fileno()] # so it's not empty initially, not used elsewhere
		readlist = [self.yolo.stdout.fileno()]
		
		_ = self.detection + self.yolo.stdout.readline()
		bbs = BoundingBoxes()
		while ready_stdout:
			# self.detection = self.detection + self.yolo.stdout.readline()
			line = self.yolo.stdout.readline().split()
			bb = BoundingBox()
			print(line[1:-3])
			bb.id, bb.xmin, bb.xmax, bb.ymin, bb.ymax, bb.probability, _, bb.Class = line
			print(bb)
			print(line)
			ready_stdout, _, _ = select.select(readlist, [], [], 0)

		rospy.loginfo("Received detection: \n%s" % self.detection)
		# TODO: parse detection and insert it into the result
		result = CheckForObjectsAction()
		#print(result)
		# fill result
		self.as_.set_succeeded(result, text="Detection complete")


	def detectedImage(self, signum, frame):
		print("Yolov5 returned")
		self.got_detection = True



if __name__ == '__main__':
	rospy.init_node('yolov5_as')
	# TODO: take these values from param server
	action_name = rospy.get_name()+'/check_for_objects'
	model_name = "ultralytics/yolov5"
	# weight_name = "yolov5s_sciroc.pt"
	weight_name = "best.pt"
	as_ = YoloAction(action_name, model_name, weight_name)
	rospy.spin()