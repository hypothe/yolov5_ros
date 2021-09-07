#! /usr/bin/env python

# apt-get install python-pip
# pip install subprocess32

import rospy
import rospkg

import actionlib

from darknet_ros_msgs.msg import (
	CheckForObjectsAction,
	CheckForObjectsResult,
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
		rospack = rospkg.RosPack()
		yolov5_path = os.path.dirname(rospack.get_path('yolov5_ros'))
		print(yolov5_path)

		# cwd = os.path.dirname(os.path.realpath(__file__))

		pt1, pt2 = rospy.get_param("yolo/local/yolo_path"), rospy.get_param("yolo/model/exec")
		self.yolo_path = os.path.normpath(os.path.join(yolov5_path, pt1, pt2))
		print(self.yolo_path)

		im1, im2 = rospy.get_param("yolo/local/image_path"), rospy.get_param("yolo/local/image")
		self.image_path = os.path.normpath(os.path.join(yolov5_path, im1, im2))
		print(self.image_path)

		wg1, wg2 = rospy.get_param("yolo/local/weight_path"), rospy.get_param("yolo/model/weight")
		self.weight_path = os.path.normpath(os.path.join(yolov5_path, wg1, wg2))
		print(self.weight_path)
		
		self.as_ = actionlib.SimpleActionServer(self.action_name, CheckForObjectsAction, execute_cb=self.detectCB, auto_start = False)

		self.yolo = subprocess.Popen(["python3", self.yolo_path, self.model_name, self.image_path, self.weight_path], bufsize=0, stdout=subprocess.PIPE)
		out = self.yolo.stdout.readline()
		rospy.loginfo("Subprocess started %s" % out)
		# the subprocess informs this one once the results are ready
		signal.signal(signal.SIGUSR2, self.detectedImage)

		self.as_.start()

	def __del__(self):
		self.yolo.kill()

	def detectCB(self, goal):
		rospy.loginfo("Goal received")
		
		try:
			self.image = self.bridge.imgmsg_to_cv2(goal.image, 'bgr8')
			width, height, _ = self.image.shape
		except CvBridgeError as e:
			rospy.logerr("%s: image conversion to cv2 failed" % e)

		# Save the image and inform the subprocess that it's ready
		cv2.imwrite(self.image_path, self.image)

		self.got_detection = False
		self.detection = ''
		print("Sending and waiting for detection")
		self.yolo.send_signal(signal.SIGUSR1)

		r = rospy.Rate(100)
		while not self.got_detection:
			if self.as_.is_preempt_requested():
				rospy.loginfo('%s: Preempted' % self.action_name)
				self.as_.set_preempted()
				success = False
				return
			
			r.sleep()

		# Not preempted, signal received
		ready_stdout = [self.yolo.stdout.fileno()] # so it's not empty initially, not used elsewhere
		readlist = [self.yolo.stdout.fileno()]
		
		# _ = self.detection + self.yolo.stdout.readline()
		bbs = BoundingBoxes()
		while ready_stdout:
			line = self.yolo.stdout.readline().split()
			try:
				int(line[0])
			except ValueError:
				pass
			else:
				bb = BoundingBox()
				bb.id, bb.xmin, bb.ymin, bb.xmax, bb.ymax, bb.probability = [float(x) for x in line[:-2]]
				bb.Class = line[-1]
				
				bbs.bounding_boxes.append(bb)
			ready_stdout, _, _ = select.select(readlist, [], [], 0)

		result = CheckForObjectsResult()
		result.id = goal.id
		result.bounding_boxes = bbs
		self.as_.set_succeeded(result, text="Detection complete")


	def detectedImage(self, signum, frame):
		print("Yolov5 returned")
		self.got_detection = True



if __name__ == '__main__':
	rospy.init_node('yolov5_as')
	# TODO: take these values from param server
	action_name = rospy.get_param("yolo/actions/yolo/topic")
	#model_name = "ultralytics/yolov5"
	model_name = rospy.get_param('yolo/model/name', 'ultralytics/yolov5')
	# weight_name = "yolov5s_sciroc.pt"
	#weight_name = "best.pt"
	weight_name = rospy.get_param('yolo/model/weight', 'best.pt')
	as_ = YoloAction(action_name, model_name, weight_name)
	rospy.spin()