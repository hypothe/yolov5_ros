#! /usr/bin/env python3

# pip install -r https://raw.githubusercontent.com/ultralytics/yolov5/master/requirements.txt


import signal
import os
import sys
import numpy
import torch

class YoloDetector(object):
	def __init__(self, yolo_name, weight_name, ppid):
		self.image_path = "../images/det.jpg"
		weight_path = os.path.join('/home/user/ws/src/yolov5_ros/yolov5/weights', weight_name)

		# if not os.path.isfile(weight_path):
		# 	print("No model definition found for %s", weight_path)
		# print(yolo_name)
		try:
			self.model = torch.hub.load('ultralytics/yolov5', 'yolov5s', force_reload=True)
			#self.model = torch.hub.load('ultralytics/yolov5', 'custom', path = weight_path, force_reload=True)  # local repo
			print("Model loaded")
		except Exception as e:
			print("Error loading model from torch hub: ", e)

		self.ppid = ppid
		signal.signal(signal.SIGUSR1, self.readImage)
		#timeout = 0.2
		while True:
			# signal.sigtimedwait(signal.SIGUSR1, timeout)
			signal.sigwait([signal.SIGUSR1])


	def readImage(self, signum, frame):
		image = cv2.imread(self.image_path)[..., ::-1] # BGR to RGB
		results = self.model(image, size=640)
		results.print() # should send on the pipe, right?
		# inform the parent the data is ready
		signal.pidfd_send_signal(self.ppid, signal.SIGUSR2)

if __name__ == '__main__':
	ppid = os.getppid()
	yolo_name = 'ultralytics/yolov5'
	yolo = YoloDetector(yolo_name, sys.argv[1], sys.argv[2])