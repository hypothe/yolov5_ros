#! /usr/bin/env python3

# pip install -r https://raw.githubusercontent.com/ultralytics/yolov5/master/requirements.txt


import signal
import os
import sys
import numpy
import torch
import cv2

class YoloDetector(object):
	def __init__(self, yolo_name, weight_name):
		self.image_path = "/home/user/ws/src/yolov5_ros/yolov5/images/det.jpg"
		weight_path = os.path.join('/home/user/ws/src/yolov5_ros/yolov5/weights', weight_name)

		# if not os.path.isfile(weight_path):
		# 	print("No model definition found for %s", weight_path)
		# print(yolo_name)
		try:
			self.model = torch.hub.load('ultralytics/yolov5', 'yolov5s', force_reload=False)
			#self.model = torch.hub.load('ultralytics/yolov5', 'custom', path = weight_path)  # local repo
			print("Model loaded")
			sys.stdout.flush()
		except Exception as e:
			print("Error loading model from torch hub: ", e)
			sys.stdout.flush()

		signal.signal(signal.SIGUSR1, self.readImage)
		signal.siginterrupt(signal.SIGUSR1, False)

		self.wait_for_request()
	
	def wait_for_request(self):
		while True:
			signal.pause()


	def readImage(self, signum, frame):
		image = cv2.imread(self.image_path)[..., ::-1] # BGR to RGB
		results = self.model(image, size=640)
		#results.pandas().xyxy[0] # should send on the pipe, right?
		print(results.pandas().xyxy[0]) # should send on the pipe, right?
		#sys.stdout.flush()

		# inform the parent the data is ready
		os.kill(os.getppid(), signal.SIGUSR2)

if __name__ == '__main__':
	ppid = os.getppid()
	yolo_name = 'ultralytics/yolov5'
	yolo = YoloDetector(yolo_name, sys.argv[1])