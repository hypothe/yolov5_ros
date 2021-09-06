#! /usr/bin/env python3

import signal
import os
import sys
from cv_bridge import CvBridge, CvBridgeError

import torch

class YoloDetector(object):
	def __init__(self, yolo_name, weight_name, ppid):
		self.image_path = "../images/det.jpg"
		weight_path = os.path.join('../weights', weight_name)

		if not os.path.isfile(weight_path):
			print("No model definition found for %s", weight_path)

		try:
			self.model = torch.hub.load(yolo_name, 'custom', path = weigth_path, force_reload=True)  # local repo
			print("Model loaded")
		except:
			print("Error loading model from torch hub: ", sys.exc_info()[0])
			print("ERROR: unable to load model")

		self.ppid = ppid
		signal.signal(signal.SIGUSR1, self.readImage)
		#timeout = 0.2
		while True:
			# signal.sigtimedwait(signal.SIGUSR1, timeout)
			signal.sigwait(signal.SIGUSR1)


	def readImage(self, signum, frame):
		image = cv2.imread(self.image_path)[..., ::-1] # BGR to RGB
		results = self.model(image, size=640)
		results.print() # should send on the pipe, right?
		# inform the parent the data is ready
		signal.pidfd_send_signal(self.ppid, signal.SIGUSR2)

if __name__ == '__main__':
	ppid = os.getppid()
	yolo = YoloDetector(sys.argv[1], sys.argv[2], sys.argv[3])