#!/usr/bin/env python
# This file is responsible for bridging ROS to the FaceDetector class (built with Caffe)

from __future__ import division

import sys

import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from rail_face_detector.msg import Face, Detections

import face_detector

# Debug Helpers
FAIL_COLOR = '\033[91m'
ENDC_COLOR = '\033[0m'

def eprint(error):
	sys.stderr.write(
		FAIL_COLOR
		+ type(error).__name__
		+ ": "
		+ error.message
		+ ENDC_COLOR
	)
# End Debug Helpers


class FaceDetector(object):
	"""
	This class takes in image data and finds / annotates faces within the image
	"""

	def __init__(self):
		rospy.init_node('face_detector_node')
		self.faces = []
		self.keypoint_arrays = []
		self.bridge = CvBridge()
		self.face_detector = face_detector.FaceDetector()
		self.debug = rospy.get_param('~debug', default=False)
		self.image_sub_topic_name = rospy.get_param('~image_sub_topic_name', default='/kinect/qhd/image_color_rect')

	def _draw_bb(self, image, bounding_box, color):
		start_x = bounding_box['x']
		start_y = bounding_box['y']
		end_x = start_x + bounding_box['w']
		end_y = start_y + bounding_box['h']
		cv2.rectangle(image,
					  (start_x, start_y),
					  (end_x, end_y),
					  color=color,
					  thickness=3)
		return image

	def _parse_image(self, image_msg):
		"""
		Take in an image and draw a bounding box within it
		:param image_msg: Image data
		:return: None
		"""

		header = image_msg.header

		try:
			image_cv = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
		except CvBridgeError as e:
			print e
			return
		self.faces, self.keypoint_arrays = self.face_detector.find_faces(image_cv)

		if self.debug:
			for face, points in zip(self.faces, self.keypoint_arrays):
				x1 = int(face[0])
				y1 = int(face[1])
				x2 = int(face[2])
				y2 = int(face[3])
				nose = (int(points[2]), int(points[7]))
				x_percent = (nose[0] - x1) / float(x2 - x1)
				y_percent = (nose[1] - y1) / float(y2 - y1)
				c = (255, 255, 0)
				# Black box if looking left or right
				if x_percent < 0.35 or x_percent > 0.65:
					c = (0, 0, 0)
				# Black box if looking down
				if y_percent > 0.65:
					c = (0, 0, 0)

				image_cv = self._draw_bb(image_cv, {'x': x1,
													'y': y1,
													'w': x2-x1,
													'h': y2-y1}, c)

			try:
				image_msg = self.bridge.cv2_to_imgmsg(image_cv, "bgr8")
			except CvBridgeError as e:
				print e

			image_msg.header = header
			self.image_pub.publish(image_msg)

		# Instantiate detections object
		face_arr = Detections()
		face_arr.header = header
		# For each face / keypoint set found in the image:
		for face, points in zip(self.faces, self.keypoint_arrays):
			msg = Face()
			msg.top_left_x = int(face[0])
			msg.top_left_y = int(face[1])
			msg.bot_right_x = int(face[2])
			msg.bot_right_y = int(face[3])
			# Forward face detection
			# TODO: Improve this. Use keypoint relative grouping
			nose = (int(points[2]), int(points[7]))
			x_percent = (nose[0] - int(face[0])) / float(int(face[2]) - int(face[0]))
			y_percent = (nose[1] - int(face[1])) / float(int(face[3]) - int(face[1]))
			# Black box if looking left, right, or down
			if x_percent < 0.35 or x_percent > 0.65 or y_percent > 0.65:
				msg.face_forward = 0
			else:
				msg.face_forward = 1
			face_arr.faces.append(msg)

		self.face_pub.publish(face_arr)

	def run(self,
			pub_image_topic='/face_detector/debug/face_image',
			pub_face_topic='/face_detector/faces'):
		rospy.Subscriber(self.image_sub_topic_name, Image, self._parse_image) # subscribe to sub_image_topic and callback parse
		if self.debug:
			self.image_pub = rospy.Publisher(pub_image_topic, Image, queue_size=2) # image publisher
		self.face_pub = rospy.Publisher(pub_face_topic, Detections, queue_size=2) # faces publisher
		rospy.spin()

if __name__ == '__main__':
	try:
		detector = FaceDetector()
		detector.run()
	except rospy.ROSInterruptException:
		pass
