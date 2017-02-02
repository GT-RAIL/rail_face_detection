#!/usr/bin/env python
# This file is responsible for bridging ROS to the FaceDetector class (built with Caffe)

from __future__ import division

import sys

import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo

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
		self.faces = []
		self.keypoint_arrays = []
		self.image_datastream = None
		self.input_image = None
		self.bridge = CvBridge()
		self.face_detector = face_detector.FaceDetector()
		rospy.init_node('face_detector')

	def _camera_info(self, data):
		self.image_info_pub.publish(data)

	def set_faces_datastream(self, datastream):
		"""
		Set to None to prevent images from being published
		"""
		self.image_datastream = datastream

		if datastream is None:
			self.datastream_index = -1
		else:
			self.datastream_index = 1
			self.faces = self.image_datastream[0]['obj']

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
		# If we're on a new image, increment datastream index and move on to a new faces photo object?
		# if (self.datastream_index < len(self.image_datastream)) and \
		# 		(dt.fromtimestamp(image_msg.header.stamp.to_time()) > self.image_datastream[self.datastream_index]['timestamp']):
		# 	self.input_image = self.image_datastream[self.datastream_index]['obj']
		# 	self.datastream_index += 1

		# TODO: for each of the below conditionals and detector: change self.input_image to image_msg?
		self.faces, self.keypoint_arrays = self.face_detector.find_faces(image_msg)

		if len(self.faces) > 0:
			image_ros = image_msg

		try:
			image_cv = self.bridge.imgmsg_to_cv2(image_ros, "bgr8")
		except CvBridgeError as e:
			print e
			return

		# TODO: it saves a bit of computation to just pass the 4 points instead of calculating width to recalculate those points
		# TODO: Below I am iterating and drawing bounding boxes. Should I just be passing on points and the center/not_center boolean?
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
			image_msg.header = image_ros.header
		except CvBridgeError as e:
			print e

		self.image_pub.publish(image_msg)


	# TODO do I need to subscribe and publish camera_info if I'm not using it?
	def run(self,
			sub_image_topic='/camera/rgb/image_rect_color',
			sub_image_info_topic='/camera/rgb/camera_info',
			pub_image_topic='annotator/face_image',
			pub_image_info_topic='annotator/camera_info'):
		rospy.Subscriber(sub_image_topic, Image, self._parse_image) # subscribe to sub_image_topic and callback parse
		rospy.Subscriber(sub_image_info_topic, CameraInfo, self._camera_info) # subscribe to image_info for camera info
		self.image_pub = rospy.Publisher(pub_image_topic, Image, queue_size=2) # image publisher
		self.image_info_pub = rospy.Publisher(pub_image_info_topic, CameraInfo, queue_size=2) # camera info publisher
		rospy.spin()

if __name__ == '__main__':
	try:
		detector = FaceDetector()
		detector.run()
	except rospy.ROSInterruptException:
		pass
