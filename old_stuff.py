# # Sentimental file, keeping old stuff around in case I want it later
#
# import cv2
# import face_detector
# import numpy as np
#
# capture = cv2.VideoCapture(0)
# finder = face_detector.FaceDetector()
# ret, frame = capture.read()
# eye_finder = cv2.CascadeClassifier("cascades/haarcascade_eye.xml")
#

### Haar Cascades for eyes for centering ###
# face_box = frame[y1:y2, x1:x2]
# eyes = eye_finder.detectMultiScale(face_box)
# eyes = np.array(eyes)
# # print eyes[:, 1] / float(y2-y1)
# # Ensure we have eyes to do the below operation:
# if len(eyes) > 2:
# 	# Remove eyes found on the lower half of the face
# 	eyes = eyes[(eyes[:, 1] / float(y2-y1)) < 0.5]
# for (ex, ey, ew, eh) in eyes:
# 	# print i, ':', ex / float(x2-x1)
# 	# x for forward facing: 0.1 - 0.2, 0.6 - 0.7
# 	# x for left facing: 0.29 - 0.34, 0.7 - 0.75
# 	# x for right facing: 0.0 - 0.09, 0.34-0.48
# 	pos_x = ex / float(x2 - x1)
# 	# pos_y = ey / float(y2 - y1)
# 	if 0.1 < pos_x < 0.2 or 0.6 < pos_x < 0.7 and len(eyes) > 1:
# 		print 'center'
# 	else:
# 		print 'not center'
# 	ex += x1
# 	ey += y1
# 	# Try to reject mouths...
# 	# if eh / float(y2-y1) > 0.3 or ew / float(x2-x1) > 0.3:
# 	# 	continue
# 	# elif eh / float(y2-y1) < 0.1 or ew / float(x2-x1) < 0.1:
# 	# 	continue
# 	# Debugging
# 	# print eh / float(y2-y1)
# 	# print ew / float(x2-x1)
# 	cv2.rectangle(frame, (ex, ey), (ex + ew, ey + eh), (255, 0, 0), 1)


### KEYPOINT DRAWING ###
# Drawing on keypoints for debugging
# font = cv2.FONT_HERSHEY_SIMPLEX
# X coordinates are all in the first half of the array, Y coordinates are in the second. no idea why
# half_points_len = len(points) / 2
# for i in range(half_points_len):
# 	me_point = (int(points[i]), int(points[i + half_points_len]))
# 	x_percent = (me_point[0] - x1) / float(x2-x1)
# 	y_percent = (me_point[1] - y1) / float(y2-y1)
# 	cv2.circle(frame, me_point, 3, (255, 0, 0), 1)
# 	# cv2.putText(frame, str(me_point), me_point, font, 1, (255, 255, 255), 2)
# 	cv2.putText(frame, "{:.3f}, {:.3f}".format(x_percent, y_percent), me_point, font, 1, (255, 255, 255), 2)


### PnP Computation ###
# size = frame.shape
# # Camera internals
# focal_length = size[1]
# center = (size[1] / 2, size[0] / 2)
# camera_mat = np.array(
# 	[[focal_length, 0, center[0]],
# 	 [0, focal_length, center[1]],
# 	 [0, 0, 1]], dtype="double"
# )
# # 3D model points.
# model_p = np.array([
# 	(0.0, 0.0, 0.0),  # Nose tip
# 	(-225.0, 170.0, -135.0),  # Left eye left corner
# 	(225.0, 170.0, -135.0),  # Right eye right corne
# 	(-150.0, -150.0, -125.0),  # Left Mouth corner
# 	(150.0, -150.0, -125.0)  # Right mouth corner
# ])
# nose = (int(points[2]), int(points[7]))
# right_eye = (int(points[0]), int(points[5]))
# left_eye = (int(points[1]), int(points[6]))
# right_mouth = (int(points[3]), int(points[8]))
# left_mouth = (int(points[4]), int(points[9]))
# image_p = np.array([nose, left_eye, right_eye, left_mouth, right_mouth], dtype="double")
#
# dist = np.zeros((4, 1))
# (success, rotation_vector, translation_vector) = cv2.solvePnP(model_p, image_p, camera_mat, distCoeffs=dist)
# font = cv2.FONT_HERSHEY_SIMPLEX
# # X coordinates are all in the first half of the array, Y coordinates are in the second. no idea why
# cv2.putText(frame, "{:.3f}, {:.3f}, {:.3f}".format(rotation_vector[0][0],
#                                                    rotation_vector[1][0],
#                                                    rotation_vector[2][0]), (0, 50), font, 1, (255, 255, 255), 2)

##### OLD PERSON MARKER HELPER ANNOTATOR THING ##########
# class PersonMarkerHelper(object):
#
# 	def _publish_person_markers(self):
# 		"""
# 		Publish markers for the person
# 		"""
# 		if self.person is None or self.person.get('body_x') is None:
# 			return
#
# 		marker = Marker()
#
# 		marker.header.frame_id = '/base_link'
# 		marker.header.stamp = rospy.Time(0)
# 		marker.ns = 'annotator'
# 		marker.type = Marker.ARROW
# 		marker.lifetime = rospy.Duration(1.0)
# 		marker.scale.x = 0.5
# 		marker.scale.y = 0.1
# 		marker.scale.z = 0.1
# 		marker.color.r = 1.0
# 		marker.color.a = 1.0
#
# 		marker.pose.position.x = self.person.get('body_x')
# 		marker.pose.position.y = self.person.get('body_y')
# 		marker.pose.position.z = self.person.get('body_z')
#
# 		if self.person.get('body_a') is not None:
# 			marker.pose.orientation.x = self.person.get('body_a')
# 			marker.pose.orientation.y = self.person.get('body_b')
# 			marker.pose.orientation.z = self.person.get('body_c')
# 			marker.pose.orientation.w = self.person.get('body_d')
# 		else:
# 			marker.pose.orientation.w = 1.0
#
# 		self.marker_pub.publish(marker)
#
# 	def run(self,
# 			sub_image_topic='/camera/rgb/image_rect_color',
# 			sub_image_info_topic='/camera/rgb/camera_info',
# 			sub_marker_update_topic='/tilt_controller/state',
# 			pub_image_topic='/annotator/person_image',
# 			pub_image_info_topic='/annotator/camera_info',
# 			pub_marker_topic='/annotator/person_marker'):
# 		rospy.Subscriber(sub_image_topic, Image, self._parse_image)
# 		rospy.Subscriber(sub_image_info_topic, CameraInfo, self._camera_info)
# 		rospy.Subscriber(sub_marker_update_topic, JointState, self._update_marker)
#
# 		self.image_pub = rospy.Publisher(pub_image_topic, Image, queue_size=2)
# 		self.image_info_pub = rospy.Publisher(pub_image_info_topic, CameraInfo, queue_size=2)
# 		self.marker_pub = rospy.Publisher(pub_marker_topic, Marker, queue_size=2)
# 		# rospy.spin()
# 	"""
# 	This class takes in a canonical person object and publishes that data to
# 	the appropriate topics so that person can be visualized
# 	"""
# 	def __init__(self):
# 		self.person = None
# 		self.person_datastream = None
# 		self.bridge = CvBridge()
# 	def _parse_image(self, image_msg):
# 		"""
# 		Take an image and draw a bounding box within it
# 		"""
# 		if (self.datastream_index < len(self.person_datastream)
# 			and dt.fromtimestamp(image_msg.header.stamp.to_time()) > self.person_datastream[self.datastream_index]['timestamp']):
# 			self.person = self.person_datastream[self.datastream_index]['obj']
# 			self.datastream_index += 1
#
# 		if self.person is not None and (self.person.get('body_bb_x') is not None or self.person.get('face_bb_x') is not None):
# 			image_ros = image_msg
#
# 			try:
# 				image_cv = self.bridge.imgmsg_to_cv2(image_ros, "bgr8")
# 			except CvBridgeError as e:
# 				print e
# 				return
#
# 			if self.person.get('body_bb_x') is not None:
# 				image_cv = self._draw_bb(image_cv, {
# 					'x': self.person['body_bb_x'],
# 					'y': self.person['body_bb_y'],
# 					'width': self.person['body_bb_width'],
# 					'height': self.person['body_bb_height']
# 				}, (255,0,0))
#
# 			if self.person.get('face_bb_x') is not None:
# 				image_cv = self._draw_bb(image_cv, {
# 					'x': self.person['face_bb_x'],
# 					'y': self.person['face_bb_y'],
# 					'width': self.person['face_bb_width'],
# 					'height': self.person['face_bb_height']
# 				}, (0, 255, 0))
#
# 			try:
# 				image_msg = self.bridge.cv2_to_imgmsg(image_cv, "bgr8")
# 				image_msg.header = image_ros.header
# 			except CvBridgeError as e:
# 				print e
#
# 		self.image_pub.publish(image_msg)
#
# 	def _camera_info(self, data):
# 		self.image_info_pub.publish(data)
#
# 	def _draw_bb(self, image, bounding_box, color):
# 		cv2.rectangle(image,
# 					  (bounding_box['x'], bounding_box['y']),
# 					  (bounding_box['x']+bounding_box['width'], bounding_box['y']+bounding_box['height']),
# 					  color,
# 					  3)
# 		return image
#
# 	def _update_marker(self, data):
# 		# Leave the update of the timestamp to the camera
# 		self._publish_person_markers()
#
# 	def set_person_datastream(self, datastream):
# 		"""
# 		Set to None to prevent images from being published
# 		"""
# 		self.person_datastream = datastream
#
# 		if datastream is None:
# 			self.datastream_index = -1
# 		else:
# 			self.datastream_index = 1
# 			self.person = self.person_datastream[0]['obj']


#### MARKER STUFF FOR FACES ? #######
# def _update_marker(self, data):
# 	self._publish_face_markers()

# def _publish_face_markers(self):
# 	"""
# 	Publish markers for the face
# 	:return: None
# 	"""
# 	if self.faces is None:
# 		return
#
# 	marker = Marker()
#
# 	marker.header.frame_id = '/base_link'
# 	marker.header.stamp = rospy.Time(0)
# 	marker.ns = 'annotator'
# 	marker.type = Marker.ARROW
# 	marker.lifetime = rospy.Duration(1.0)
# 	marker.scale.x = 0.5
# 	marker.scale.y = 0.1
# 	marker.scale.z = 0.1
# 	marker.color.r = 1.0
# 	marker.color.a = 1.0
# 	# TODO: figure out what this is, and how to get it to work. With multiple faces, we have multiple markers?
# 	marker.pose.position.x = self.faces.get('body_x')
# 	marker.pose.position.y = self.faces.get('body_y')
# 	marker.pose.position.z = self.faces.get('body_z')
#
# 	if self.faces.get('body_a') is not None:
# 		marker.pose.orientation.x = self.faces.get('body_a')
# 		marker.pose.orientation.y = self.faces.get('body_b')
# 		marker.pose.orientation.z = self.faces.get('body_c')
# 		marker.pose.orientation.w = self.faces.get('body_d')
# 	else:
# 		marker.pose.orientation.w = 1.0
#
# 	self.marker_pub.publish(marker)