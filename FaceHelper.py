#!/usr/bin/env python
# This file is responsible for parsing data from the bagfile player.py and the
# automatic annotator reader.py to combine the info and presenting it to the
# human in a manner that allows the human to automatically parse what's going on
# and correctly annotate the data

from __future__ import division

import rospy
import datetime
import sys
import glob
import cv2
import threading
import readline
import signal
import os
import time
import pickle
import face_detector

from datetime import datetime as dt

# Classes from the annotator module
from reader import TimeStampedPeopleDataStream
from player import BagfilePlayer

from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from visualization_msgs.msg import Marker
from std_msgs.msg import Header
from dynamixel_msgs.msg import JointState

# Classes

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


class FaceMarkerHelper(object):
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

	def _update_marker(self, data):
		self._publish_face_markers()

	def _publish_face_markers(self):
		"""
		Publish markers for the face
		:return: None
		"""
		if self.faces is None:
			return

		marker = Marker()

		marker.header.frame_id = '/base_link'
		marker.header.stamp = rospy.Time(0)
		marker.ns = 'annotator'
		marker.type = Marker.ARROW
		marker.lifetime = rospy.Duration(1.0)
		marker.scale.x = 0.5
		marker.scale.y = 0.1
		marker.scale.z = 0.1
		marker.color.r = 1.0
		marker.color.a = 1.0
		# TODO: figure out what this is, and how to get it to work. With multiple faces, we have multiple markers?
		marker.pose.position.x = self.faces.get('body_x')
		marker.pose.position.y = self.faces.get('body_y')
		marker.pose.position.z = self.faces.get('body_z')

		if self.faces.get('body_a') is not None:
			marker.pose.orientation.x = self.faces.get('body_a')
			marker.pose.orientation.y = self.faces.get('body_b')
			marker.pose.orientation.z = self.faces.get('body_c')
			marker.pose.orientation.w = self.faces.get('body_d')
		else:
			marker.pose.orientation.w = 1.0

		self.marker_pub.publish(marker)

	def run(self,
			sub_image_topic='/camera/rgb/image_rect_color',
			sub_image_info_topic='/camera/rgb/camera_info',
			sub_marker_update_topic='/tilt_controller/state',
			pub_image_topic='annotator/person_image',
			pub_image_info_topic='annotator/camera_info',
			pub_marker_topic='/annotator/person_marker'):
		rospy.Subscriber(sub_image_topic, Image, self._parse_image) # subscribe to sub_image_topic and callback parse
		rospy.Subscriber(sub_image_info_topic, CameraInfo, self._camera_info) # subscribe to image_info for camera info
		rospy.Subscriber(sub_marker_update_topic, JointState, self._update_marker) # subscribe to marker update and callback marker update

		self.image_pub = rospy.Publisher(pub_image_topic, Image, queue_size=2) # image publisher
		self.image_info_pub = rospy.Publisher(pub_image_info_topic, CameraInfo, queue_size=2) # camera info publisher
		self.marker_pub = rospy.Publisher(pub_marker_topic, Marker, queue_size=2)
		# rospy.spin()


class PersonMarkerHelper(object):

	def _publish_person_markers(self):
		"""
		Publish markers for the person
		"""
		if self.person is None or self.person.get('body_x') is None:
			return

		marker = Marker()

		marker.header.frame_id = '/base_link'
		marker.header.stamp = rospy.Time(0)
		marker.ns = 'annotator'
		marker.type = Marker.ARROW
		marker.lifetime = rospy.Duration(1.0)
		marker.scale.x = 0.5
		marker.scale.y = 0.1
		marker.scale.z = 0.1
		marker.color.r = 1.0
		marker.color.a = 1.0

		marker.pose.position.x = self.person.get('body_x')
		marker.pose.position.y = self.person.get('body_y')
		marker.pose.position.z = self.person.get('body_z')

		if self.person.get('body_a') is not None:
			marker.pose.orientation.x = self.person.get('body_a')
			marker.pose.orientation.y = self.person.get('body_b')
			marker.pose.orientation.z = self.person.get('body_c')
			marker.pose.orientation.w = self.person.get('body_d')
		else:
			marker.pose.orientation.w = 1.0

		self.marker_pub.publish(marker)

	def run(self,
			sub_image_topic='/camera/rgb/image_rect_color',
			sub_image_info_topic='/camera/rgb/camera_info',
			sub_marker_update_topic='/tilt_controller/state',
			pub_image_topic='/annotator/person_image',
			pub_image_info_topic='/annotator/camera_info',
			pub_marker_topic='/annotator/person_marker'):
		rospy.Subscriber(sub_image_topic, Image, self._parse_image)
		rospy.Subscriber(sub_image_info_topic, CameraInfo, self._camera_info)
		rospy.Subscriber(sub_marker_update_topic, JointState, self._update_marker)

		self.image_pub = rospy.Publisher(pub_image_topic, Image, queue_size=2)
		self.image_info_pub = rospy.Publisher(pub_image_info_topic, CameraInfo, queue_size=2)
		self.marker_pub = rospy.Publisher(pub_marker_topic, Marker, queue_size=2)
		# rospy.spin()
	"""
	This class takes in a canonical person object and publishes that data to
	the appropriate topics so that person can be visualized
	"""
	def __init__(self):
		self.person = None
		self.person_datastream = None
		self.bridge = CvBridge()
	def _parse_image(self, image_msg):
		"""
		Take an image and draw a bounding box within it
		"""
		if (self.datastream_index < len(self.person_datastream)
			and dt.fromtimestamp(image_msg.header.stamp.to_time()) > self.person_datastream[self.datastream_index]['timestamp']):
			self.person = self.person_datastream[self.datastream_index]['obj']
			self.datastream_index += 1

		if self.person is not None and (self.person.get('body_bb_x') is not None or self.person.get('face_bb_x') is not None):
			image_ros = image_msg

			try:
				image_cv = self.bridge.imgmsg_to_cv2(image_ros, "bgr8")
			except CvBridgeError as e:
				print e
				return

			if self.person.get('body_bb_x') is not None:
				image_cv = self._draw_bb(image_cv, {
					'x': self.person['body_bb_x'],
					'y': self.person['body_bb_y'],
					'width': self.person['body_bb_width'],
					'height': self.person['body_bb_height']
				}, (255,0,0))

			if self.person.get('face_bb_x') is not None:
				image_cv = self._draw_bb(image_cv, {
					'x': self.person['face_bb_x'],
					'y': self.person['face_bb_y'],
					'width': self.person['face_bb_width'],
					'height': self.person['face_bb_height']
				}, (0, 255, 0))

			try:
				image_msg = self.bridge.cv2_to_imgmsg(image_cv, "bgr8")
				image_msg.header = image_ros.header
			except CvBridgeError as e:
				print e

		self.image_pub.publish(image_msg)

	def _camera_info(self, data):
		self.image_info_pub.publish(data)

	def _draw_bb(self, image, bounding_box, color):
		cv2.rectangle(image,
					  (bounding_box['x'], bounding_box['y']),
					  (bounding_box['x']+bounding_box['width'], bounding_box['y']+bounding_box['height']),
					  color,
					  3)
		return image

	def _update_marker(self, data):
		# Leave the update of the timestamp to the camera
		self._publish_person_markers()

	def set_person_datastream(self, datastream):
		"""
		Set to None to prevent images from being published
		"""
		self.person_datastream = datastream

		if datastream is None:
			self.datastream_index = -1
		else:
			self.datastream_index = 1
			self.person = self.person_datastream[0]['obj']

class AnnotationHelper(object):
	"""
	This class instantiates a reader and a player and then uses them to handle
	the data streams such that the human can reliably annotate the data
	"""

	def __init__(self,
				 bagfile_glob,
				 bagfile_folder_name,
				 bagfile_pickle_glob,
				 sighthound_pickle_glob=None,
				 annotation_filename='annotations-'+time.strftime('%Y-%m-%dT%H-%M-%S')+'.pickle',
				 annotated_data_folder='data/reannotations/',
				 bagfile_segment_duration=0.25):
		rospy.init_node('person_annotator')

		# First initialize the bag file player
		self.bagfile_player = BagfilePlayer(bagfile_glob, bagfile_folder_name, bagfile_segment_duration)

		# Then the data stream which contains the auto-annotated data
		pickle_files_glob = glob.glob(bagfile_pickle_glob)
		if sighthound_pickle_glob is not None:
			pickle_files_glob.extend(glob.glob(sighthound_pickle_glob))
		if len(pickle_files_glob) != 4 and len(pickle_files_glob) != 5:
			raise Exception("There is something wrong with the glob expressions")
		self.datastream = TimeStampedPeopleDataStream(*sorted(pickle_files_glob))
		self.datastream.parse()

		# Sanity check the timestamps
		self._sanity_check_timestamps()

		# Initialize the marker
		self.marker_helper = PersonMarkerHelper()

		# Set the private variables
		self.segment_duration = bagfile_segment_duration
		self.data_time_window = TimeStampedPeopleDataStream.TIME_WINDOW
		self.segment_duration_td = datetime.timedelta(seconds=self.segment_duration)

		# Prepare the output files
		self.annotated_data_file = os.path.join(annotated_data_folder, annotation_filename)
		with open(self.annotated_data_file, 'wb') as fd:
			pickle.dump([], fd)

		# Prepare the data stream
		self.datastream_people = self.datastream.get_people()

	def _sanity_check_timestamps(self):
		player_timestamps = self.bagfile_player.get_boundary_timestamps()
		data_timestamps = self.datastream.get_boundary_timestamps()

		if player_timestamps[0] > data_timestamps[0] or player_timestamps[1] < data_timestamps[1]:
			raise ValueError("Invalid values in the timestamps")

	def _prompt_annotations(self, segment_index, datastream, startstamp, endstamp):
		"""
		Prompts for annotations and saves them
		"""
		relevant_data = []
		for data in datastream:
			if data['timestamp'] >= startstamp:
				relevant_data.append(self.datastream.get_canonical_person(data['obj']))
				relevant_data[-1]['timestamp'] = data['timestamp']
				relevant_data[-1]['name'] = data['name']
			if data['timestamp'] > endstamp:
				break

		print "Found", len(relevant_data), "data objects"
		annotate_var = None # raw_input("Annotate? ")
		if not annotate_var or annotate_var == 'y':
			annotate_var = 'n'

		while annotate_var == 'n':
			self.bagfile_player.play_segment(segment_index)

			scene_object = "none" # raw_input("What object is this person with? ")
			try:
				int_input = raw_input("\033[92mRate this person's interruptibility: \033[0m")
				if int_input == 'x':
					sys.exit(0)
				interruptibility = int(int_input)
			except ValueError as e:
				print e
				continue

			annotate_var = raw_input("\033[91mDone Annotating? (x exits)\033[0m")
			if annotate_var == 'x':
				sys.exit(0)

		# Save the annotations
		for data in relevant_data:
			data['scene_object'] = scene_object
		annotated_block = {'data': relevant_data, 'value': interruptibility}

		with open(self.annotated_data_file, 'rb') as fd:
			saved_annotations = pickle.load(fd)

		saved_annotations.append(annotated_block)

		with open(self.annotated_data_file, 'wb') as fd:
			pickle.dump(saved_annotations, fd)
		print "Annotations have been saved"

	def annotate_data(self):
		"""
		This is the main method for this class
		"""
		self.marker_helper.run()
		people = self.datastream.get_people()

		# DEBUG
		counts = {}
		for person in self.datastream.get_people():
			num = len(self.datastream.get_datastream_for_person(person))
			if counts.get(num) is None:
				counts[num] = 0
			counts[num] += 1
		print counts
		if raw_input():
			sys.exit(0)
		# End DEBUG

		person_idx = 0
		while person_idx < len(people):
			person_name = people[person_idx]
			person_datastream = self.datastream.get_datastream_for_person(person_name)
			if len(person_datastream) < 5:
				person_idx += 1
				continue

			print "Presenting data about person:", person_name
			person_startstamp = person_datastream[0]['timestamp']
			person_endstamp = person_datastream[-1]['timestamp']
			start_bag_index = self.bagfile_player.get_index_for_timestamp(person_startstamp)
			end_bag_index = self.bagfile_player.get_index_for_timestamp(person_endstamp)

			print "Person is present for %.6f seconds" % (person_endstamp-person_startstamp).total_seconds()
			print "Start seconds:", person_startstamp.second, person_startstamp.microsecond, "\tEnd seconds:", person_endstamp.second, person_endstamp.microsecond

			continue_var = raw_input("\033[91mContinue? \033[0m")
			if continue_var == 'x':
				sys.exit(0)

			# Prepare the ROS player for playing the bags in this time segment
			self.marker_helper.set_person_datastream([
				{
					'timestamp': person['timestamp'],
					'obj': self.datastream.get_canonical_person(person['obj'])
				}
				for person in person_datastream
			])

			idx = start_bag_index
			ts = person_startstamp
			while idx <= end_bag_index and person_endstamp > ts:
				# Annotate the data
				self._prompt_annotations(idx, person_datastream, ts, ts+self.segment_duration_td)

				# Continue with the next segment if the person wants to
				idx += 1
				ts += self.segment_duration_td

			print "Done with", person_name
			self.marker_helper.set_person_datastream(None)
			repeat_var = raw_input("\033[91mNext person? (x exits) \033[0m")
			if repeat_var == 'x':
				sys.exit(0)
			if repeat_var == 'y' or not repeat_var:
				person_idx += 1


if __name__ == '__main__':
	if len(sys.argv) != 5:
		print "annotator.py <bagfiles> <bagfolder> <bagpickle> <sightpickle>"
		sys.exit(1)
	helper = AnnotationHelper(*sys.argv[1:])
	helper.annotate_data()
