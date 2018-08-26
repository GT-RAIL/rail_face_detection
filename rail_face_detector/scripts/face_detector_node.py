#!/usr/bin/env python
# This file is responsible for bridging ROS to the FaceDetector class (built with Caffe)

from __future__ import division

import sys

import cv2
import rospy
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CompressedImage
from rail_face_detection_msgs.msg import Face, Detections

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
        self.debug = rospy.get_param('~debug', default=False)
        self.image_sub_topic_name = rospy.get_param('~image_sub_topic_name', default='/kinect/qhd/image_color_rect')
        use_gpu = rospy.get_param('~use_gpu', default=True)
        self.use_compressed_image = rospy.get_param('~use_compressed_image', default=False)
        self.face_detector = face_detector.FaceDetector(use_gpu)

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

    def _convert_msg_to_image(self, image_msg):
        """
        Convert an incoming image message (compressed or otherwise) into a cv2
        image
        """
        if not self.use_compressed_image:
            try:
                image_cv = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
            except CvBridgeError as e:
                print e
                return None
        else:
            image_np = np.fromstring(image_msg.data, np.uint8)
            image_cv = cv2.imdecode(image_np, cv2.CV_LOAD_IMAGE_COLOR)

        return image_cv

    def _parse_image(self, image_msg):
        """
        Take in an image and draw a bounding box within it
        :param image_msg: Image data
        :return: None
        """

        header = image_msg.header
        image_cv = self._convert_msg_to_image(image_msg)
        if image_cv is None:
            return
        self.faces, self.keypoint_arrays = self.face_detector.find_faces(image_cv)
        if self.debug:
            c = (0, 255, 0)
            for face, points in zip(self.faces, self.keypoint_arrays):
                x1 = int(face[0])
                y1 = int(face[1])
                x2 = int(face[2])
                y2 = int(face[3])
                nose_x = int(points[2])
                nose_y = int(points[7])
                left_eye_x = int(points[1])
                left_eye_y = int(points[6])
                right_eye_x = int(points[0])
                right_eye_y = int(points[5])
                left_mouth_x = int(points[4])
                left_mouth_y = int(points[9])
                right_mouth_x = int(points[3])
                right_mouth_y = int(points[8])
                image_cv = self._draw_bb(image_cv, {'x': x1,
                                                    'y': y1,
                                                    'w': x2-x1,
                                                    'h': y2-y1}, c)
                cv2.circle(image_cv, (nose_x, nose_y), 3, c, thickness=1)
                cv2.circle(image_cv, (left_eye_x, left_eye_y), 3, c, thickness=1)
                cv2.circle(image_cv, (right_eye_x, right_eye_y), 3, c, thickness=1)
                cv2.circle(image_cv, (left_mouth_x, left_mouth_y), 3, c, thickness=1)
                cv2.circle(image_cv, (right_mouth_x, right_mouth_y), 3, c, thickness=1)
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
            msg.nose_x = int(points[2])
            msg.nose_y = int(points[7])
            msg.left_eye_x = int(points[1])
            msg.left_eye_y = int(points[6])
            msg.right_eye_x = int(points[0])
            msg.right_eye_y = int(points[5])
            msg.left_mouth_x = int(points[4])
            msg.left_mouth_y = int(points[9])
            msg.right_mouth_x = int(points[3])
            msg.right_mouth_y = int(points[8])
            face_arr.faces.append(msg)

        self.face_pub.publish(face_arr)

    def run(self,
            pub_image_topic='~debug/face_image',
            pub_face_topic='~faces'):
        if not self.use_compressed_image:
            rospy.Subscriber(self.image_sub_topic_name, Image, self._parse_image) # subscribe to sub_image_topic and callback parse
        else:
            rospy.Subscriber(self.image_sub_topic_name+'/compressed', CompressedImage, self._parse_image)
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
