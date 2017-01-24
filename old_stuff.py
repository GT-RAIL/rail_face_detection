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
