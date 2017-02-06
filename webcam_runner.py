import cv2

from scripts import face_detector


def run():
	capture = cv2.VideoCapture(0)
	finder = face_detector.FaceDetector()
	while True:
		ret, frame = capture.read()

		input_image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
		# find face bounding boxes and keypoints (0-right eye, 1-left eye, 2-nose, 3-right mouth, 4-left mouth
		faces, keypoints = finder.find_faces(input_image)

		for (face, points) in zip(faces, keypoints):
			x1 = int(face[0])
			y1 = int(face[1])
			x2 = int(face[2])
			y2 = int(face[3])
			nose = (int(points[2]), int(points[7]))
			x_percent = (nose[0] - x1) / float(x2 - x1)
			# y_percent = (nose[1] - y1) / float(y2 - y1)
			if x_percent < 0.35 or x_percent > 0.65:
				c = (0, 0, 0)
			else:
				c = (0, 255, 0)

			cv2.rectangle(frame, (x1, y1), (x2, y2), c, 3)

		cv2.imshow('frame', frame)

		if cv2.waitKey(1) & 0xFF == ord('q'):
			break
	capture.release()
	cv2.destroyAllWindows()

run()

