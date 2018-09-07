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
            c = (0, 255, 0)

            cv2.rectangle(frame, (x1, y1), (x2, y2), c, 3)
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

            cv2.circle(frame, (nose_x, nose_y), 3, c, thickness=1)
            cv2.circle(frame, (left_eye_x, left_eye_y), 3, c, thickness=1)
            cv2.circle(frame, (right_eye_x, right_eye_y), 3, c, thickness=1)
            cv2.circle(frame, (left_mouth_x, left_mouth_y), 3, c, thickness=1)
            cv2.circle(frame, (right_mouth_x, right_mouth_y), 3, c, thickness=1)

        cv2.imshow('frame', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    capture.release()
    cv2.destroyAllWindows()

run()

