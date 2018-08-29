# rail_face_detector
Face detection node using caffe and python

## Two Minute Intro

This detector uses [Caffe](http://caffe.berkeleyvision.org/) to perform face detection. It publishes faces found in images from a subscribed image topic. The face detector can be found here: https://github.com/kpzhang93/MTCNN_face_detection_alignment.

The message type coming back from the face detector is a Detections.msg which contains an array of [Face.msg](../rail_face_detection_msgs/msg/Face.msg) objects. Each Face.msg has:
```
int16 top_left_x        # X pixel value of top-left corner of bounding box
int16 top_left_y        # Y pixel value of top-left corner of bounding box
int16 bot_right_x       # X pixel value of bottom-right corner of bounding box
int16 bot_right_y       # Y pixel value of bottom-right corner of bounding box
int16 nose_x            # X coordinate of the nose
int16 nose_y            # Y coordinate of the nose
int16 left_eye_x        # X coordinate of the left eye
int16 left_eye_y        # Y coordinate of the left eye
int16 right_eye_x       # X coordinate of the right eye
int16 right_eye_y       # Y coordinate of the right eye
int16 left_mouth_x      # X coordinate of the left corner of the mouth
int16 left_mouth_y      # Y coordinate of the left corner of the mouth
int16 right_mouth_x     # X coordinate of the right corner of the mouth
int16 right_mouth_y     # Y coordinate of the right corner of the mouth
```

## Menu
 * [Installation](#installation)
 * [Testing your Installation](#testing-your-installation)
 * [ROS Nodes](#ros-nodes)
 * [Startup](#startup)

## Installation

1. Install Caffe and PyCaffe (following instructions at http://caffe.berkeleyvision.org/installation.html)
1. Add this package to your ROS workspace
1. Run `catkin_make` and enjoy the ability to use face detection!

If caffe is not in your PYTHONPATH, you will need to explicitly point the node to your `/caffe/python` directory. To do this:
1. Open `scripts/face_detector.py` in this package
1. Near the top (lines 6-10) set a variable named `caffe_root` to the absolute path of your `/caffe/python` directory. There are commented examples for Ubuntu and Mac already there.
1. Uncomment the `sys.path.append(caffe_root)` line (line 11)

The node should now run properly.

## Testing your Installation

- Set up an image topic that this node can subscribe to
- Launch the `face_detector_node` node with the debug flag and your chosen image topic name (for example, with a Kinect):
```
roslaunch rail_face_detector detector.launch image_sub_topic_name:=/kinect/qhd/image_color_rect debug:=true
```
You can also just test the network by running the `webcam_runner.py` script in the home directory of this package. This uses OpenCV to pull images from your webcam, highlight faces and keypoints, and show the images in real time.

## ROS Nodes

### face_detector_node

Wrapper for object detection through ROS services.  Relevant services and parameters are as follows:

* **Topics**
  * `detector_node/faces` ([rail_face_detector/faces](../rail_face_detection_msgs/msg/Detections.msg))
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; Topic with face detections performed in the background by running on images as they come in the subscriber.
  * `detector_node/debug/face_images` (sensor_msgs/Image)
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; Topic with face detections visualized on incoming images as they come in from the subscriber. Only published if debug=true.
* **Parameters**
  * `image_sub_topic_name` (`string`, default: "/kinect/qhd/image_color_rect")
 &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; Image topic name to use for detections.
  * `debug` (`bool`, default: false)
 &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; Enable or disable debug mode, which publishes incoming images with bounding boxes over faces
  * `use_gpu` (`bool`, default: true)
 &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; Enable or disable gpu mode. Enabled by default. Significantly speeds up detection.
   * `use_compressed_image` (`bool`, default: false)
 &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; Change to compressed image stream or not. Simply appends a "/compressed" to the image topic name. This lightens the load your local network if the images are being transmitted to the detector.

## Startup

Simply run the launch file to bring up all of the package's functionality (default: use Scene Queries only):
```
roslaunch rail_face_detector detector.launch
```
