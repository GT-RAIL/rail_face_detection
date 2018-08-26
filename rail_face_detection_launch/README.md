# RAIL Face Detection Launch

Meta launch package for different face detection solutions.

## Willow Garage face detection

Prerequisites:
- wg-perception's [people library](https://github.com/wg-perception/people): `sudo apt install ros-indigo-people`

Running:
- `roslaunch rail_face_detection_launch face_detector.launch wg_detector:=true`
- Will bring up a face_detector instance preconfigured and tuned for the Fetch's RGBD camera setup
- Additional documentation of library: http://wiki.ros.org/face_detector
