Don't forget to build this package first

in the directory altair_ws:
source install/setup.bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select yantra_vision_deeplearning
source install/setup.bash
source /opt/ros/jazzy/setup.bash 

btw, due to its big size, the weights is not put in the repo but instead can be downloaded here
https://drive.google.com/drive/folders/1m8CeaoPj6KXPPewlVC-cSJIqhSTBaHyI?usp=sharing
place it on altair_ws/

note:
(x, y) = (0, 0) is the centre of image. +x to the right, and +y to the up.
z is the estimated distance to the ball, dependant on the cal_const, should be adjusted for the real camera.

Ensure your virtual environment is active and has the following dependencies:
- supervision
- numpy < 2.0 (Critical for cv_bridge compatibility)
- opencv-python
btw in my side the PYTHONPATH was adjusted in the .venv to allow ROS 2 to see the local supervision and rfdetr modules.

ROS2 interface:
Topic, Type, QoS, Description
1. Subscribed Topics
  - /camera/image/raw, sensor_msgs/Image, Sensor Data, Raw BGR input from camera
2. Published Topics
  - /vision/annotated_frame, sensor_msgs_/Image, Sensor Data, Debug stream with bounding boxes
  - /vision/ball_position, geometry_msgs/Point, Reliable, Ball screen coordinates and estimated distance

running:
terminal 1 (run the balldetect):
source install/setup.bash
source /opt/ros/jazzy/setup.bash
ros2 run yantra_vision_deeplearning balldetectrfdetr

terminal 2 (initialise camera):
source install/setup.bash
source /opt/ros/jazzy/setup.bash
ros2 run v4l2_camera v4l2_camera_node --ros-args -r image_raw:=/camera/image_raw

terminal 3 (rqt image_view):
source install/setup.bash
source /opt/ros/jazzy/setup.bash
ros2 run rqt_image_view rqt_image_view

terminal 4 (see in the topics):
source install/setup.bash
source /opt/ros/jazzy/setup.bash
ros2 topic echo /vision/ball_position
