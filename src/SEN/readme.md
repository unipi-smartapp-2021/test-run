# Commands to let our sensors run

In order to run following commands make sure that you executed this command before: source ~/catkin_ws/devel/setup.bash

By running the following commands, you should see the cone detection in action in real-time. Each one of those commands opens a window that shows the cone detection.

To run the stereo camera, type: `rosrun smartapp rgb_camera.py`

To tun the lidar, type: `rosrun smartapp lidar.py`

To subscribe to the output of the lidar, type: `rostopic echo /model/lidar/output`

Other commands:

If you want to change the model confidence of the model that recognizes the cones on the lidar, type: `rostopic pub /lidar/confidence std_msgs/Float32 <float_value>`.

To do the same, but for the camera, type: `rostopic pub /camera/confidence std_msgs/Float32 <float_value>`. The <float_value> should be inside the range [0,1].
For instance, you can type: `rostopic pub /camera/confidence std_msgs/Float32 0.4` that sets the model confidence of the stereo camera to 0.4.
