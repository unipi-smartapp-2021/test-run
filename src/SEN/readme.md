# Commands to let sensors run

**Make sure that your python packets installed match the one within the file requirements.txt**.

In order to run following commands, make sure that you executed this command before: `source ~/<catkin_root>/devel/setup.bash`. For example: `source ~/catkin_ws/devel/setup.bash`

By running the following commands, you should see the cone detection in action in real-time. Each one of those commands opens a window that shows the cone detection.

## Main commands
To run the stereo camera, type: `rosrun sensory rgb_camera.py`

To tun the lidar, type: `rosrun sensory lidar.py`

Only after executing both of the above commands, type: `rosrun sensory output_fusion.py`

## Other commands:

To subscribe to the output of the lidar, type: `rostopic echo /model/lidar/output`

If you want to change the model confidence of the model that recognizes the cones on the lidar, <br />
type: `rostopic pub /lidar/confidence std_msgs/Float32 <float_value>`.

To do the same, but for the camera, type: `rostopic pub /camera/confidence std_msgs/Float32 <float_value>`. <br />
**The <float_value> should be inside the range [0,1].** <br />
For instance, you can type: `rostopic pub /camera/confidence std_msgs/Float32 0.4` that sets the model confidence of the stereo camera to 0.4.
