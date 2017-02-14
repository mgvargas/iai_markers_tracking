# iai_markers_tracking

## Installation 
Using ```catkin_tools``` and ```wstool``` in a new workspace for ```ROS Kinetic``` installed on ```Ubuntu 16.04```:

```
source /opt/ros/kinetic/setup.bash         # start using ROS Kinetic
mkdir -p ~/obj_grasp_ws/src                # create directory for workspace
cd ~/obj_grasp_ws/src                      # go to workspace directory
catkin_init_workspace                      # init workspace
wstool init                                # init rosinstall
wstool merge https://raw.githubusercontent.com/mgvargas/iai_markers_tracking/master/rosinstall/catkin.rosinstall
                                           # update rosinstall file
wstool update                              # pull source repositories
rosdep install --ignore-src --from-paths . # install dependencies available through apt
cd ..                                      # go to workspace directory
catkin_make                                # build packages
source ~/obj_grasp_ws/devel/setup.bash     # source new overlay
```

Install ```apscheduler``` for Python:
```
sudo apt install python-pip					# Install pip
pip install apscheduler							# Install apscheduler
```
Install ```ArUco ROS``` and ```video_stream_opencv``` (if required):
```
	https://github.com/pal-robotics/aruco_ros				# Marker detection
	https://github.com/ros-drivers/video_stream_opencv # Video streaming from webcam
```

# Running the code
Run ```ArUco ROS``` and ```video_stream_opencv```:
```
   roslaunch aruco_ros double.launch
	roslaunch video_stream_opencv camera.launch  # If no perceptionsystem is running
```
 just rename the ```camera_info``` and ```image``` topics to the ones provided by the camera. To visualize the result of the marker detection, run
```
 	rosrun image_view image_view image:=/aruco_simple/result
```
Run ```db_reader.py``` from the ```iai_markers_tracking``` package
```
	roslaunch iai_markers_tracking db_reader.py
```
Run RVIZ and add a MarkerArray and Axes, change the reference frame to the camera reference frame.
Place the objects with markers in front of the camera.

