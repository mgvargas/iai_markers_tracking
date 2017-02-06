# rviz_markers

## Installation 
Using ```catkin_tools``` and ```wstool``` in a new workspace for ```ROS Kinetic``` installed on ```Ubuntu 16.04```:

```
source /opt/ros/kinetic/setup.bash         # start using ROS Kinetic
mkdir -p ~/obj_grasp_ws/src                # create directory for workspace
cd ~/obj_grasp_ws/src                      # go to workspace directory
catkin_init_workspace                      # init workspace
wstool init                                # init rosinstall
wstool merge https://raw.githubusercontent.com/mgvargas/rviz_markers/master/rosinstall/catkin.rosinstall
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

