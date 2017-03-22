Prerequisites
-------------------

If you haven't done already:

Install wstool

```
sudo apt-get install python-wstool
```

Create catkin workspace directory (skip this if you already have one)
```
mkdir -p ~/catkin_ws/src
```

Download RethinkRobotics github repos (this assumes that your catkin workspace folder is ~/catkin_ws).
```
cd ~/catkin_ws/src
wstool init .
wstool merge https://raw.githubusercontent.com/RethinkRobotics/baxter/master/baxter_sdk.rosinstall
wstool update
```

To make and install
-------------------
Download the repository:
```
cd ~/catkin_ws/src
wstool merge https://raw.githubusercontent.com/cristipiticul/baxter_h2r_packages/indigo-devel/h2r.rosinstall
wstool update
```

-Build it
```
cd .. 
source /opt/ros/indigo/setup.bash
rosdep update
rosdep install --from-paths . --ignore-src --rosdistro indigo -y
catkin_make
```


Components available
-----------------------

baxter_kinect_calibration
Calibration of Kinect relative to robot pose

baxter_grasps_server
A ROS service that responds to requests of object names with Grasp messages (moveit_msgs/Grasp) loaded from yaml files. It also contains scripts for annotating grasps on objects

baxter_pick_and_place
Scripts and launch files that demonstrate picking and placing using Baxter and MoveIt!

baxter_props
Commands baxter to give a high five, a fist bump or a hug

baxter_screen_writer
Write messages to baxter's screen using either a library or a listener node

baxter_urdf_builder
Merge your robot's specific URDF file into a file URDF
