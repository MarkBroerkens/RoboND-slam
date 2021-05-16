[![Udacity - Robotics NanoDegree Program](https://s3-us-west-1.amazonaws.com/udacity-robotics/Extra+Images/RoboND_flag.png)](https://www.udacity.com/robotics) 
![ROS CI](https://github.com/MarkBroerkens/RoboND-slam/workflows/ROS%20CI/badge.svg)

# Robot Simulatnous Localization and Mapping (SLAM)
Simulation of 2-wheeled robot with *differential drive** that applies the GraphSLAM algorithm RTABMAP for simultanous localization and mapping (SLAM). 
SLAM enables constructing or updating a map of an unknown environment while simultaneously keeping track of the robot's location within it. Inputs: Measurements, Controls, Outputs: Map, Trajestory.

You can use the keyboard to control the robot. See [Readme of the Teleoperation Package](https://github.com/MarkBroerkens/RoboND-slam/blob/main/teleop_twist_keyboard/README.md).

While navigating through the example house, the rtabmap algorithm feeds data into a database.
Please find an [example database in google drive](https://drive.google.com/file/d/13yFbeKmmZcJY8Eg5G_mXbVAK3gq-PYAX/view?usp=sharing)


# Impressions
## The robot
![Robot](https://github.com/MarkBroerkens/RoboND-slam/blob/main/my_robot/images/mybot.png)

## The environment
![World](https://github.com/MarkBroerkens/RoboND-slam/RoboND-slam/blob/main/my_robot/images/gazebo_01.png)

## 3D Occupancy Grid Map
![3D Occupancy Grid](https://github.com/MarkBroerkens/RoboND-slam/blob/main/my_robot/images/OccupancyGrid.png)

## RTABMap Database Viewer
![RTABMap Database Viewer](https://github.com/MarkBroerkens/RoboND-slam/blob/main/my_robot/images/rtabmap-databaseviewer.png)


### Directory Structure
```
.
├── LICENSE
├── my_robot
│   ├── CMakeLists.txt
│   ├── images                         # images for documentation
│   │   └── mybot.png
│   ├── launch                         # launch files
│   │   ├── maping.launch              # launch mapping
│   │   ├── localization.launch        # launch localization
│   │   ├── teleop.launch              # launch teleop
│   │   ├── robot_description.launch   # launch robot
│   │   └── world.launch               # launch world
│   ├── meshes                         
│   │   └── hokuyo.dae                 # mesh of lidar sensor
│   ├── package.xml                    # package info
│   ├── rviz                           # rviz configuration
│   │   └── default.rviz
│   ├── urdf                           # robot description files
│   │   ├── my_robot.gazebo
│   │   └── my_robot.xacro
└── README.md                          # this README.md file

```


### Steps to launch the simulation

#### Step 1 Update and upgrade the Workspace image
```sh
$ sudo apt-get update
$ sudo apt-get upgrade -y
```

#### Step 2 Create the catkin workspace
```sh
$ mkdir -p $HOME/catkin_ws/src
$ cd $HOME/catkin_ws/src
$ catkin_init_workspace
```


#### step 3 Install dependencies of packages in workspace
```sh
$ cd $HOME/catkin_ws
$ sudo apt-get install python-rosinstall
$ rosinstall . https://raw.githubusercontent.com/MarkBroerkens/RoboND-slam/main/slam.rosinstall
$ rosdep install --from-paths src --ignore-src -r -y
```


#### Step 4 Compile the code
```sh
$ cd $HOME/catkin_ws
$ catkin_make
$ source devel/setup.bash
```


#### Step 5 Run the Simulation 
##### in Terminal 1
```sh
$ source $HOME/catkin_ws/devel/setup.bash
$ roslaunch my_robot world.launch gazebo_gui:=true

```
This will open Rviz and Gazebo. Omit the "gazebo_gui:=true" if you do not need the gazebo gui.

##### in Terminal 2
```sh
$ source $HOME/catkin_ws/devel/setup.bash
$ roslaunch my_robot teleop.launch

```
This will run the teleoperation mode.

##### in Terminal 3

```sh
$ source $HOME/catkin_ws/devel/setup.bash
$ roslaunch my_robot maping.launch
```
This will run the RTAB mapping.




# License
MIT license

# Thanks to
* ros teleop for [https://github.com/ros-teleop/teleop_twist_keyboard](https://github.com/ros-teleop/teleop_twist_keyboard)
* Amazon AWS for its [Gazebo - Robomaker Small House World](https://github.com/aws-robotics/aws-robomaker-small-house-world)

# Further Reading
* [RTAB-Map Parameter Tutorial](http://wiki.ros.org/rtabmap_ros/Tutorials/Advanced%20Parameter%20Tuning)
* [List of RTAB-Map Parameters](https://github.com/introlab/rtabmap/blob/master/corelib/include/rtabmap/core/Parameters.h)
