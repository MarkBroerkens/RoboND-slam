[![Udacity - Robotics NanoDegree Program](https://s3-us-west-1.amazonaws.com/udacity-robotics/Extra+Images/RoboND_flag.png)](https://www.udacity.com/robotics) 
![ROS CI](https://github.com/MarkBroerkens/RoboND-slam/workflows/ROS%20CI/badge.svg)

# Robot Simulatnous Localization and Mapping (SLAM)
Simulation of 2-wheeled robot with *differential drive** that applies the FastSLAM algorithm RTABMAP for simultanous localization and mapping (SLAM).


You can use the keyboard to control the robot. See [Readme of the Teleoperation Package](https://github.com/MarkBroerkens/RoboND-slam/blob/main/teleop_twist_keyboard/README.md)


![Robot](https://github.com/MarkBroerkens/RoboND-slam/blob/main/my_robot/images/mybot.png)

![World](https://github.com/MarkBroerkens/RoboND-slam/blob/main/aws-robomaker-small-house-world/docs/images/gazebo_01.png)

### SLAM - How it works?
#### Terminology
* **Robot localization** determine where a mobile robot is located within its environment. (known map, unknown pose)
* **Mapping** modelling the environment. (known poses, unknown map)
* **SLAM** Simultanous Localization And Mapping (SLAM). Constructing or updating a map of an unknown environment while simultaneously keeping track of the robot's location within it. Inputs: Measurements, Controls, Outputs: Map, Trajestory.

SLAM allows a robot to navigate in an enviroment that it has never seen before.



#### SLAM Algorithms
1. **Forms**
 * **Online SLAM**: Robot estimates its current pose and the map using current measurements and controls.
    p(xt , m, ct | z1:t, u1:t) => estimate **current pose** and map (x: pose, m: map, z: measurement, u: control, c: correspondance)
 * **Full SLAM**: Robot estimates its entire trajectory and the map using all the measurements and controls.
    p(x1:t , m, c1:t | z1:t u1:t) => estimate **trajestory** and map 

2. **Nature**
 * **Continuous**: Robot continuously senses its pose and the location of the objects.
 * **Discrete**: Robot has to identify if a relation exists between any newly detected and previously detected objects.


##### Implementations
* Extended Kalman Filter SLAM (EKF)
* Sparse Extended Information Filter (SEIF)
* Extended Information Form (EIF)
* FastSLAM - e.g. real time appearance based mapping (RTABmap) -> used in this project
* GraphSLAM - e.g. gmapping



### Directory Structure
```
.
├── LICENSE
├── my_robot
│   ├── CMakeLists.txt
│   ├── config                         # config files for navigation
│   │   ├── base_local_planner_params.yaml
│   │   ├── costmap_common_params.yaml
│   │   ├── global_costmap_params.yaml
│   │   └── local_costmap_params.yaml
│   ├── images                         # images for documentation
│   │   └── mybot.png
│   ├── launch                         # launch files
│   │   ├── maping.launch              # launch mapping
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
├── README.md                          # this README.md file
├── teleop_twist_keyboard              # teleoperation with keyboard
│   ├── CHANGELOG.rst
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── README.md
│   └── teleop_twist_keyboard.py
└── aws-robomaker-small-house-world    # small house world
    ├── README.md
    └── ...

                                                                  

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
$ git clone https://github.com/MarkBroerkens/RoboND-slam
```


#### step 3 Install dependencies of packages in workspace
```sh
$ cd $HOME/catkin_ws
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
* The teleop_twist_keyboard code is copied from [https://github.com/ros-teleop/teleop_twist_keyboard](https://github.com/ros-teleop/teleop_twist_keyboard)
* Amazon AWS for its [Gazebo - Robomaker Small House World](https://github.com/aws-robotics/aws-robomaker-small-house-world)

# Further Reading
* [RTAB-Map Parameter Tutorial](http://wiki.ros.org/rtabmap_ros/Tutorials/Advanced%20Parameter%20Tuning)
* [List of RTAB-Map Parameters](https://github.com/introlab/rtabmap/blob/master/corelib/include/rtabmap/core/Parameters.h)
