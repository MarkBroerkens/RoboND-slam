[![Udacity - Robotics NanoDegree Program](https://s3-us-west-1.amazonaws.com/udacity-robotics/Extra+Images/RoboND_flag.png)](https://www.udacity.com/robotics) 
![ROS CI](https://github.com/MarkBroerkens/RoboND-slam/workflows/ROS%20CI/badge.svg)

# Robot Simulatnous Localization and Mapping (SLAM)
Simulation of 2-wheeled robot with *differential drive** that localizes itself using Adaptive Montecarlo Localization (AMCL).


You have two options to control the robot:
* If the focus is on the terminal window you can use the keyboard to control the robot. See [Readme of the Teleoperation Package](https://github.com/MarkBroerkens/RoboND-localization/blob/main/teleop_twist_keyboard/README.md)
* Alternatively you can set a 2D navigation target in Rviz and robot calculates the path to the target autonomously.

AMCL provides many parameters which allow for optimizing the algorithm for the given scenario. Please see [amcl.launch](https://github.com/MarkBroerkens/RoboND-localization/blob/main/my_robot/launch/amcl.launch) for details.

![Robot](https://github.com/MarkBroerkens/RoboND-localization/blob/main/my_robot/images/mybot.png)

![Initial localization shown in Rviz](https://github.com/MarkBroerkens/RoboND-localization/blob/main/my_robot/images/localization_rviz2.png)

![Go to target navigation shown in Rviz](https://github.com/MarkBroerkens/RoboND-localization/blob/main/my_robot/images/localization_rviz_animation.gif)


### SLAM - How it works?
#### Terminology
* **Robot localization** determine where a mobile robot is located within its environment. (known map, unknown position)
* **Mapping** modelling the environment. (known position, unknown map)
* **SLAM** Simultanous Localization And Mapping (SLAM). Constructing or updating a map of an unknown environment while simultaneously keeping track of the robot's location within it. 

SLAM allows a robot to navigate in an enviroment that it has never seen before



#### SLAM Algorithms
* Extended Kalman Filter SLAM (EKF)
* Sparse Extended Information Filter (SEIF)
* Extended Information Form (EIF)
* FastSLAM - e.g. real time appearance based mapping (RTABmap)
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
│   │   ├── maping.launch    
│   │   ├── teleop.launch     
│   │   ├── robot_description.launch
│   │   └── world.launch
│   ├── maps                           # map, generated from myworld.world
│   │   ├── myworld.pgm                
│   │   └── myworld.yaml
│   ├── meshes                         
│   │   └── hokuyo.dae                 # mesh of lidar sensor
│   ├── package.xml                    # package info
│   ├── rviz                           # rviz configuration
│   │   └── amcl.rviz
│   ├── urdf                           # robot description files
│   │   ├── my_robot.gazebo
│   │   └── my_robot.xacro
│   └── world                          # world folder for world files
│       └── myworld.world
└── README.md                          # this README.md file
└── teleop_twist_keyboard              # teleoperation with keyboard
    ├── CHANGELOG.rst
    ├── CMakeLists.txt
    ├── package.xml
    ├── README.md
    └── teleop_twist_keyboard.py

                                                                  

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
$ roslaunch my_robot world.launch

```
This will open Rviz and Gazebo. 

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
