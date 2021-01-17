[![Udacity - Robotics NanoDegree Program](https://s3-us-west-1.amazonaws.com/udacity-robotics/Extra+Images/RoboND_flag.png)](https://www.udacity.com/robotics) 
![ROS CI](https://github.com/MarkBroerkens/RoboND-localization/workflows/ROS%20CI/badge.svg)

# Robot Localization
Simulation of 4-wheeled robot with **skid steer drive** that localizes itself using Adaptive Montecarlo Localization (AMCL).

![Skid Steer Robot](https://github.com/MarkBroerkens/RoboND-localization/blob/main/my_robot/images/mybot.png)


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
│   │   ├── amcl.launch         
│   │   ├── main.launch                # main launch files to start all other launch files needed in this project
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
$ git clone https://github.com/MarkBroerkens/RoboND-localization skid-steer-robot
```


### step 3 Install teleoperation node
```sh
$ cd $HOME/catkin_ws
git clone https://github.com/ros-teleop/teleop_twist_keyboard
```


#### Step 4 Compile the code
```sh
$ cd $HOME/catkin_ws
$ catkin_make
$ source devel/setup.bash
```


#### Step 5 Run the Simulation 
##### in terminal 1:

```sh
$ source $HOME/catkin_ws/devel/setup.bash
$ roslaunch my_robot main.launch

```


##### in Rviz:

define a 2D navigation target.


# License
MIT license
