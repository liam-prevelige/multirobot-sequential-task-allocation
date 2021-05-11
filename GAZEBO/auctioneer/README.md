# tf_coordination ROS package

This ROS package contains a ROS node that allows for a multi-robot system to move with a flocking behavior.

## Requirements
- ROS -- tested on Melodic, but other versions may work.
- catkin -- used for building the application. 
- (optional) stage_ros -- used for running 10 robots in Stage simulator

## Build
Once cloned in a ROS workspace, e.g., `ros_workspace/src/`, run the following commands to build it:
```bash
	cd ros_workspace
	catkin_make
```
	
## Run
First, source the tf_coordination file. 
```bash
source ros_workspace/devel/setup.sh
```
To view the tf_coordination behavior of 3 robots in Gazebo, run the corresponding launch file with roslaunch.
```bash
    roslaunch tf_coordination gazebo_tf_coordination.launch
```
Or, to view the tf_coordination behavior of 10 robots in Stage, run the corresponding launch file with roslaunch:
```bash
    roslaunch tf_coordination stage_tf_coordination.launch
```

## Attribution
Authored by Liam Prevelige, modified from Professor Alberto Quattrini Li's simple_motion ROS node: https://github.com/quattrinili/simple_motion.git
