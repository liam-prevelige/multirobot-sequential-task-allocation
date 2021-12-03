# Auction-based task allocation

ROS package for robots to coordinate to cover specific locations in a common reference frame. Assumes that only one robot is the auctioneer and that robots bid on a single location independently. The auctioneer then assigns a task to robots following the sequential auction mechanism, until each task is allocated to a robot. The test is done in an empty world with 3 robots and the locations are randomly generated over a space of 10m x 10m

## Requirements
- ROS -- tested on Melodic, but other versions may work.
- catkin -- used for building the application.

## Build
Once cloned in a ROS workspace, e.g., `ros_workspace/src/auction_tasks`, run the following commands to build it:
```bash
	cd ros_workspace
	catkin_make
```
	
## Run
First, source the files. 
```bash
source ros_workspace/devel/setup.sh
```
To view the auction-based task allocation with 3 turtlebots in Gazebo, run the corresponding launch file with roslaunch.
```bash
    roslaunch tf_coordination gazebo_tf_coordination.launch
```

## Attribution
Authored by Liam Prevelige, with modifications from Professor Alberto Quattrini Li's simple_motion ROS node: https://github.com/quattrinili/simple_motion.git
