# Collaborative Legged Loco-Manipulation over Discrete Terrain

This repository contains the code for the paper "[Safety-critical Motion Planning for Collaborative Legged Loco-Manipulation over Discrete Terrain](https://arxiv.org/abs/2410.11023)." This work addresses safe motion planning for collaborative manipulation of an unknown payload on discrete terrain while avoiding obstacles. Our approach uses two sets of MPCs as motion planners: a global MPC generates a safe trajectory for the team with obstacle avoidance, while decentralized MPCs for each robot ensure safe footholds on discrete terrain as they follow the global trajectory. 

## Video Demonstration
[![Video Title](https://img.youtube.com/vi/MuJY9rYxTO4/0.jpg)](https://www.youtube.com/watch?v=MuJY9rYxTO4)

# Installation
## Dependencies

Follow the dependencies, check this [repository](https://github.com/DRCL-USC/Quadruped_Wrapper/tree/master).

## Build the library

Create a new catkin workspace:

```
# Create the directories
# Do not forget to change <...> parts
mkdir -p <directory_to_ws>/<catkin_ws_name>/src
cd <directory_to_ws>/<catkin_ws_name>/

# Initialize the catkin workspace
catkin init
catkin config --extend /opt/ros/noetic
catkin config -DCMAKE_BUILD_TYPE=RelWithDebInfo
```
Clone the code:

```
# Navigate to the directory of src
cd <directory_to_ws>/<catkin_ws_name>/src
git clone --depth 1 --recurse-submodules 
```
Build package for the planner as well as legged controllers:
```
cd ..
catkin build multi_legged_controllers planner_ros
```

# Usage
To facilitate running the motion planner stack and the decentralized loco-manipulation controller stack, we provide a Tmux script to manage the entire stack efficiently. To use this script, first install Tmux and Tmuxp:

```
sudo apt-get install tmux tmuxp
```

Next, navigate to the scripts directory and load the Tmux session:

```
cd src/multi_legged_control/scripts
tmuxp load simulation.yaml
```

The Tmux session includes a window for the motion planner and two additional windows for decentralized loco-manipulation control for each robot. Detailed information about the loco-manipulation stack can be found [here](https://github.com/DRCL-USC/Loco_manipulation_control).

The planner stack starts with this launch file, which loads all the core nodes:

```
roslaunch ocs2_object_manipulation_ros manipulation_stack.launch gui:=false rviz:=true multiplot:=false record_data:=false
```

This launch file contains several arguments. Setting `gui` to true enables Gazebo simulations. `rviz` enables RViz data visualization. `multiplot` plots the planner variables online, and `record_data` stores rostopic data related to the planner and decentralized controller into a ROS bag file.

Once the stack is running, a new terminal will pop up, allowing you to enter the target position and orientation for the manipulated object. After specifying the target, the manipulation process will begin.

# Citation 
```
@misc{Sombolestan2024Safety-criticalTerrain,
    title = {{Safety-critical Motion Planning for Collaborative Legged Loco-Manipulation over Discrete Terrain}},
    year = {2024},
    author = {Sombolestan, Mohsen and Nguyen, Quan},
    month = {10},
    url = {https://arxiv.org/abs/2410.11023v1},
    isbn = {2410.11023v1},
    arxivId = {2410.11023}
}
```
