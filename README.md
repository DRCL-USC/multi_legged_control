# Collaborative Legged Loco-Manipulation over Discrete Terrain

This repository hosts the code for the paper "[Safety-critical Motion Planning for Collaborative Legged Loco-Manipulation over Discrete Terrain](https://arxiv.org/abs/2410.11023)." The research focuses on safe motion planning for collaborative manipulation of an unknown payload on discrete terrain while avoiding obstacles. Our method employs two sets of MPCs as motion planners: a global MPC that generates a safe trajectory for the team with obstacle avoidance, and decentralized MPCs for each robot that ensure safe footholds on discrete terrain as they follow the global trajectory.

## Video Demonstration
[![Video Title](https://img.youtube.com/vi/MuJY9rYxTO4/0.jpg)](https://www.youtube.com/watch?v=MuJY9rYxTO4)

# Installation
## Dependencies

To check the dependencies, visit this [repository](https://github.com/DRCL-USC/Quadruped_Wrapper/tree/master).

## Build the Library

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

Clone the repository:

```
# Navigate to the src directory
cd <directory_to_ws>/<catkin_ws_name>/src
git clone --depth 1 --recurse-submodules https://github.com/DRCL-USC/multi_legged_control.git
```

Build the packages for the planner and legged controllers:

```
cd ..
catkin build multi_legged_controllers planner_ros
```

# Usage
To efficiently run the motion planner stack and the decentralized loco-manipulation controller stack, we provide a Tmux script. First, install Tmux and Tmuxp:

```
sudo apt-get install tmux tmuxp
```

Then, navigate to the scripts directory and load the Tmux session:

```
cd src/multi_legged_control/scripts
tmuxp load simulation.yaml
```

If you prefer not to use Tmux, you can run each shell command from the `simulation.yaml` script in a separate terminal.

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
