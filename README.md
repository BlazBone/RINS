# POZOR

POTREBNO JE DA SI NADOMETSTITA TO ZA ZVOK

```bash
sudo apt install ros-noetic-sound-play
```

### RUN TASK 1

```bash
roslaunch exercise3 rins_world.launch
```

```bash
roslaunch exercise3 amcl_simulation.launch
```

```bash
roslaunch turtlebot_rviz_launchers view_navigation.launch
```

```bash
rosrun sound_play soundplay_node.py
```

```bash
rosrun task1 task1
```

or (to create images)

```bash
(rm run_info/* && rosrun task1 task1) || rosrun task1 task1
```

### RUN TASK 1 ON REAL ROBOT

#### SSH to laptop which is connected to robot

connect to Turtlenet wifi

add this to .bashrc

```bash
export ROS_IP={YOUR_IP}
export ROS_MASTER_URI=http://kili.local:11311/
```

```bash
ssh team_ai_avengers@kili.local
```

```
password: 068bd410
```

#### Commands running on laptop connected to robot

```bash
export TURTLEBOT_MAP_FILE=/home/team_ai_avengers/RW_map.yaml
```

```bash
roslaunch *something* minimal.launch
```

```bash
roslaunch *something* amcl_demo.launch
```

#### Commands running localy on workstation

```bash
roslaunch turtlebot_rviz_launchers view_navigation.launch
```

```bash
rosrun sound_play soundplay_node.py
```

```bash
rosrun task1 task1
```

# Shorcuts, tips, tricks

### Change border sensitivity of a robot

1. Open the file

```bash
vim path_to_RINS/ROS_TURTLE/src/Turtlebot_packs_part1/turtlebot_navigation/param/costmap_common_params.yaml
```

2. Change value of `inflation_radius` (default is 0.5, 0.25 works best)

```yaml
inflation_radius: 0.25 # max. distance from an obstacle at which costs are incurred for planning paths.
cost_scaling_factor: 20.0 # exponential rate at which the obstacle cost drops off (default: 10)
```

### Check if ros packages are in path variable

```bash
echo $ROS_PACKAGE_PATH
```

### Source your packages

Add this to your .bashrc or .zshrc file, be sure to change the endings acording to your shell.

```bash
source /opt/ros/noetic/setup.zsh
source /home/bone/RINS/ROS/devel/setup.zsh
source /home/bone/RINS/ROS_TURTLE/devel/setup.zsh --extend
```
