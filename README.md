
## RUN MAIN TASK (task3)

```bash
roslaunch task3 rins_world.launch
```

```bash
roslaunch exercise3 amcl_simulation.launch
```

```bash
roslaunch turtlebot_rviz_launchers view_navigation.launch
```

```bash
roslaunch task3 start.launch
```

## RUN TASK 1

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

## RUN EXERCISE 6

IMPORTANT: IF rins_world DOESN'T START, CHECK THAT YOU'RE NOT EXPORTING 'ROS_IP' AND 'ROS_MASTER_URI'
IMPORTANT: You have to turn shadows off in gazebo.

```bash
roslaunch exercise6 rins_world.launch
```

```bash
roslaunch turtlebot_custom_navigation amcl_simulation.launch
```

```bash
roslaunch turtlebot_rviz_launchers view_navigation.launch
```

```bash
rosrun exercise6 detect_rings
```

## RUN EXERCISE 7

IMPORTANT: IF rins_world DOESN'T START, CHECK THAT YOU'RE NOT EXPORTING 'ROS_IP' AND 'ROS_MASTER_URI'
IMPORTANT: You have to tunr shadows off in gazebo.
IMPORTANT: PLACE on more light behind everything

```bash
roslaunch exercise7 rins_world.launch
```

```bash
roslaunch exercise3 amcl_simulation.launch
```

```bash
roslaunch turtlebot_rviz_launchers view_navigation.launch
```

```bash
rosrun exercise7 move_arm.py
```

```bash
rosrun exercise6 detect_rings
```

## Shorcuts, tips, tricks

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

```bash
ps aux | grep -ie ros | awk '{print $2}' | xargs kill -9
```