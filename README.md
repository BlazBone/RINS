ne pozabit dodati source

# TASK 1

```bash
roslaunch exercise3 rins_world.launch
```

```bash
roslaunch exercise3 amcl_simulation.launch
```

```bash
roslaunch turtlebot_rviz_launchers view_navigation.launch

# Shorcuts, tips, tricks

### Change border sensitivity of a robot
1. Open the file 
```bash
vim path_to_RINS/ROS_TURTLE/src/Turtlebot_packs_part1/turtlebot_navigation/param/costmap_common_params.yaml
```

2. Change value of `inflation_radius` (default is 0.5, 0.25 works best)
```yaml
inflation_radius:     0.25  # max. distance from an obstacle at which costs are incurred for planning paths.
```
