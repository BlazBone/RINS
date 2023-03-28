ne pozabit dodati source


# Shorcuts, tips, tricks

### Change border sensitivity of a robot
1. Open the file 
```bash
vim path_to_RINS/ROS_TURTLE/src/Turtlebot_packs_part1/turtlebot_navigation/param/costmap_common_params.yaml
```

2. Change value of `inflation_radius` (default is 0.5, 0.25 works best)
