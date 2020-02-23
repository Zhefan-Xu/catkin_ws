# catkin_ws
Add sjtu_drone, nbv_3d, move_quadrotor to your catkin workspace.

## Compile
```
cd ~/your_catkin_ws
catkin_make
```

## Execute
launch enviroment
```
roslaunch sjtu_drone tunnel_drone.launch
```
Take off drone by pressing `z`

Call NBV_3d Planner
```
rosrun nbv_3d planning
```

## Stop
`ctrl c' to stop NBV planner when it is finished.
