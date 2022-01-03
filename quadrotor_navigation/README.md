# quadrotor_navigation
Contains all files needed to implement navigation on quadrotor. Depends on hector_quadrotor_sim.

## Getting Started
### One Quadrotor (gmapping)
```
roslaunch hector_quadrotor_gazebo camera_launch.launch
roslaunch quadrotor_navigation drone_gmap.launch
rostopic pub /takeoff std_msgs/Empty "{}"
roslaunch multi_jackal_control jackal_teleop.launch
```
### Two Jackals and One Quadrotor
```
roslaunch multi_jackal_tutorials jackal_drone.launch gui:=true
rostopic pub /drone/takeoff std_msgs/Empty "{}"
```

## Author
Rachel Zheng - rz246
