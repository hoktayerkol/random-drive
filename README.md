# random-drive ROS 2

Random_drive is a ROS2 node that subscribes to laser scans and publishes velocity commands for random driving and avoiding obstacles.

It has been tested with ROS2 Humble on Ubuntu 22.04.

It can be used with any mobile robot which has a laser scanner by remapping the topics (see the launch file).

For a quick test, after building the package, run Turtlebot3 simulation and random_drive node using the commands given below:

- On the first terminal:
```
$ export TURTLEBOT3_MODEL=waffle
$ ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```
- On the second terminal:
```
$ ros2 launch random_drive random_drive_launch.py
```
The robot will start to drive randomly by avoiding obstacles.
