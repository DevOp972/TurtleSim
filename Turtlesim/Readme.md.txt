turtlesim_cleaner is the node created for controlling the turtle
The directory is in
catkin_ws/src/

If node is not recognised by rosrun command, use:
source ~/catkin_ws/devel/setup.bash

There are 2 launch files in turtlesim_cleaner
1)roslaunch ~/catkin_ws/src/turtlesim_cleaner/launch/part2.launch
This moves the turtle from source to destination following the path generated using rrt*-connect.The initial point should be changed in 2nd line of part.launch file
2)roslaunch ~/catkin_ws/src/turtlesim_cleaner/launch/part2.launch
Moves the turtle along the path and also spawns a turtle

