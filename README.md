# Self-Driving Robot in Gazebo

## Instructions

Install OpenCV library for C++
Install OpenCV for Python 3 through the console
Create a Catkin Workspace and put the src folder inside

Put the contents of the world/models folder inside ".gazebo/models"
Run catkin_make

After sourcing the bash file,
Run the following commands in order:

```sh
roslaunch world course.launch
rosrun autonomous-robot autonomous-robot-node
rosrun autonomous-robot sign_publisher.py
```





