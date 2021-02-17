Instructions

Install OpenCV library for C++

Install OpenCV for Python 3 through the console


Create a Catkin Workspace and put the src folder inside

Put the contents of the mycustomworld/models folder inside ".gazebo/models"
Run catkin_make


After sourcing the bash file,
Run the following commands in order:


roslaunch mycustomworld course.launch
rosrun image_processor image_processor_node
rosrun image_processor state_publisher.py


