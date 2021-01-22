Instructions

Install OpenCV library for C++
    Download the Source Code from the official state_publisher
    Unzip the contents
    Create a "build" folder within
    cd build
    cd .. make
Install OpenCV for Python 3 through the console


1 Create a Catkin Workspace and put the src folder inside
Run catkin_make








After sourcing the bash file,
Run the following commands in order:


roslaunch mycustomworld course.launch
rosrun image_processor image_processor_node
rosrun image_processor state_publisher.py


