
package1 and package2 from /src as per instruction

Initiation:
pip install --upgrade pip
pip install rospy

Build workspace with:
catkin build

Source the workspace so ROS tooling finds your packages with:
source devel/setup.bash

Run package1 and package2 with: 
roscore
rosrun package1 nodeA.py 
rosrun package2 nodeB.py