# CMPUT412 FALL 2019 - competition one report


- ***Purpose***  
	     In this competition, each group will implement the following behaviour in their ‘Cop’ robot, to be tested and compared with other robots in a competitive setting. The behaviour will be used to follow i.e. chase a leading ‘Robber’ robot (implemented by the TA) around an oval track without bumping into it. The ‘Robber’ robot will follow a white line and make three loops around a track in CSC 2-29. Your ‘Cop’ robot will be placed behind the Robber robot at the start of the competition. Once the Robber robot starts you must chase the Robber while avoiding ‘rear end’ collisions with it. Your robot is not allowed to follow the white line. It must use the depth sensor to follow the Robber robot.
	     
- ***Pre-requisites***
	- 
	The project is built with python2.7 on Ubuntu 16.04.
	Dependencies include ROS kinetic package, smach state machine, and other drivers for the turtle bot sensor. If these are not installed please refer to the official installation page on ROS wiki or official python installation websites.
	
- http://wiki.ros.org/kobuki/Tutorials/Installation/kinetic
- http://wiki.ros.org/kinetic/Installationu 
- https://www.python.org/downloads/

	The source code of the project can be found at [https://github.com/TianqiCS/CMPUT412-C1](https://github.com/TianqiCS/CMPUT412-C1)
	Create or navigate the existing catkin workspace and clone our repository.

-  ***Execution***
	- 
	Once you have the package in your workspace, you can launch the program using 
	```
	$roslaunch follow_bot follow_bot_real.launch
	```
	After the program is launch, use the joystick to control the turtle bot to follow the object.
- arguments and parameters
	In the launch file follow_bot_real.launch, the file will launch basic driver for the kuboki robot which is essential for the competition ( minimal.launch and 3dsensor.launch). Next, the file will bring up the basic node for this competition like nodelet and joysticks. Finally, there are different sections for in the launch file to give a different view of the field for the robot. By twisting parameters with the name filter, the robot can unaware of anything that is not the target object. How far the robot can see, how fast can the robot turn is also defined in the launch file for further adjustments to the competition.

- ***Concepts and Code***
	- 
Our code is based on the tutorial given by [https://github.com/pirobot/rbx1](https://github.com/pirobot/rbx1)
It is the following method based on the pointcloud2. The basic algorithm is:

● Initialize the position (x,y,z) of the tracking object
● Define a search box in front of the robot
● Define how close the robot should be to the object (i.e. close = goal position)
WHILE not shutdown do:
● Sample all the points in the point cloud within the searchBox
● With these search points do:
○ Compute the centroid of the region e.g. avg of x, y,  z value of search
points
○ If the centroid is not NULL (robot has found an object to track)
■ Z-coordinate is the distance to the object
■ X-coordinate says if the object is left or right of the robot
■ Compute Twist message to keep the robot close to the object
● Move the robot or stop

Besides we add some more features:
- if the target is lost, the robot will automatically turn left trying to find the target.
- the target is 1.3m away or further, the robot will accelerate faster.

