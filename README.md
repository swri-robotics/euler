euler
=====

SwRI mobile manipulation platform



Navigation Interface
--------------------

In its current state, the navigation configuration assumes a number of different topics in the euler namespace
 - `/euler/scan1` and `/euler/scan2` - navigation subscribes to the two LaserScans provided by the laser scanners on the base
 - `/euler/cmd_vel` - navigation publishes the Twist message used by the base for driving
 - `/euler/odom` - navigation subscribes to Odometry messages

In addition, it also assumes that the following transforms are published. 
 - `/odom --> /base_link`
 - `/base_link --> X` (where X is the frames for that the laser data is published in)
 - `/map --> /odom` - This is the localization step

The navigation configuration provides numerous topics for visualizing the robot's progress toward the goal, too many to list here. 

In terms of input, you can specify goals by publishing on the topic `/move_base_simple/goal`, which is what RViz does. However, the higher-level smach-compatibile Action interface is also available, as a [MoveBaseAction](http://docs.ros.org/api/move_base_msgs/html/action/MoveBase.html) with the namespace `/move_base`. 
