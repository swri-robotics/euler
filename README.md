euler
=====

SwRI mobile manipulation platform

![Omni-Directional Euler Robot with Industrial Manipulator](EulerRobot.PNG)

# Installation

The following installation instruction are broken into multiple parts: depedencies installed from source, ROS install, and, amos source install

## Prerequisites:

### Install CANFestival libraries for the CAN bus
---------------------------------------------

The vetex platform is given velocity commands via the CAN bus, which uses the CANOpen protocol. The open source CANFestival libraries are used on top of linux's socketbus architecture to talk over the CANBus.

To install, download the libraries from www.canfestival.org - downloads. It will redirect you to their
main repo, which is at http://dev.automforge.net/  Download the CanFestival-3 source using the gz or bz2 button.

Once downloaded, uncompress/tar the file, go into the directory, and run the following commands:
 - `./configure --timers=unix --can=socket`
 - `make`
 - `sudo make install`
This will install the can festival libraries on the machine.

### ROS Install

 1. Install ROS
Follow all the instructions to install the base version of ROS [Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu): Install ROS-Deskstop Full. Please make sure you have followed all the ROS installation steps, including calls to `rosdep`.
 1. Get the wstool package
`sudo apt-get install python-wstool`

 1. Initialize the Euler workspace in your src directory.
```
wstool merge https://raw.githubusercontent.com/swri-robotics/euler/kinetic-devel/euler.rosinstall
wstool update
cd ..
```
 1. Make sure dependencies are installed.  *Note: If this step fails then you may have to install dependencies manually using `sudo apt-get <package>`
`rosdep install --from-paths src --ignore-src `
 1. Build Euler!
Assuming you are in the folder created above,
`catkin build`
 1. Setup your environment
You will have to do this every time you work with this particular source install of the code. Assuming you are in the moveit folder created above,
`source devel/setup.bash`


# Navigation Stage Demo
---------------------

## Prerequisites


Enable CAN bus at startup
-------------------------

When the CAN USB dongle is plugged in, the kernel in Ubuntu automatically recognizes it and creates a canbus interface for it, however it begins in the down state. The following lines need to be added to the box on boot (such as in rc.local) to setup the CAN parameters and to set the link up (this assumes that the dongle gets assigned can0):
 - `ip link set can0 type can bitrate 250000 restart-ms 50`
 - `ip link set can0 up`

* This step should only be needed once


## Run the Demo
To run the navigation demo with simulated input/driving (via [Stage](http://wiki.ros.org/stage_ros)), run the following command. 
 - `roslaunch euler_navigation_demo stage.launch`

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



