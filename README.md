euler
=====

SwRI mobile manipulation platform

![Omni-Directional Euler Robot with Industrial Manipulator](EulerRobot.PNG)

# Installation:

## Prerequisites:

### Install CANFestival libraries for the CAN bus


The vetex platform is given velocity commands via the CAN bus, which uses the CANOpen protocol. The open source CANFestival libraries are used on top of linux's socketbus architecture to talk over the CANBus.

To install, download the libraries from www.canfestival.org - downloads. It will redirect you to their
main repo, which is at http://dev.automforge.net/  Download the CanFestival-3 source using the gz or bz2 button.

Once downloaded, uncompress/tar the file, go into the directory, and run the following commands:
 - `./configure --timers=unix --can=socket`
 - `make`
 - `sudo make install`
This will install the can festival libraries on the machine.

### ROS Install

 1. Install ROS [kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu). Make sure to install the ROS-Deskstop Full option.
 
 1. Get the wstool package
`sudo apt-get install python-wstool`

 1. Create a catkin worspace *euler_ws* using catkin tools as instructed [here](https://catkin-tools.readthedocs.io/en/latest/quick_start.html#initializing-a-new-workspace)
 
 1. Clone this repository into the source directory using *git clone ...*

 1. Download the required repositories using *wstool*
   ```
   cd euler_ws
   wstool merge -t src src/euler/euler.rosinstall
   wstool update
   ```
 6. Install binary dependencies 
  ```
  rosdep install --from-paths src --ignore-src
  ```

 7. Build Euler!  
  In the top level directory of the euler_ws workspace run the following  
  ```
  catkin build
  ```
  
 8. Source your workspace 
  ```
  source devel/setup.bash
  ```

At this point you should be able to run the demo application

# Navigation Demo
---------------------

## Prerequisites


Enable CAN bus at startup
-------------------------

When the CAN USB dongle is plugged in, the kernel in Ubuntu automatically recognizes it and creates a canbus interface for it, however it begins in the down state. The following lines need to be added to the box on boot (such as in rc.local) to setup the CAN parameters and to set the link up (this assumes that the dongle gets assigned can0):
 - `ip link set can0 type can bitrate 250000 restart-ms 50`
 - `ip link set can0 up`

* This step should only be needed once

Turn On Physical Devices
-------------------------
Make sure the following devices are up and running:
 - Vetex Mobile Base
 - SICK sensor
 - Motoman Arm
 
Setup ROS across multiple machines
---------------------------------------------
Follow the instructions to setup [ROS across Multiple machines](http://wiki.ros.org/ROS/Tutorials/MultipleMachines) in order 
to setup the euler computer as the master.  This way you can run rviz on your machine and avoid sending the graphics data from euler through the network.

## Run the Demo
- SSH into the machine using the ssh command and euler user name
 ```
 ssh euler@<ip>
 ```

- To run the navigation demo with input/driving via Rviz, run the following command. 
 - `roslaunch euler_navigation_demo demo.launch sim_localization:=false sensor_ip:=<sensor ip> sim_base:=false sim_robot:=false robot_ip:=<robot ip> gui:=false` 
 
 * The gui:=false argument is recomended in order to avoid running rviz remotely as it is very slow and unworkable in this mode.
 
 - Enable robot arm  
   SSH into another terminal and call the following service to enable the robot
   ```
   rosservice call /robot_enable {}
   ```
 - In your local machine run rviz
   ```
   roscd euler_navigation_demo
   rosrun rviz rviz -d rviz/euler.rviz
   ```
    * Make sure you have followed the instructions for setting ROS across multiple machines for this step to work
 
 - Command the base
  - In Rviz, click on "2D Navigation Goal" from the tools tab and then click a desire goal for the base.
  - You will then see the base move to that location.

Additional Navigation Interface Info
----------------------------------------

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



