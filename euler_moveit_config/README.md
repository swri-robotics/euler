==== Usage
===== Run in Simulation

  In a terminal run the following command: 

  ```
  roslaunch euler_moveit_config planning_and_execution.launch 
  ```

===== Run in Real Mode
  - In the robot controller:
    - Switch the controller into **Play** mode
    - Press the **Servo On** button
    - Switch the robot controller into  **Remote** mode

  - In a sourced ROS terminal run the following launch file:
  ```
  roslaunch euler_moveit_config planning_and_execution.launch sim:=<true|false> robot_ip:=<value>
  ```
  - In another ROS terminal call the following service to enable the robot:
  ```
  rosservice call /robot_enable "{}"
  ```

===== Plan and Execute
  - In the RViZ window drag the robot tool to the desired location
  - Click the **Plan** button in the *Planning* tab and verify the plan
  - Click the **Execute** button and the robot should move.
