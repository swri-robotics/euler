cmake_minimum_required(VERSION 2.8.3)
project(euler_navigation)

find_package(catkin REQUIRED)

catkin_package()

install(FILES
   launch/move_base.launch 
   euler_nav.rviz 
   params/costmap_common.yaml 
   params/dwa.yaml 
   params/move_base.yaml
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


