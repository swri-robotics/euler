cmake_minimum_required(VERSION 2.8.3)
project(vetex_driver)

add_definitions("-std=c++0x")
find_package(catkin REQUIRED COMPONENTS
             roscpp
	     std_msgs
	     message_generation)

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES vetex_driver
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  /usr/local/include/canfestival)

# Declare a cpp executable
add_executable(vetex_driver
  src/vetex_driver.cpp
  src/vetexcanopen.cpp
  src/vetexcanopenSlaveOD.cpp
)


target_link_libraries(vetex_driver
	${catkin_LIBRARIES}
	/usr/local/lib/libcanfestival_unix.a
	/usr/local/lib/libcanfestival.a
	pthread
	rt
	dl
)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS vetex_driver
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#############
## Testing ##
#############

