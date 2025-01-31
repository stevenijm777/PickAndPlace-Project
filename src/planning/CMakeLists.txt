cmake_minimum_required(VERSION 3.0.2)
project(planning)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp moveit_ros_planning_interface
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES planning
  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
## Declare ROS dynamic reconfigure parameters ##
)

###########
## Build ##
###########
#include_dire
## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/planning.cpp
# )

## Add cmake target dependencies of the library
## Declare ROS dynamic reconfigure parameters ##
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/planning_node.cpp)
add_executable(run src/run.cpp src/planning.cpp include/planning.h)
target_link_libraries(run ${catkin_LIBRARIES})

add_executable(banda src/banda.cpp)
target_link_libraries(banda ${catkin_LIBRARIES})

