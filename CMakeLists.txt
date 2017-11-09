cmake_minimum_required(VERSION 2.8.3)
project(free_gait_trajectory_optimization)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")  # -O3 -mavx -mfma
set(CMAKE_CXX_FLAGS "-g ${CMAKE_CXX_FLAGS}")

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
        roscpp
        roslib
        ds_base
        ds_systems
        ds_visual_base
        pointfootleg_visualization
        ds_anymal_visualization
        legged_robot_task_dt
        direct_trajectory_optimization
        anymal_beth_sim
  free_gait_core
  free_gait_ros
  kindr
)

## System dependencies are found with CMake's conventions
find_package(Eigen3 REQUIRED)
find_library(SNOPT_LIBRARY1 snopt7_cpp  /usr/local/lib/snopt7)
find_library(SNOPT_LIBRARY2 snopt7      /usr/local/lib/snopt7)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS
    include
    ${EIGEN3_INCLUDE_DIR}
  LIBRARIES
  CATKIN_DEPENDS
    free_gait_core
    free_gait_ros
    kindr
  DEPENDS
    eigen
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

## Declare a cpp executable
add_executable(
  ${PROJECT_NAME}
  src/free_gait_trajectory_optimization_node.cpp
  src/SendTrajectory.cpp
  src/ReceiveTrajectory.cpp
  src/SolveTrajectoryOptimization.cpp
)

## Declare a cpp executable
add_executable(
        free_gait_trajectory_optimization_rviz
        src/free_gait_trajectory_optimization_rviz.cpp
        src/ReceiveTrajectory.cpp
        src/SolveTrajectoryOptimization.cpp
)


set_target_properties(
        ${PROJECT_NAME}
        PROPERTIES COMPILE_DEFINITIONS "COMPILE_SNOPT")

set_target_properties(
        free_gait_trajectory_optimization_rviz
        PROPERTIES COMPILE_DEFINITIONS "COMPILE_SNOPT")

## Specify libraries to link a library or executable target against
target_link_libraries(
        ${PROJECT_NAME}
        ${catkin_LIBRARIES}
        ${SNOPT_LIBRARY1} ${SNOPT_LIBRARY2}
)

target_link_libraries(
        free_gait_trajectory_optimization_rviz
        ${catkin_LIBRARIES}
        ${SNOPT_LIBRARY1} ${SNOPT_LIBRARY2}
)

#############
## Testing ##
#############
