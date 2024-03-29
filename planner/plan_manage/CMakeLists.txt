cmake_minimum_required(VERSION 2.8.3)
project(plan_manage)
ADD_COMPILE_OPTIONS(-std=c++11 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(nlopt REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  kr_mav_msgs
  kr_tracker_msgs
  plan_env
  path_searching
  bspline
  bspline_opt
  traj_utils
  message_generation
  cv_bridge
  nlopt
)

# Generate messages in the 'msg' folder
add_message_files(
  FILES
  Bspline.msg
  )
  
# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

# catkin_package(CATKIN_DEPENDS message_runtime)
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES plan_manage
 CATKIN_DEPENDS plan_env path_searching bspline bspline_opt traj_utils message_runtime nlopt
#  DEPENDS system_lib
)

include_directories(
  include
  SYSTEM
  ${catkin_INCLUDE_DIRS} 
  ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)


add_executable(planner_node
  src/fast_planner_node.cpp 
  src/kino_replan_fsm.cpp
  src/topo_replan_fsm.cpp
  src/planner_manager.cpp
  )
target_link_libraries(planner_node 
  ${catkin_LIBRARIES}
  )
