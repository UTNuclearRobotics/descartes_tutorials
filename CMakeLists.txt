cmake_minimum_required(VERSION 2.8.3)
project(vf_descartes)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  descartes_core
  descartes_moveit
  descartes_trajectory
  descartes_planner
  trajectory_msgs
  geometry_msgs
  moveit_ros_planning_interface
  move_interface
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

 add_executable(${PROJECT_NAME}_plan_d_path src/plan_d_path.cpp)

 target_link_libraries(${PROJECT_NAME}_plan_d_path
   ${catkin_LIBRARIES}
 )

set_target_properties(${PROJECT_NAME}_plan_d_path
  PROPERTIES OUTPUT_NAME plan_d_path
  PREFIX "")



#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}_plan_d_path
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
