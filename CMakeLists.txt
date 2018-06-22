cmake_minimum_required(VERSION 2.8.3)
project(descartes_tutorials)

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

 add_executable(${PROJECT_NAME}_tutorial1 src/tutorial1.cpp)

 target_link_libraries(${PROJECT_NAME}_tutorial1
   ${catkin_LIBRARIES}
 )

set_target_properties(${PROJECT_NAME}_tutorial1
  PROPERTIES OUTPUT_NAME tutorial1
  PREFIX "")

 add_executable(${PROJECT_NAME}_tutorial2 src/tutorial2.cpp)

 target_link_libraries(${PROJECT_NAME}_tutorial2
   ${catkin_LIBRARIES}
 )

set_target_properties(${PROJECT_NAME}_tutorial2
  PROPERTIES OUTPUT_NAME tutorial2
  PREFIX "")

 add_executable(${PROJECT_NAME}_get_pose src/get_pose.cpp)


 target_link_libraries(${PROJECT_NAME}_get_pose
   ${catkin_LIBRARIES}
 )
set_target_properties(${PROJECT_NAME}_get_pose
  PROPERTIES OUTPUT_NAME get_pose
  PREFIX "")


 add_executable(${PROJECT_NAME}_shapely src/shapely.cpp)


 target_link_libraries(${PROJECT_NAME}_shapely
   ${catkin_LIBRARIES}
 )
set_target_properties(${PROJECT_NAME}_shapely
  PROPERTIES OUTPUT_NAME shapely
  PREFIX "")

 add_executable(${PROJECT_NAME}_tutorial_austin src/tutorial_austin.cpp)


 target_link_libraries(${PROJECT_NAME}_tutorial_austin
   ${catkin_LIBRARIES}
 )
set_target_properties(${PROJECT_NAME}_tutorial_austin
  PROPERTIES OUTPUT_NAME tutorial_austin
  PREFIX "")


#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}_tutorial1
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(TARGETS ${PROJECT_NAME}_tutorial2
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
