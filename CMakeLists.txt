cmake_minimum_required(VERSION 2.8.3)
project(proj1_heron)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  sensor_msgs
  gazebo_msgs
  tf
  cv_bridge
  image_transport
  std_msgs
)

find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})


## Add folders to be run by python nosetests
# catkin_add_nosetests(test)





################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################

catkin_package(
#    INCLUDE_DIRS include
#    LIBRARIES msg2tf
    CATKIN_DEPENDS nav_msgs sensor_msgs tf gazebo_msgs opencv std_msgs
#    DEPENDS 
)

###########
## Build ##
###########

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)


## Declare a C++ executable

add_executable(camera_subscriber_node src/camera_sub_P1.cpp )
add_executable(heron_controller_node src/heron_move_P2.cpp )



## Rename C++ executable without prefix
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
#set_target_properties(heronsafe_node PROPERTIES OUTPUT_NAME heronsafe PREFIX "")

## Add cmake target dependencies of the executable

add_dependencies(camera_subscriber_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(heron_controller_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})



## Specify libraries to link a library or executable target against

target_link_libraries(camera_subscriber_node
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
)
target_link_libraries(heron_controller_node
${catkin_LIBRARIES}
${OpenCV_LIBRARIES}
)


#############
## Install ##
#############

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
#install(TARGETS heronsafe_node
#    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

## Mark cpp header files for installation
#install(DIRECTORY include/
#    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#    FILES_MATCHING PATTERN "*.h"
#    PATTERN ".svn" EXCLUDE
#)

## Mark other files for installation (e.g. launch and bag files, etc.)
#install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
#install(DIRECTORY cfg DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/cfg)
