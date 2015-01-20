##LINK@RAHUL::http://wiki.ros.org/catkin/CMakeLists.txt


cmake_minimum_required(VERSION 2.8.3)
project(catkin_tutorial)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED #COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
  message_generation
)


## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)


##@Rahul:LINK::http://wiki.ros.org/th/ROS/Tutorials/CreatingMsgAndSrv ::Tutorial for creating msgs and srvs
##@Rahul: Below are the setting done for including the messages and services present in the msg and srv
##directories of the catkin_tutorial package.
##@Rahul: 4-places need to be modified for accessing the srv and msg files

add_message_files(
   FILES
   Num.msg
)

add_service_files(
   DIRECTORY srv
   FILES
   Multiply.srv
)

## Generate actions in the 'action' folder
#add_action_files(
#   FILES
#   Action1.action
#   Action2.action
#)

#@Rahul: For Hydro and later we need to uncomment below lines.
##For <rosmake> tool it is simply <generate_messages()>
## Generate added messages and services with any dependencies listed here
#generate_messages(
#  DEPENDENCIES
#  geometry_msgs
#   std_msgs
#)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need

##@Rahul:<message_runtime> To export the message runtime dependency:: I dont understand this meaning.

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES catkin_tutorial
#  CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs
#  DEPENDS system_lib
   CATKIN_DEPENDS message_runtime
   DEPENDS roscpp std_msgs 
)

###########
## Build ##
###########


## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(include 
  ${catkin_INCLUDE_DIRS}
  ${roscpp_INCLUDE_DIRS}
  ${rospy_INCLUDE_DIRS}
)
#Adding executable and their linked libraries  #@Rahul:add_dependencies#what does this mean??

add_library(serial1 src/serial.cpp)
add_dependencies(serial1 catkin_tutorial_gencpp)
target_link_libraries(serial1 ${roscpp_LIBRARIES} ${rospy_LIBRARIES})



add_executable(subscriber_node src/subscriber_node.cpp)
target_link_libraries(subscriber_node ${roscpp_LIBRARIES} ${rospy_LIBRARIES})

#add_executable(publisher_talker src/publisher_talker.cpp)
#target_link_libraries(publisher_talker ${roscpp_LIBRARIES} ${rospy_LIBRARIES})

add_executable(serialtest src/serialtest.cpp)
#add_dependencies(serialtest catkin_tutorial_gencpp)
target_link_libraries(serialtest ${roscpp_LIBRARIES} ${rospy_LIBRARIES} serial1)

add_executable(serial_publisher_node src/serial_publisher_node.cpp)
target_link_libraries(serial_publisher_node ${roscpp_LIBRARIES} ${rospy_LIBRARIES} serial1)

add_executable(serial_subscriber_node src/serial_subscriber_node.cpp)
target_link_libraries(serial_subscriber_node ${roscpp_LIBRARIES} ${rospy_LIBRARIES} serial1)

#add_executable(serialtest2 src/serialtest2.cpp)
#add_dependencies(serialtest2 catkin_tutorial_gencpp)
#target_link_libraries(serialtest2 ${roscpp_LIBRARIES} ${rospy_LIBRARIES})

#add_executable(multiply_server src/server.cpp)
#add_dependencies(multiply_server catkin_tutorial_gencpp )   			
#target_link_libraries(multiply_server ${catkin_LIBRARIES} ${roscpp_LIBRARIES} )

#add_executable(multiply_client src/client.cpp)
#target_link_libraries(multiply_client ${roscpp_LIBRARIES} ${catkin_LIBRARIES} )
#add_dependencies(multiply_client catkin_tutorial_gencpp)




##@Rahul: gencpp: means generate cpp

#${catkin_tutorial_EXPORTED_TARGETS} 
#catkin_tutorial_generate_messages_cpp 
#catkin_tutorial_generate_services_cpp 

## Declare a cpp library
# add_library(catkin_tutorial
#   src/${PROJECT_NAME}/catkin_tutorial.cpp
# )

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(catkin_tutorial_node catkin_tutorial_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(catkin_tutorial_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
install(TARGETS 
   #catkin_tutorial catkin_tutorial_node
   #publisher_talker
   #subscriber_node
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_catkin_tutorial.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
