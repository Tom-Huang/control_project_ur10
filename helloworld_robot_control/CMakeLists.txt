cmake_minimum_required(VERSION 2.8.3)
project(helloworld_robot_control)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tum_ics_ur_robot_lli
  tum_ics_ur_robot_msgs
  message_generation
  cv_tracker
  tum_ics_lacquey_gripper_msgs
  )

##############################################################################
# Eigen
##############################################################################
find_package(Eigen3 REQUIRED)
#find_package(PCL 1.2 REQUIRED)

add_definitions(${EIGEN_DEFINITIONS})

find_package(TMath REQUIRED)
add_definitions(${TMATH_DEFINITIONS})
include_directories(${TMATH_INCLUDE_DIRS})

find_package(RtThreads REQUIRED)
include_directories(${RTTHREADS_INCLUDE_DIR})


## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

# Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#
# )

# Generate services in the 'srv' folder
 add_service_files(
    DIRECTORY srv
    FILES
    setTrajectoryGoal.srv
    TorqueFusionControlCommander.srv
    TorqueFusionSubcontrollerList.srv
    ArucoRequest.srv
 )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
 )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES helloworld_robot_controllers
  CATKIN_DEPENDS roscpp std_msgs tum_ics_ur_robot_lli tum_ics_ur_robot_msgs message_runtime
  DEPENDS
)

find_package(Qt4 REQUIRED)
SET(QT_USE_QTNETWORK TRUE)

include(${QT_USE_FILE})
ADD_DEFINITIONS(-DQT_NO_KEYWORDS)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
  ${roscpp_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${CMAKE_CURRENT_BINARY_DIR}
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3")
set(CMAKE_BUILD_TYPE Release)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/helloworld_robot_control.cpp
# )



add_library(helloworld_robot_controllers
        src/ControlDataPublisher.cpp include/helloworld_robot_control/ControlDataPublisher.h
        src/TorqueFusionController.cpp include/helloworld_robot_control/TorqueFusionController.h
        src/PDGControl.cpp include/helloworld_robot_control/PDGControl.h
        src/PIDGControl.cpp include/helloworld_robot_control/PIDGControl.h
        src/LeapControl.cpp include/helloworld_robot_control/LeapControl.h
        src/OrientationControl.cpp include/helloworld_robot_control/OrientationControl.h
        src/OrientationLeapControl.cpp include/helloworld_robot_control/OrientationLeapControl.h
        src/GravityControl.cpp include/helloworld_robot_control/GravityControl.h
        include/helloworld_robot_control/hasRunning.h
        )

target_link_libraries(helloworld_robot_controllers
        ${catkin_LIBRARIES}
        ${roscpp_LIBRARIES}
        ${tum_ics_ur_robot_lli_LIBRARIES}
        ${QT_LIBRARIES}
        ${RTTHREADS_LIBRARIES}
        ${TMATH_LIBRARIES}
        )
add_dependencies(helloworld_robot_controllers
        ${tum_ics_ur_robot_msgs_EXPORTED_TARGETS}
        helloworld_robot_control_generate_messages_cpp
        #        ${sensor_msgs_EXPORTED_TARGETS}
        ${std_msgs_EXPORTED_TARGETS}
#        ${visualization_msgs_EXPORTED_TARGETS}
        )

set(APP_TARGETS
        HelloworldTester
        )

foreach(target ${APP_TARGETS})
    add_executable(${target} src/Applications/${target}.cpp)
    target_link_libraries(${target}
            helloworld_robot_controllers
            ${catkin_LIBRARIES}
            ${roscpp_LIBRARIES}
            ${tum_ics_ur_robot_lli_LIBRARIES}
            ${QT_LIBRARIES}
            ${RTTHREADS_LIBRARIES}
            ${TMATH_LIBRARIES})
    add_dependencies(${target}
            ${tum_ics_ur_robot_msgs_EXPORTED_TARGETS}
#            ${sensor_msgs_EXPORTED_TARGETS}
            ${std_msgs_EXPORTED_TARGETS}
#            ${visualization_msgs_EXPORTED_TARGETS}
            )
endforeach()

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/helloworld_robot_control_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
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
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_helloworld_robot_control.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
