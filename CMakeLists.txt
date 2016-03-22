# Synthetic 2D liDAR for precise vehicle localization in 3d urban environment
cmake_minimum_required(VERSION 2.8.3)
project(foobar)

set(${PROJECT_NAME}_CATKIN_DEPS
    angles
    nodelet
    pcl_ros
    roscpp
    roslib
    sensor_msgs
    tf
    velodyne_driver
    velodyne_msgs
    velodyne_pointcloud
    velodyne
    cv_bridge
    image_transport
   )

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  tf
  cv_bridge
  image_transport
  message_generation
  libpointmatcher_ros
)

MESSAGE( STATUS "CMAKE_MODULE_PATH: " ${CMAKE_MODULE_PATH} )
LIST(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/../Armadillo/CMake/Armadillo")

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(Boost)
find_package(Hiredis REQUIRED)
find_package( PCL REQUIRED )
find_package(Eigen REQUIRED)

find_package(OpenCV REQUIRED)
find_package(Hiredis REQUIRED)
find_package(Armadillo REQUIRED)

find_package(libpointmatcher REQUIRED)

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

## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   Canbus.msg
   Esr_track.msg
   GenericObjCanData.msg
#   Message1.msg
#   Message2.msg
 )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

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
#  INCLUDE_DIRS include
#  LIBRARIES beginner_tutorials
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  $ENV{HOME}/Downloads/IbeoSDK2.11.1/src
  ${HIREDIS_INCLUDE_DIR}
  /usr/include/hiredis
  ${PCL_INCLUDE_DIRS}
  ../velodyne/velodyne_pointcloud/include
  ${Eigen3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS} ${HIREDIS_INCLUDE_DIR} /usr/include/hiredis
  ${ARMADILLO_INCLUDE_DIRS}
  ${libpointmatcher_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(beginner_tutorials
#   src/${PROJECT_NAME}/beginner_tutorials.cpp
# )

## Declare a cpp executable
# add_executable(beginner_tutorials_node src/beginner_tutorials_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(beginner_tutorials_node beginner_tutorials_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(beginner_tutorials_node
#   ${catkin_LIBRARIES}
# )

#SET(CMAKE_INSTALL_RPATH "$ENV{HOME}/Downloads/IbeoSDK2.11.1/libs")

link_directories(${PCL_LIBRARY_DIRS} src)
add_definitions(${PCL_DEFINITIONS})

# Define a library target called hello_world
#add_library(IbeoSdkFileDemo src/IbeoSdkFileDemo.cpp)
#target_link_libraries(IbeoSdkFileDemo ${roscpp_LIBRARIES})

#add_library(IbeoSdkScalaLiveDemo src/IbeoSdkScalaLiveDemo.cpp)
#target_link_libraries(IbeoSdkScalaLiveDemo ${roscpp_LIBRARIES})

#add_executable(hello_world_node src/hello_world_node.cpp)
#target_link_libraries(hello_world_node ${roscpp_LIBRARIES} 
#  IbeoSdkFileDemo 
#  boost_system 
#  boost_thread 
#  boost_date_time 
#  boost_program_options
#  $ENV{HOME}/Downloads/IbeoSDK2.11.1/libs/libibeosdk_d.so
#  
#)
# ${PCL_LIBRARIES}

#add_executable(scala_coms_detect src/scala_coms_detect.cpp)
#target_link_libraries(scala_coms_detect ${roscpp_LIBRARIES} 
#  IbeoSdkScalaLiveDemo 
#  boost_system 
#  boost_thread 
#  boost_date_time 
#  boost_program_options
#  $ENV{HOME}/Downloads/IbeoSDK2.11.1/libs/libibeosdk_d.so
#  ${HIREDIS_LIBRARIES}
#
#)

#MESSAGE( STATUS "PCL_LIBRARIES : " ${PCL_LIBRARIES} )
# the compiler flags for compiling C sources 
MESSAGE( STATUS "CMAKE_C_FLAGS: " ${CMAKE_C_FLAGS} )

#SET( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -std=c++11 ")
SET( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 ")

# the compiler flags for compiling C++ sources 
MESSAGE( STATUS "CMAKE_CXX_FLAGS: " ${CMAKE_CXX_FLAGS} )

#ADD_LIBRARY(test STATIC IMPORTED)

add_executable(cloud_test_crf src/cloud_test_crf.cpp src/cloud_util.cpp src/ring_process.cpp src/common.cpp src/data/velo_data.cpp)
target_link_libraries(cloud_test_crf ${roscpp_LIBRARIES} ${PCL_LIBRARIES} ${Eigen3_LIBRARIES} ${HIREDIS_LIBRARIES} armadillo libtest.a
)

add_executable(cloud_repub src/cloud_repub.cpp)
target_link_libraries(cloud_repub ${roscpp_LIBRARIES} ${PCL_LIBRARIES} ${Eigen3_LIBRARIES} ${HIREDIS_LIBRARIES}
)

add_executable(cloud_filter src/cloud_filter.cpp)
target_link_libraries(cloud_filter ${roscpp_LIBRARIES} ${PCL_LIBRARIES} ${Eigen3_LIBRARIES} ${HIREDIS_LIBRARIES}
)

add_executable(image_saver src/image_saver.cpp)
target_link_libraries(image_saver ${roscpp_LIBRARIES} ${OpenCV_LIBS} ${HIREDIS_LIBRARIES} ${catkin_LIBRARIES}
)


add_executable(cloud_extract_one_ring src/cloud_extract_one_ring.cpp)
target_link_libraries(cloud_extract_one_ring ${roscpp_LIBRARIES} ${PCL_LIBRARIES} ${Eigen3_LIBRARIES} ${HIREDIS_LIBRARIES} 
)


add_executable(velo_2d src/velo_2d.cpp)
target_link_libraries(velo_2d ${roscpp_LIBRARIES} ${PCL_LIBRARIES} ${Eigen3_LIBRARIES} ${HIREDIS_LIBRARIES} 
  ${catkin_LIBRARIES}
  ${libpointmatcher_LIBRARIES}
)

#  ${PCL_LIBRARIES}
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
# install(TARGETS beginner_tutorials beginner_tutorials_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_beginner_tutorials.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
