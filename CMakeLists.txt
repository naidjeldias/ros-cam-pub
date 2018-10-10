cmake_minimum_required(VERSION 2.8.3)
project(cam_pub)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
)
## Find opencv 
find_package(OpenCV REQUIRED)

find_package(PkgConfig)

pkg_check_modules(GTKMM gtkmm-3.0)
link_directories(${GTKMM_LIBRARY_DIRS})

pkg_check_modules(V4L2 libv4l2 libv4lconvert)
link_directories(${V4L2_LIBRARY_DIRS})

set(SOURCE_FILES
    cam_pub_node.cpp
    vcap/vcap.cpp
    vcap/device.cpp
)

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
   #INCLUDE_DIRS include
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  #include
  ${GTKMM_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${V4L2_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

##set(CMAKE_CXX_FLAGS "-std=c++14 -g -m64 -pipe -pthread -lpthread -lopencv_shape -lopencv_stitching -lopencv_objdetect -lopencv_superres -lopencv_videostab -lopencv_calib3d -lopencv_features2d -lopencv_highgui -lopencv_videoio -lopencv_imgcodecs -lopencv_video -lopencv_photo -lopencv_ml -lopencv_imgproc -lopencv_flann -lopencv_core -lboost_thread -lboost_system -lboost_filesystem")


## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(${PROJECT_NAME}_node src/cam_pub_node.cpp)

## Add cpp files as libraries 
add_library(vcap src/vcap/vcap.cpp src/vcap/device.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
  vcap
  ${OpenCV_LIBS}
  ${GTKMM_LIBRARIES}
  ${V4L2_LIBRARIES}
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
)

