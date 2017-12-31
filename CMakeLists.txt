cmake_minimum_required(VERSION 2.8.3)
project(extract_zed_data)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  rosbag
  roscpp
  sensor_msgs
  message_filters
  camera_calibration_parsers
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(OpenCV REQUIRED)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

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
   INCLUDE_DIRS include
#  LIBRARIES extract_zed_data
   CATKIN_DEPENDS cv_bridge image_transport rosbag roscpp sensor_msgs message_filters camera_calibration_parsers
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/extract_zed_data.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
add_executable(extract_rosbag_zed
  src/extract_rosbag_zed.cpp
  src/DepthProcessor.cpp
  src/ImageIOpfm.cpp
)
target_link_libraries(extract_rosbag_zed
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_executable(extract_topic_zed
  src/extract_topic_zed.cpp
  src/DepthProcessor.cpp
  src/ImageIOpfm.cpp
  src/ZedData.cpp
)
target_link_libraries(extract_topic_zed
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_extract_zed_data.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
