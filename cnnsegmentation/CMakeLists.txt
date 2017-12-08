cmake_minimum_required(VERSION 2.8.3)
project(cnnsegmentation)

find_package(catkin REQUIRED 
  COMPONENTS 
    geometry_msgs 
    roscpp
    modules_perception_msgs
    modules_perception_cnnseg_msgs
    pcl_conversions
    hdmapinput
)
SET( CMAKE_CXX_FLAGS "-std=c++11")
# glog
find_package(Glog REQUIRED)
include_directories(BEFORE ${GLOG_INCLUDE_DIRS}) 
#PCL
find_package(PCL REQUIRED)

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs roscpp modules_perception_msgs modules_perception_cnnseg_msgs)

## Declare a cpp library
add_library(${PROJECT_NAME} src/cnn_segmentation.cpp)

add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
  ${Boost_INCLUDE_DIRS}
)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_INCLUDE_DIRS}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h")
