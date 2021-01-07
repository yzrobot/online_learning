cmake_minimum_required(VERSION 2.8.3)
project(object3d_detector)

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs visualization_msgs people_msgs pcl_conversions pcl_ros bayes_tracking)

find_package(PCL REQUIRED)

include_directories(${catkin_INCLUDE_DIRS})

catkin_package(CATKIN_DEPENDS bayes_tracking)

### Offline version
add_executable(object3d_detector src/object3d_detector.cpp)
target_link_libraries(object3d_detector ${catkin_LIBRARIES})
if(catkin_EXPORTED_TARGETS)
  add_dependencies(object3d_detector ${catkin_EXPORTED_TARGETS})
endif()

### Online version
add_executable(object3d_detector_ol src/object3d_detector_ol.cpp)
target_link_libraries(object3d_detector_ol ${catkin_LIBRARIES})
if(catkin_EXPORTED_TARGETS)
  add_dependencies(object3d_detector_ol ${catkin_EXPORTED_TARGETS})
endif()
