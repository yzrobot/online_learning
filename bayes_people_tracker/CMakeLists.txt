cmake_minimum_required(VERSION 2.8.3)
project(bayes_people_tracker)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    bayes_tracking
    geometry_msgs
    message_generation
    people_msgs
    roscpp
    std_msgs
    tf
    visualization_msgs
)
find_package(Boost REQUIRED COMPONENTS thread)

#######################################
## Declare ROS messages and services ##
#######################################

add_message_files(
  FILES
  PeopleTracker.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS bayes_tracking geometry_msgs roscpp std_msgs tf visualization_msgs
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

### Offline version
add_executable(bayes_people_tracker
  src/people_tracker/people_tracker.cpp
)

add_dependencies(bayes_people_tracker ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

target_link_libraries(bayes_people_tracker
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

### Online version
#add_definitions(-DONLINE_LEARNING)

add_executable(bayes_people_tracker_ol
  src/people_tracker/people_tracker.cpp
)

add_dependencies(bayes_people_tracker_ol ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

target_link_libraries(bayes_people_tracker_ol
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

target_compile_definitions(bayes_people_tracker_ol PUBLIC ONLINE_LEARNING=1)

#############
## Install ##
#############

install(TARGETS bayes_people_tracker
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS bayes_people_tracker_ol
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".git" EXCLUDE
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
