cmake_minimum_required(VERSION 2.8.3)
project(topic_example)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
  
  message_generation
  message_runtime
)
add_message_files(
  FILES
  laserscan_filted.msg
)
generate_messages(
  DEPENDENCIES
  std_msgs
)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES topic_example
  CATKIN_DEPENDS message_runtime
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
)



#add_executable(publish_node src/laserscan_filter.cpp 
#			    src/mbot_linux_serial.cpp)
add_executable(publish_node src/laserscan_filter.cpp)
target_link_libraries(publish_node ${catkin_LIBRARIES})
add_dependencies(publish_node ${PROJECT_NAME}_generate_messages_cpp)

#add_executable(test_pwm src/test_pwm.cpp)

#target_link_libraries(test_pwm ${catkin_LIBRARIES})
