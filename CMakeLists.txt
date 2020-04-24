cmake_minimum_required(VERSION 2.8.3)
project(RoboticArm)


## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg)


add_compile_options(-std=c++11)
SET(SRC_LIST src/ArmNode.cpp src/main.cpp src/arm.cpp)
SET(SRC_LIST1 src/ulgy.cpp src/main.cpp src/arm.cpp)
## Declare ROS messages and services
add_message_files(FILES state.msg controls.msg setpoint.msg)
#add_service_files(FILES AddTwoInts.srv)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package()


## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})
#include_directories(/usr/local/include/serial)

#add_executable(auxiliary src/main.cpp src/datapackage.cpp src/arm.cpp)
add_executable(armcontrol ${SRC_LIST})
target_link_libraries(armcontrol ${catkin_LIBRARIES})
#add_dependencies(auxiliary beginner_tutorials_generate_messages_cpp)

add_executable(ulgy ${SRC_LIST})
target_link_libraries(ulgy ${catkin_LIBRARIES})
#add_executable(listener src/listener.cpp)
#target_link_libraries(listener ${catkin_LIBRARIES})
#add_dependencies(listener beginner_tutorials_generate_messages_cpp)
