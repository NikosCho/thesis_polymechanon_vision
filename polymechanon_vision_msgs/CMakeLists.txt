cmake_minimum_required(VERSION 2.8.3)
project(polymechanon_vision_msgs)

find_package(catkin REQUIRED COMPONENTS std_msgs message_generation sensor_msgs geometry_msgs)

add_message_files(FILES
	QrHzlMsg.msg
)

generate_messages(DEPENDENCIES 
	std_msgs 
	geometry_msgs
)

catkin_package(CATKIN_DEPENDS std_msgs message_runtime geometry_msgs)

include_directories(
  ${catkin_INCLUDE_DIRS} )


# How to inlcude them:

# CMakeLists--
# cmake_minimum_required(VERSION 2.8.3)
# project(foo)
# find_package(catkin REQUIRED COMPONENTS foo_msgs)
# catkin_package()
# include_directories(include ${catkin_INCLUDE_DIRS})
# add_executable(foo_node src/foo_node.cpp)
# add_dependencies(foo_node foo_msgs_gencpp)

# package.xml---
# <build_depend>foo_msgs</build_depend>
# <run_depend>foo_msgs</run_depend>

# .cpp--
# #include <iostream>
# #include <polymechanon_vision_msgs/QrHzlMsg.h>
# int main(void) {
#     polymechanon_vision_msgs::QrHzlMsg msg;
#     msg.bar = 4;
#     std::cout << msg << std::endl;
# }

