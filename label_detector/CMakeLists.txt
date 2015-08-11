cmake_minimum_required(VERSION 2.8.3)
project(label_detector)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  dynamic_reconfigure
  polymechanon_vision_msgs
  OpenCV
  cv_bridge
  image_transport
)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules/")
find_package(Zbar REQUIRED)

set(PACKAGE_PATH "${PROJECT_SOURCE_DIR}")
configure_file(${PROJECT_SOURCE_DIR}/config/config_PP.h.in ${PROJECT_SOURCE_DIR}/include/label_detector/config_PP.h)

generate_dynamic_reconfigure_options(
	dyn_cfg/LabelDetector.cfg
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES test_package
#  CATKIN_DEPENDS roscpp rospy std_msgs
  CATKIN_DEPENDS polymechanon_vision_msgs
#  DEPENDS system_lib
)

#enable c++11 features ??
# list( APPEND CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS} -g -ftest-coverage -fprofile-arcs")
# list( APPEND CMAKE_CXX_FLAGS "-std=c++1y ${CMAKE_CXX_FLAGS} -g -ftest-coverage ")
list( APPEND CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS} -g -ftest-coverage ")


###########
## Build ##
###########

include_directories( include 
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Zbar_INCLUDE_DIR}
)  

# include_directories(
#   ${catkin_INCLUDE_DIRS}
# )

add_executable(label_detector_node 
    src/label_detector_main.cpp
    src/TEST_c_label_detector_nodehandler.cpp
    src/c_label_detector_nodehandler.cpp
    src/c_label_detector.cpp
    src/c_label.cpp
    #scanners
    src/c_scanner.cpp
    src/scanners/c_qr_scanner.cpp
    src/scanners/c_hzl_scanner.cpp
    )

target_link_libraries( label_detector_node ${catkin_LIBRARIES})
target_link_libraries( label_detector_node ${OpenCV_LIBRARIES})
target_link_libraries( label_detector_node ${ZBAR_LIBRARIES} )

add_dependencies(label_detector_node polymechanon_vision_msgs_gencpp)


#############
## Testing ##
#############


file(GLOB_RECURSE TESTS ${PROJECT_SOURCE_DIR}/test/*.cpp)

foreach(ITEM ${TESTS})
    get_filename_component(filename ${ITEM} NAME_WE)
    catkin_add_gtest(${filename} ${ITEM}  src/c_label_detector.cpp
                                          src/c_label.cpp
                                          src/c_scanner.cpp
                                          src/scanners/c_qr_scanner.cpp)
    target_link_libraries(${filename} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${Zbar_INCLUDE_DIR})
endforeach()


# catkin_make run_tests_label_detector

# git fetch --all
# git reset --hard origin/master