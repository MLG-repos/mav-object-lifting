cmake_minimum_required(VERSION 2.8.3)
project(control_frontal)

find_package(catkin REQUIRED COMPONENTS
  #parrot_arsdk
  image_transport
  bebop_msgs
  roscpp
  cv_bridge
  rospy
  sensor_msgs
  std_msgs
)

find_package(OpenCV REQUIRED)

catkin_package(
#  INCLUDE_DIRS include 
#  LIBRARIES control_frontal
#  CATKIN_DEPENDS roscpp parrot_arsdk cv_bridge sensor_msgs std_msgs image_transport
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  #${ARSDK_INCLUDE_DIR}
)

#link_directories(${ARDRONESDK3_PATH}/lib
#  ${ARSDK_LIB_DIR}
#)

include_directories(include include/control_frontal)

## Declare a C++ executable
add_executable(control_frontal 
				src/control_frontal_node.cpp 
				src/control_frontal_nodelet.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(control_frontal 
				${catkin_LIBRARIES}
				${OpenCV_LIBS})

