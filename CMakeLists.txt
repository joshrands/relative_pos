cmake_minimum_required(VERSION 2.8.3)
project(relative_pos)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  cv_bridge
  image_transport
	message_generation
)

find_package(OpenCV 3 REQUIRED)

add_message_files(
	FILES
	ArucoConfig.msg
)

generate_messages(
	DEPENDENCIES
	std_msgs
)

catkin_package(
  INCLUDE_DIRS
  CATKIN_DEPENDS roscpp rospy std_msgs cv_bridge image_transport message_runtime
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

link_directories(${OpenCV_LIBRARY_DIRS})

add_executable(talker src/talker.cpp)
target_link_libraries(talker ${catkin_LIBRARIES})
add_dependencies(talker beginner_tutorials_generate_messages_cpp)

add_executable(detect_arucos src/detect_arucos.cpp)
target_link_libraries(detect_arucos ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(get_pose src/get_pose.cpp)
target_link_libraries(get_pose ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
