cmake_minimum_required(VERSION 2.8.3)
project(landing_zone_detection)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  geometry_msgs
  roscpp
  sensor_msgs
  std_msgs
  cv_bridge
  image_transport
  message_generation
)

add_message_files(
   FILES
   LandingZone.msg
)

generate_messages(
   DEPENDENCIES
   std_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(landing_zone_detector_node src/landing_zone_detector.cpp)

target_link_libraries(landing_zone_detector_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
