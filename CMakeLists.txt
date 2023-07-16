cmake_minimum_required(VERSION 3.0.2)
project(test_ros_comm)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)


catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES test_ros_comm
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/test_ros_comm.cpp
# )

add_executable(publish_heartbeat src/publish_heartbeat.cpp)
target_link_libraries(publish_heartbeat
  ${catkin_LIBRARIES}
)

add_executable(respond_to_heartbeats src/respond_to_heartbeats.cpp)
target_link_libraries(respond_to_heartbeats
  ${catkin_LIBRARIES}
)
