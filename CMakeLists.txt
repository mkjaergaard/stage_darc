cmake_minimum_required(VERSION 2.8)
project(stage_darc)

set(CMAKE_PREFIX_PATH /opt/ros/fuerte)
set(STAGE_PATH /opt/stage4)

find_package(catkin)
find_package(darc)
find_package(Boost COMPONENTS system thread)
find_package(ROS REQUIRED COMPONENTS cpp_common rostime roscpp_traits roscpp_serialization roscpp rosconsole darc tf2 tf2_darc tf2_msgs)

include_directories(include)
include_directories(${Boost_INCLUDE_DIRS})
include_directories(${ROS_INCLUDE_DIRS})
include_directories(${STAGE_PATH}/include/Stage-4.1)

link_directories(${STAGE_PATH}/lib64)

set(CMAKE_CXX_FLAGS "-Wall -g")
#set(CMAKE_CXX_FLAGS "-Os")

add_executable(stage_component src/stage_component.cpp src/stage_node.cpp)
target_link_libraries(stage_component ${Boost_LIBRARIES} ${ROS_LIBRARIES} -lstage)
