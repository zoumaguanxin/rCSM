cmake_minimum_required(VERSION 2.8)

project(rcsm)

find_package(catkin REQUIRED COMPONENTS
roscpp
tf
tf2
geometry_msgs
sensor_msgs
tf2_ros
pcl_ros
)

find_package(Eigen3 REQUIRED)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(rcsm main.cpp)

add_executable(testrcsm test.cpp)

target_link_libraries(rcsm ${catkin_LIBRARIES})

target_link_libraries(testrcsm ${catkin_LIBRARIES})

#install(TARGETS rcsm RUNTIME DESTINATION bin)
