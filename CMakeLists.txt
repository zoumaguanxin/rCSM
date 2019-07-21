cmake_minimum_required(VERSION 2.8)

project(rcsm)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3 -march=native")

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
endif()

find_package(catkin REQUIRED COMPONENTS
roscpp
tf
tf2
geometry_msgs
sensor_msgs
tf2_ros
pcl_ros
nav_msgs
)
find_package(PCL 1.7 REQUIRED)

find_package(Eigen3 REQUIRED)

include_directories(${catkin_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS} src/)

SET(SRCS src/point_cloud.cpp src/real_time_correlative_scan_matcher.cpp)

add_library(scanMatcher ${SRCS})

add_executable(rcsm main.cpp)

add_executable(testrcsm test.cpp)

target_link_libraries(rcsm ${catkin_LIBRARIES} ${PCL_LIBRARIES})

target_link_libraries(testrcsm ${catkin_LIBRARIES}  ${PCL_LIBRARIES})

#install(TARGETS rcsm RUNTIME DESTINATION bin)
