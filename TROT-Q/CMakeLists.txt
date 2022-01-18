cmake_minimum_required(VERSION 2.8.12)
project(trot-q)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14") # if want to use auto in Lamda function, should be more recent than c++14
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  cv_bridge
  pcl_conversions
  pcl_ros
)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(octomap REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
)

catkin_package()

add_executable(trot-q_node src/main.cpp)
target_link_libraries(trot-q_node ${catkin_LIBRARIES} ${EIGEN3_LIBS} ${OpenCV_LIBS} ${OCTOMAP_LIBRARIES}
)