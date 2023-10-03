cmake_minimum_required(VERSION 3.0.2)
project(td_ibvs_nsftasmc)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  gazebo_msgs
  geometry_msgs
  image_transport
  nav_msgs
  roscpp
  sensor_msgs
  std_msgs
  tf2
)

find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})

catkin_package(
CATKIN_DEPENDS cv_bridge gazebo_msgs geometry_msgs image_transport nav_msgs roscpp sensor_msgs std_msgs tf2
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_3_INCLUDE_DIRS}
)

add_executable(imfeat_node src/image_features.cpp)
target_link_libraries(imfeat_node  ${catkin_LIBRARIES})
target_link_libraries(imfeat_node  ${OpenCV_LIBRARIES})

add_executable(nftasmc_att_node src/attitude_nftasmc.cpp)
target_link_libraries(nftasmc_att_node ${catkin_LIBRARIES})

add_executable(ibvs_node src/ibvs_pos_ctrl.cpp)
target_link_libraries(ibvs_node ${catkin_LIBRARIES})

add_executable(quav_dynamics_node src/uav_dynamics.cpp)
target_link_libraries(quav_dynamics_node ${catkin_LIBRARIES})

add_executable(tgt_pos_node src/target_position.cpp)
target_link_libraries(tgt_pos_node ${catkin_LIBRARIES})

add_executable(dist_node src/disturbances.cpp)
target_link_libraries(dist_node ${catkin_LIBRARIES})

add_executable(fixed_td_node src/fxt_TD.cpp)
target_link_libraries(fixed_td_node ${catkin_LIBRARIES})