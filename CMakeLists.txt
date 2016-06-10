cmake_minimum_required(VERSION 2.8.3)
project(miniking_ros)

find_package(catkin REQUIRED COMPONENTS
  driver_base
  dynamic_reconfigure
  roscpp
  sensor_msgs
  std_msgs
  std_srvs
  nav_msgs
  visualization_msgs
  message_generation
  tf
  cmake_modules
  laser_geometry
  message_filters
  occupancy_grid_utils
)
find_package(libminiking REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen REQUIRED)
find_package(occupancy_grid_utils REQUIRED)

add_message_files(
  DIRECTORY msg
  FILES AcousticBeam.msg ObstacleDistance.msg
)

generate_messages(
  DEPENDENCIES std_msgs
)

generate_dynamic_reconfigure_options(
  cfg/MiniKing.cfg
)

catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS driver_base dynamic_reconfigure roscpp sensor_msgs std_msgs std_srvs libminiking
)

include_directories(include 
  ${catkin_INCLUDE_DIRS} 
  ${libminiking_INCLUDE_DIRS} 
  ${Eigen_INCLUDE_DIRS} )

add_executable(miniking_node src/miniking_node.cpp
                             src/miniking_ros.cpp)
add_dependencies(miniking_node miniking_ros_gencfg miniking_ros_gencpp)
target_link_libraries(miniking_node
  ${catkin_LIBRARIES}
  ${libminiking_LIBRARIES}
)

add_executable(acousticbeam_viewer src/acousticbeam_viewer.cpp)
add_dependencies(acousticbeam_viewer miniking_ros_gencpp)
target_link_libraries(acousticbeam_viewer
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${libminiking_LIBRARIES}
)

add_executable(acousticbeam_to_laserscan src/acousticbeam_to_laserscan.cpp)
add_dependencies(acousticbeam_to_laserscan miniking_ros_gencpp)
target_link_libraries(acousticbeam_to_laserscan
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${libminiking_LIBRARIES}
)

add_executable(laserscan_to_pointcloud src/laserscan_to_pointcloud.cpp)
target_link_libraries(laserscan_to_pointcloud
  ${catkin_LIBRARIES}
)

add_executable(obstacle_detector src/obstacle_detector.cpp)
add_dependencies(obstacle_detector miniking_ros_gencpp)
target_link_libraries(obstacle_detector
  ${catkin_LIBRARIES}
)
