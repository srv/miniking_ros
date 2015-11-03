cmake_minimum_required(VERSION 2.8.3)
project(miniking_ros)

find_package(catkin REQUIRED COMPONENTS
  driver_base
  dynamic_reconfigure
  roscpp
  sensor_msgs
  std_msgs
  std_srvs
  message_generation
)
find_package(libminiking REQUIRED)
find_package(OpenCV REQUIRED)

add_message_files(
  DIRECTORY msg
  FILES AcousticBeam.msg
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

include_directories(include ${catkin_INCLUDE_DIRS} ${libminiking_INCLUDE_DIRS})
add_executable(miniking_node src/miniking_node.cpp
                             src/miniking_ros.cpp)
add_dependencies(miniking_node miniking_ros_gencfg)
add_dependencies(miniking_node miniking_ros_gencpp)
target_link_libraries(miniking_node
  ${catkin_LIBRARIES}
  ${libminiking_LIBRARIES}
)

add_executable(acoustic_beam_viewer src/acoustic_beam_viewer.cpp)
add_dependencies(acoustic_beam_viewer miniking_ros_gencpp)
target_link_libraries(acoustic_beam_viewer
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${libminiking_LIBRARIES}
)

# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

install(TARGETS miniking_node acoustic_beam_viewer
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
