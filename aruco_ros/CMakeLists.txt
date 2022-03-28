cmake_minimum_required(VERSION 2.8.3)
project(aruco_ros)
add_compile_options(-std=c++11)
find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  dynamic_reconfigure
  geometry_msgs
  image_transport
  roscpp
  rospy
  tf
  aruco
  aruco_msgs
  sensor_msgs
  visualization_msgs
  moveit_msgs
  moveit_ros_perception
  moveit_ros_planning_interface
  trajectory_msgs
        rviz_visual_tools
)

generate_dynamic_reconfigure_options(
  cfg/ArucoThreshold.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES aruco_ros_utils
  CATKIN_DEPENDS roscpp  moveit_core moveit_ros_planning_interface aruco sensor_msgs tf
)

include_directories(
  include
)
include_directories(
  SYSTEM
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
add_library(aruco_ros_utils src/aruco_ros_utils.cpp)
target_link_libraries(aruco_ros_utils ${catkin_LIBRARIES})

add_executable(single src/simple_single.cpp
                      src/aruco_ros_utils.cpp)
add_dependencies(single ${PROJECT_NAME}_gencfg)
target_link_libraries(single ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(double src/simple_double.cpp
                      src/aruco_ros_utils.cpp)
add_dependencies(double ${PROJECT_NAME}_gencfg)
target_link_libraries(double ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(marker_publisher src/marker_publish.cpp
                                src/aruco_ros_utils.cpp)
add_dependencies(marker_publisher ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})
target_link_libraries(marker_publisher ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})


add_executable(charuco src/charuco.cpp
                       src/aruco_ros_utils.cpp )
add_dependencies(charuco ${PROJECT_NAME}_gencfg)
target_link_libraries(charuco ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})








