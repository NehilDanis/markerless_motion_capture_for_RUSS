cmake_minimum_required(VERSION 3.0.2)
project(shape_registration)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

find_package(PCL 1.11.1 REQUIRED)


find_package(Yaml-cpp 0.6.0 REQUIRED)
#find_package(OpenCV 4.5.2 REQUIRED core imgproc calib3d video features2d highgui)
#find_package(OpenCV 3 REQUIRED)
message("??? ${OpenCV_LIBS}")
message("??? END")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  iiwa_ros
  iiwa_msgs
  sensor_msgs
  visualization_msgs
  std_msgs
  image_transport
  cv_bridge
  geometry_msgs
  tf2
  tf2_ros
)

find_package(OpenCV 3 REQUIRED)

message("??? ${OpenCV_LIBS}")
message("??? END")

set(CMAKE_AUTOUIC ON)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES shape_registration
  CATKIN_DEPENDS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  visualization_msgs
  image_transport
  cv_bridge
  geometry_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  algorithms/include
  utils/include
  apps/include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS scripts/publisher_pyro.py scripts/image_saver.py scripts/movement_detector.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_executable(calibration_checker_node utils/calibration_checker_node.cpp)

add_executable(aruco_checker_node utils/aruco_checker_node.cpp)

add_executable(ROI_cropping_node utils/crop_roi.cpp)

add_executable(plane_segmentation_node utils/plane_segmentation.cpp utils/preprocessing.cpp)

add_executable(registration_node src/surface_registration.cpp algorithms/icp_algorithm.cpp utils/preprocessing.cpp)

add_executable(tracking_node src/tracking.cpp algorithms/icp_algorithm.cpp utils/preprocessing.cpp)

add_executable(find_trajectory_node utils/find_pca_of_artery.cpp)

add_executable(robot_controller utils/control_the_robot.cpp)

add_executable(test_depth_vals_node utils/test_depth_vals.cpp)

add_executable(hand_eye_manual_calibration utils/hand_eye_calibration.cpp algorithms/icp_algorithm.cpp utils/preprocessing.cpp)

add_executable(test_checker_board_node utils/test_checker_board.cpp)

target_link_libraries(test_depth_vals_node
   ${OpenCV_LIBRARIES}
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
 )


target_include_directories(calibration_checker_node PRIVATE ${YAML_CPP_INCLUDE_DIR})
message(STATUS "YAML-CPP INCLUDE DIR: ${YAML_CPP_INCLUDE_DIR}")

target_link_libraries(calibration_checker_node
   ${OpenCV_LIBRARIES}
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
   ${YAML_CPP_LIBRARIES}
 )


target_link_libraries(aruco_checker_node
   ${OpenCV_LIBRARIES}
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
 )

target_link_libraries(ROI_cropping_node
   ${OpenCV_LIBRARIES}
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
 )

target_include_directories(plane_segmentation_node PRIVATE ${YAML_CPP_INCLUDE_DIR})
message(STATUS "YAML-CPP INCLUDE DIR: ${YAML_CPP_INCLUDE_DIR}")
target_link_libraries(plane_segmentation_node
   ${OpenCV_LIBS}
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
   ${YAML_CPP_LIBRARIES}
 )

target_include_directories(registration_node PRIVATE ${YAML_CPP_INCLUDE_DIR})
message(STATUS "YAML-CPP INCLUDE DIR: ${YAML_CPP_INCLUDE_DIR}")

target_link_libraries(registration_node
   ${OpenCV_LIBS}
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
   ${YAML_CPP_LIBRARIES}
 )

target_include_directories(tracking_node PRIVATE ${YAML_CPP_INCLUDE_DIR})
message(STATUS "YAML-CPP INCLUDE DIR: ${YAML_CPP_INCLUDE_DIR}")
target_link_libraries(tracking_node
   ${OpenCV_LIBS}
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
   ${YAML_CPP_LIBRARIES}
 )

target_link_libraries(find_trajectory_node
   ${OpenCV_LIBRARIES}
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
 )

target_link_libraries(robot_controller
   ${OpenCV_LIBRARIES}
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
 )


target_include_directories(hand_eye_manual_calibration PRIVATE ${YAML_CPP_INCLUDE_DIR})
message(STATUS "YAML-CPP INCLUDE DIR: ${YAML_CPP_INCLUDE_DIR}")
target_link_libraries(hand_eye_manual_calibration
   ${OpenCV_LIBS}
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
   ${YAML_CPP_LIBRARIES}
 )

