cmake_minimum_required(VERSION 2.8.3)
project(iiwa_hardware_interface)
add_compile_options(-std=c++11 -funroll-loops -Wall -Ofast)
set(CMAKE_BUILD_TYPE Debug)


option( ENABLE_FRI "Enable the FRI client" ON )

if( ENABLE_FRI )
  set( CMAKE_POSITION_INDEPENDENT_CODE ON)
  add_definitions( -DENABLE_FRI )
  SET( KUKA_FRI_LIBRARIES FRIClient )
  SET( KUKA_FRI_INCLUDE  /usr/local/include )
endif()

find_package(catkin REQUIRED COMPONENTS
  control_msgs
  controller_interface
  controller_manager
  diagnostic_msgs
  diagnostic_msgs
  geometry_msgs
  hardware_interface
  itia_basic_hardware_interface
  realtime_utilities
  nodelet
  roscpp
  sensor_msgs
  iiwa_msgs
  iiwa_ros
  eigen_state_space_systems
)

find_package(Eigen3 REQUIRED)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES iiwa_hardware_interface
 CATKIN_DEPENDS control_msgs controller_interface controller_manager diagnostic_msgs diagnostic_msgs geometry_msgs hardware_interface itia_basic_hardware_interface realtime_utilities nodelet roscpp sensor_msgs iiwa_msgs
 DEPENDS 
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
  ${KUKA_FRI_INCLUDE}
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}/iiwa_hw.cpp
  src/${PROJECT_NAME}/iiwa_hi_nodelet.cpp
)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
