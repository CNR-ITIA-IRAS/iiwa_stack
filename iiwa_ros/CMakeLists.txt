cmake_minimum_required(VERSION 2.8.3)
project(iiwa_ros)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
set(CMAKE_BUILD_TYPE Debug)

option( ENABLE_FRI "Enable the FRI client" ON )

if( ENABLE_FRI )
  set( CMAKE_POSITION_INDEPENDENT_CODE ON)
  add_definitions( -DENABLE_FRI )
  SET( KUKA_FRI_LIBRARIES FRIClient )
  SET( KUKA_FRI_INCLUDE  /usr/local/include )
endif()

find_package(catkin REQUIRED COMPONENTS
  iiwa_msgs
  roscpp
  realtime_utilities
  realtime_tools
  kdl_parser
  eigen_conversions
  kdl_conversions
  eigen_state_space_systems
)

find_package(Boost REQUIRED system filesystem date_time thread )
find_package(Eigen3)

find_package( orocos_kdl )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES	 ${PROJECT_NAME}
  CATKIN_DEPENDS iiwa_msgs realtime_utilities realtime_tools roscpp
  DEPENDS ${KUKA_FRI_LIBRARIES}
  
)

## Specify additional locations of header files
include_directories(include ${catkin_INCLUDE_DIRS} ${KUKA_FRI_INCLUDE} ${EIGEN3_INCLUDE_DIR} )

## Declare a cpp library
add_library(${PROJECT_NAME}
  src/iiwa_ros.cpp
  src/servo_motion_service.cpp
  src/path_parameters_service.cpp
  src/time_to_destination_service.cpp
  src/fri_client.cpp
)

set_target_properties(${PROJECT_NAME} PROPERTIES POSITION_INDEPENDENT_CODE ON)

## Add dependence to the iiwa_msg module for the library
add_dependencies(${PROJECT_NAME} iiwa_msgs_generate_messages_cpp)


add_executable(test_fri src/test_fri.cpp)
target_link_libraries(test_fri ${PROJECT_NAME} orocos-kdl ${KUKA_FRI_LIBRARIES} ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES} ${Boost_LIBRARIES})
