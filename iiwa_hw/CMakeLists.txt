cmake_minimum_required(VERSION 2.8.3)
project(iiwa_hw)

set(CMAKE_CXX_FLAGS "-std=c++14 ${CMAKE_CXX_FLAGS}")

option( ENABLE_FRI "Enable the FRI client" ON )

if( ENABLE_FRI )
  set( CMAKE_POSITION_INDEPENDENT_CODE ON)
  add_definitions( -DENABLE_FRI )
  SET( KUKA_FRI_LIBRARIES FRIClient )
  SET( KUKA_FRI_INCLUDE  /usr/local/include )
endif()

find_package(catkin REQUIRED COMPONENTS
  cnr_hardware_interface
  control_toolbox
  eigen_state_space_systems
  controller_interface
  controller_manager
  hardware_interface
  realtime_tools
  joint_limits_interface
  tf
  urdf
  cmake_modules
  iiwa_msgs
  iiwa_ros
  nodelet
)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES	 ${PROJECT_NAME}
  CATKIN_DEPENDS  controller_interface
                  hardware_interface
                  control_toolbox
                  nodelet
)

## Specify additional locations of header files
include_directories(include ${Boost_INCLUDE_DIR}
                            ${catkin_INCLUDE_DIRS}
                            ${KUKA_FRI_INCLUDE} 
                            ${EIGEN3_INCLUDE_DIR}
)

## Declare a cpp library
add_library(${PROJECT_NAME}
  src/iiwa_hw.cpp
  #src/iiwa_hi_nodelet.cpp
)

## Add dependence to the iiwa_msg module for the library
add_dependencies(${PROJECT_NAME}        iiwa_msgs_generate_messages_cpp)

## Declare a cpp executable
# add_executable(${PROJECT_NAME}-bin  src/main.cpp  )

## Add dependence to the iiwa_msg module for the executable
# add_dependencies(${PROJECT_NAME}-bin    iiwa_msgs_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(
${PROJECT_NAME} 
${catkin_LIBRARIES}
${KUKA_FRI_LIBRARIES}
)
