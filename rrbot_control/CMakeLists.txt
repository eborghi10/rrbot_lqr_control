cmake_minimum_required(VERSION 2.8.3)
project(rrbot_control)

add_compile_options(-std=c++11)

# Eigen3
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR})
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(catkin REQUIRED COMPONENTS 
  controller_manager 
  controller_interface
  hardware_interface
  roscpp
)

include_directories(include ${catkin_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    controller_manager 
    controller_interface
    hardware_interface
    roscpp
    pluginlib
  DEPENDS Eigen3
  LIBRARIES ${PROJECT_NAME}
)

add_library(lqr_controller src/lqr_controller.cpp)
target_link_libraries(lqr_controller ${catkin_LIBRARIES})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS lqr_controller
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(FILES controller_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
