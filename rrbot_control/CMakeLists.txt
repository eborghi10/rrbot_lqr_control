cmake_minimum_required(VERSION 2.8.3)
project(rrbot_control)

add_compile_options(-std=c++11)

find_package(Eigen3 REQUIRED)
include_directories(${Eigen3_INCLUDE_DIRS})

find_package(catkin REQUIRED COMPONENTS 
  controller_manager 
  controller_interface
  hardware_interface
  roscpp

  ct_core
  ct_rbd
  ct_optcon
  ct_models
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

# get_filename_component(COSTFUNCTION_TEST_DIR "include/${PROJECT_NAME}" ABSOLUTE)
# configure_file(costfunction/costfunction_test_dir.h.in test/costfunction/costfunction_test_dir.h)

install(TARGETS lqr_controller
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(FILES controller_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
