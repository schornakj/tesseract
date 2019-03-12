cmake_minimum_required(VERSION 3.5.0)
project(tesseract_kinematics)

## Compile as C++11
add_compile_options(
  -std=c++11
  -Wall
  -Wextra
  -Wsuggest-override
  -Wconversion
  -Wsign-conversion)


## System dependencies are found with CMake's conventions
find_package(Eigen3 REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(console_bridge REQUIRED)

set(ROS_BUILD_TESTING OFF)
if ($ENV{ROS_VERSION} VERSION_EQUAL "1")

  message(STATUS "Building tesseract_collision for ROS1")

  find_package(catkin REQUIRED COMPONENTS tesseract_scene_graph)
  
  set(ROS_INCLUDE_DIRS ${catkin_INCLUDE_DIRS})
  set(ROS_LIBRARIES ${catkin_LIBRARIES})
  
  catkin_package(
    INCLUDE_DIRS
      include
      ${ROS_INCLUDE_DIRS}
      ${EIGEN3_INCLUDE_DIRS}
      ${orocos_kdl_INCLUDE_DIRS}
      ${console_bridge_INCLUDE_DIRS}
    LIBRARIES
      ${PROJECT_NAME}_kdl
      ${ROS_LIBRARIES}
      ${orocos_kdl_LIBRARIES}
      ${console_bridge_LIBRARIES}
    CATKIN_DEPENDS
      tesseract_scene_graph
    DEPENDS
      EIGEN3
      orocos_kdl
      console_bridge
  )

  if (CATKIN_ENABLE_TESTING)

    set(ROS_BUILD_TESTING ON)   

    find_package(roslib REQUIRED)
    list(APPEND ROS_INCLUDE_DIRS ${roslib_INCLUDE_DIRS})
    list(APPEND ROS_LIBRARIES ${roslib_LIBRARIES})

    macro(ros_add_gtest)
      catkin_add_gtest(${ARGN})
    endmacro()
  
  endif()

else()

  message(STATUS "Building tesseract_collision for ROS2")

  list(APPEND CMAKE_MODULE_PATH ${tesseract_scene_graph_DIR})
  
  find_package(ament_cmake REQUIRED)
  find_package(tesseract_scene_graph)

  set(ROS_LIBRARIES ${tesseract_scene_graph_LIBRARIES})
  set(ROS_INCLUDE_DIRS ${tesseract_scene_graph_INCLUDE_DIRS})
  
  ament_export_include_directories(
    include
    ${ROS_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${orocos_kdl_INCLUDE_DIRS}
    ${console_bridge_INCLUDE_DIRS})
  
  ament_export_libraries(
    ${PROJECT_NAME}_kdl
    ${ROS_LIBRARIES}
    ${orocos_kdl_LIBRARIES}
    ${console_bridge_LIBRARIES})
  
  ament_export_dependencies(
    tesseract_scene_graph
    EIGEN3
    orocos_kdl
    console_bridge)
  
  if (BUILD_TESTING)

    add_definitions(-DROS2)

    set(ROS_BUILD_TESTING ON)

    find_package(ament_index_cpp REQUIRED)
    find_package(ament_cmake_gtest REQUIRED)
    find_package(ament_lint_auto REQUIRED)

    list(APPEND ROS_INCLUDE_DIRS ${ament_index_cpp_INCLUDE_DIRS})
    list(APPEND ROS_LIBRARIES ${ament_index_cpp_LIBRARIES})

    macro(ros_add_gtest)
      ament_add_gtest(${ARGN})
    endmacro()

  endif()

  ament_package()

endif()

include_directories(
  include
  ${ROS_INCLUDE_DIRS}
  SYSTEM ${EIGEN3_INCLUDE_DIRS}
  SYSTEM ${orocos_kdl_INCLUDE_DIRS}
  SYSTEM ${console_bridge_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_kdl
  src/kdl/kdl_fwd_kin_chain.cpp
  src/kdl/kdl_fwd_kin_tree.cpp
)
target_link_libraries(${PROJECT_NAME}_kdl ${ROS_LIBRARIES} ${orocos_kdl_LIBRARIES} ${console_bridge_LIBRARIES})
target_compile_options(${PROJECT_NAME}_kdl PRIVATE -Wsuggest-override -Wconversion -Wsign-conversion)

if ($ENV{ROS_VERSION} VERSION_EQUAL "1")
    set(ROS_LIB_DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
    set(ROS_BIN_DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
    set(ROS_INCLUDE_DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
else()
    set(ROS_LIB_DESTINATION lib)
    set(ROS_BIN_DESTINATION bin)
    set(ROS_INCLUDE_DESTINATION include)
endif()

# Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}_kdl
  ARCHIVE DESTINATION ${ROS_LIB_DESTINATION}
  LIBRARY DESTINATION ${ROS_LIB_DESTINATION}
  RUNTIME DESTINATION ${ROS_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION ${ROS_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

if (ROS_BUILD_TESTING)

  add_definitions(-DDATA_DIR="${CMAKE_SOURCE_DIR}/test")

  ros_add_gtest(${PROJECT_NAME}_kdl_unit test/kdl_kinematics_unit.cpp)
  target_link_libraries(${PROJECT_NAME}_kdl_unit ${PROJECT_NAME}_kdl ${ROS_LIBRARIES} ${orocos_kdl_LIBRARIES} ${console_bridge_LIBRARIES})
  target_compile_options(${PROJECT_NAME}_kdl_unit PRIVATE -Wsuggest-override -Wconversion -Wsign-conversion)

endif()