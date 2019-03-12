cmake_minimum_required(VERSION 3.5.0)
project(tesseract_scene_graph)

add_compile_options(
  -std=c++11
  -Wall
  -Wextra
  -Wsuggest-override
  -Wconversion
  -Wsign-conversion)

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/")

find_package(Eigen3 REQUIRED)
find_package(urdfdom REQUIRED)
find_package(urdfdom_headers REQUIRED)
find_package(console_bridge REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(TinyXML REQUIRED)

set(Boost_USE_STATIC_LIBS OFF)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)
find_package(Boost REQUIRED)


find_package(ASSIMP QUIET)
if (NOT ASSIMP_FOUND)
  find_package(PkgConfig REQUIRED)
  # assimp is required, so REQUIRE the second attempt
  pkg_check_modules(ASSIMP REQUIRED assimp)
endif()

set(ROS_BUILD_TESTING OFF)
if ($ENV{ROS_VERSION} VERSION_EQUAL "1")

  find_package(catkin REQUIRED COMPONENTS tesseract_geometry)

  set(ROS_INCLUDE_DIRS ${catkin_INCLUDE_DIRS})
  set(ROS_LIBRARIES ${catkin_LIBRARIES})
  

  catkin_package(
    INCLUDE_DIRS
      include
      ${ROS_INCLUDE_DIRS}
      ${Boost_INCLUDE_DIRS}
      ${EIGEN3_INCLUDE_DIRS}
      ${urdfdom_INCLUDE_DIRS}
      ${urdfdom_headers_INCLUDE_DIRS}
      ${console_bridge_INCLUDE_DIRS}
      ${orocos_kdl_INCLUDE_DIRS}
      ${TinyXML_INCLUDE_DIRS}
    LIBRARIES
      ${ROS_LIBRARIES}
      ${Boost_LIBRARIES}
      ${urdfdom_LIBRARIES}
      ${urdfdom_headers_LIBRARIES}
      ${console_bridge_LIBRARIES}
      ${ASSIMP_LIBRARIES}
      ${orocos_kdl_LIBRARIES}
      ${TinyXML_LIBRARIES}
    CATKIN_DEPENDS
      tesseract_geometry
    DEPENDS
      EIGEN3
      urdfdom
      urdfdom_headers
      console_bridge
      TinyXML
  #    ASSIMP
  )

  set(ROS_INCLUDE_DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

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

  find_package(ament_cmake REQUIRED)
  find_package(tesseract_geometry REQUIRED)

  set(ROS_LIBRARIES ${tesseract_geometry_LIBRARIES})
  set(ROS_INCLUDE_DIRS ${tesseract_geometry_INCLUDE_DIRS})
  set(ROS_INCLUDE_DESTINATION include)

  ament_export_include_directories(
    include
    ${ROS_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${urdfdom_INCLUDE_DIRS}
    ${urdfdom_headers_INCLUDE_DIRS}
    ${console_bridge_INCLUDE_DIRS}
    ${orocos_kdl_INCLUDE_DIRS}
    ${TinyXML_INCLUDE_DIRS})
  
  ament_export_libraries(
    ${ROS_LIBRARIES}
    ${Boost_LIBRARIES}
    ${urdfdom_LIBRARIES}
    ${urdfdom_headers_LIBRARIES}
    ${console_bridge_LIBRARIES}
    ${ASSIMP_LIBRARIES}
    ${orocos_kdl_LIBRARIES}
    ${TinyXML_LIBRARIES})
  
  ament_export_dependencies(
    tesseract_geometry
    EIGEN3
    urdfdom
    urdfdom_headers
    console_bridge
    TinyXML)

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
  SYSTEM ${Boost_INCLUDE_DIRS}
  SYSTEM ${urdfdom_INCLUDE_DIRS}
  SYSTEM ${urdfdom_headers_INCLUDE_DIRS}
  SYSTEM ${console_bridge_INCLUDE_DIRS}
  SYSTEM ${ASSIMP_INCLUDE_DIRS}
  SYSTEM ${orocos_kdl_INCLUDE_DIRS}
  SYSTEM ${TinyXML_INCLUDE_DIRS}
)

if ($ENV{ROS_VERSION} VERSION_EQUAL "1")
    set(ROS_SHARE_DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/cmake)
else()
    set(ROS_SHARE_DESTINATION share/${PROJECT_NAME})
endif()

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION ${ROS_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY cmake
  DESTINATION ${ROS_SHARE_DESTINATION})

if (ROS_BUILD_TESTING)

  add_definitions(-DDATA_DIR="${CMAKE_SOURCE_DIR}/test")

  ros_add_gtest(${PROJECT_NAME}_unit test/tesseract_scene_graph_unit.cpp)
  target_link_libraries(${PROJECT_NAME}_unit ${ROS_LIBRARIES} ${Boost_LIBRARIES} ${ASSIMP_LIBRARIES} ${urdfdom_LIBRARIES} ${urdfdom_headers_LIBRARIES} ${orocos_kdl_LIBRARIES} ${console_bridge_LIBRARIES} ${TinyXML_LIBRARIES})
  target_compile_options(${PROJECT_NAME}_unit PRIVATE -Wsuggest-override -Wconversion -Wsign-conversion)

endif()
