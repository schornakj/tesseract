cmake_minimum_required(VERSION 3.5.0)
project(tesseract_geometry)

add_compile_options(
  -std=c++11
  -Wall
  -Wextra
  -Wsuggest-override
  -Wconversion
  -Wsign-conversion)



find_package(Eigen3 REQUIRED)
find_package(console_bridge REQUIRED)
find_package(Octomap REQUIRED)

if ($ENV{ROS_VERSION} VERSION_EQUAL "1")
  
  message(STATUS "Building tesseract_collision for ROS1")

  find_package(catkin REQUIRED)
  

  catkin_package(
    INCLUDE_DIRS
      include
      ${catkin_INCLUDE_DIRS}
      ${EIGEN3_INCLUDE_DIRS}
      ${console_bridge_INCLUDE_DIRS}
      ${OCTOMAP_INCLUDE_DIRS}
    LIBRARIES
      ${catkin_LIBRARIES}
      ${console_bridge_LIBRARIES}
      ${OCTOMAP_LIBRARIES}

    #CATKIN_DEPENDS

    DEPENDS
      EIGEN3
      console_bridge
  )

else()
  
  message(STATUS "Building tesseract_collision for ROS2")

  find_package(ament_cmake REQUIRED)

  ament_export_include_directories(
    include
    ${EIGEN3_INCLUDE_DIRS}
    ${console_bridge_INCLUDE_DIRS}
    ${OCTOMAP_INCLUDE_DIRS}
    )

  ament_export_libraries(
    ${console_bridge_LIBRARIES}
    ${OCTOMAP_LIBRARIES})

  ament_export_dependencies(
    EIGEN3
    console_bridge
    octomap)

  ament_package()

endif()

include_directories(
  include
  SYSTEM ${EIGEN3_INCLUDE_DIRS}
  SYSTEM ${console_bridge_INCLUDE_DIRS}
  SYSTEM ${OCTOMAP_INCLUDE_DIRS}
)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
