cmake_minimum_required(VERSION 3.5.0)
project(tesseract_visualization)

add_compile_options(-std=c++11 -Wall -Wextra)



find_package(Eigen3 REQUIRED)

if ($ENV{ROS_VERSION} VERSION_EQUAL "1")

  find_package(catkin REQUIRED COMPONENTS tesseract_collision)

  set(ROS_INCLUDE_DIRS ${catkin_INCLUDE_DIRS})

  catkin_package(
    INCLUDE_DIRS
      include
      ${ROS_INCLUDE_DIRS}
      ${EIGEN3_INCLUDE_DIRS}
  #  LIBRARIES
    CATKIN_DEPENDS
      tesseract_collision
    DEPENDS
      EIGEN3
  )

else()

  find_package(ament_cmake REQUIRED)
  find_package(tesseract_collision REQUIRED)

  set(ROS_INCLUDE_DIRS ${tesseract_collision_INCLUDE_DIRS})

  ament_export_include_directories(
    include
    ${ROS_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS})
  
  ament_export_dependencies(
    tesseract_collision
    EIGEN3)

  ament_package()
  
endif()

include_directories(
  include
  ${ROS_INCLUDE_DIRS}
  SYSTEM ${EIGEN3_INCLUDE_DIRS}
)

if ($ENV{ROS_VERSION} VERSION_EQUAL "1")
    set(ROS_LIB_DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
    set(ROS_BIN_DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
    set(ROS_INCLUDE_DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
else()
    set(ROS_LIB_DESTINATION lib)
    set(ROS_BIN_DESTINATION bin)
    set(ROS_INCLUDE_DESTINATION include)
endif()

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION ${ROS_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
