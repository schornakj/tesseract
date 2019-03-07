cmake_minimum_required(VERSION 3.5.0)
project(tesseract_collision)

add_compile_options(
  -std=c++11
  -Wall
  -Wextra
  -Wsuggest-override
  -Wconversion
  -Wsign-conversion)

find_package(Eigen3 REQUIRED)
find_package(Boost COMPONENTS system thread program_options REQUIRED)
find_package(octomap REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(ccd REQUIRED)
find_package(console_bridge REQUIRED)

pkg_check_modules(LIBFCL REQUIRED fcl)
find_library(LIBFCL_LIBRARIES_FULL ${LIBFCL_LIBRARIES} ${LIBFCL_LIBRARY_DIRS})
set(LIBFCL_LIBRARIES "${LIBFCL_LIBRARIES_FULL}")
list(APPEND LIBFCL_LIBRARIES ${CCD_LIBRARIES})

set(ROS_BUILD_TESTING OFF)
if ($ENV{ROS_VERSION} VERSION_EQUAL "1")

  message(STATUS "Building tesseract_collision for ROS1")

  find_package(catkin REQUIRED COMPONENTS tesseract_geometry bullet3_ros)

  set(ROS_LIBRARIES ${catkin_LIBRARIES})
  set(ROS_INCLUDE_DIRS ${catkin_INCLUDE_DIRS})

  catkin_package(
    INCLUDE_DIRS
      include
      ${CCD_INCLUDE_DIRS}
      ${LIBFCL_INCLUDE_DIRS}
      ${EIGEN3_INCLUDE_DIRS}
      ${ROS_INCLUDE_DIRS}
      ${console_bridge_INCLUDE_DIRS}
    LIBRARIES
      ${PROJECT_NAME}_bullet
      ${PROJECT_NAME}_fcl
      ${OCTOMAP_LIBRARIES}
      ${LIBFCL_LIBRARIES}
      ${ROS_LIBRARIES}
      ${console_bridge_LIBRARIES}
    CATKIN_DEPENDS
      bullet3_ros
      tesseract_geometry
    DEPENDS
      EIGEN3
      Boost
      OCTOMAP
      LIBFCL
      ccd
      console_bridge
  )

  if (CATKIN_ENABLE_TESTING)

    set(ROS_BUILD_TESTING ON)

    find_package(rostest REQUIRED)

    macro(ros_add_gtest)
      catkin_add_gtest(${ARGN})
    endmacro()

  endif()

else()

  message(STATUS "Building tesseract_collision for ROS2")

  find_package(ament_cmake REQUIRED)
  find_package(Bullet REQUIRED)
  find_package(tesseract_geometry REQUIRED)

  add_definitions(${BULLET_DEFINITIONS})

  set(ROS_LIBRARIES ${BULLET_LIBRARIES})
  set(ROS_INCLUDE_DIRS ${BULLET_INCLUDE_DIRS})

  list(APPEND ROS_LIBRARIES ${tesseract_geometry_LIBRARIES})
  list(APPEND ROS_INCLUDE_DIRS ${tesseract_geometry_INCLUDE_DIRS})
  
  ament_export_include_directories(
    include
    ${CCD_INCLUDE_DIRS}
    ${LIBFCL_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${ROS_INCLUDE_DIRS}
    ${console_bridge_INCLUDE_DIRS})

  ament_export_libraries(
    ${PROJECT_NAME}_bullet
    ${PROJECT_NAME}_fcl
    ${OCTOMAP_LIBRARIES}
    ${LIBFCL_LIBRARIES}
    ${ROS_LIBRARIES}
    ${console_bridge_LIBRARIES})

  ament_export_dependencies(
    Bullet
    tesseract_geometry
    EIGEN3
    Boost
    OCTOMAP
    LIBFCL
    ccd
    console_bridge)

  if (AMENT_ENABLE_TESTING)

    set(ROS_BUILD_TESTING ON)

    find_package(ament_cmake_gtest REQUIRED)
    find_package(ament_lint_auto REQUIRED)

    macro(ros_add_gtest)
      ament_add_gtest(${ARGN})
    endmacro()

  endif()

  ament_package()

endif()

include_directories(
  include
  ${ROS_INCLUDE_DIRS}
  ${LIBFCL_INCLUDE_DIRS}
  SYSTEM ${EIGEN3_INCLUDE_DIRS}
  SYSTEM ${Boost_INCLUDE_DIRS}
  SYSTEM ${OCTOMAP_INCLUDE_DIRS}
  SYSTEM ${console_bridge_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_bullet
  src/bullet/bullet_cast_bvh_manager.cpp
  src/bullet/bullet_cast_simple_manager.cpp
  src/bullet/bullet_discrete_bvh_manager.cpp
  src/bullet/bullet_discrete_simple_manager.cpp
  src/bullet/bullet_utils.cpp
)
target_link_libraries(${PROJECT_NAME}_bullet ${ROS_LIBRARIES} ${Boost_LIBRARIES} ${OCTOMAP_LIBRARIES} ${console_bridge_LIBRARIES})
target_compile_options(${PROJECT_NAME}_bullet PRIVATE -Wsuggest-override -Wconversion -Wsign-conversion)

add_library(${PROJECT_NAME}_fcl src/fcl/fcl_discrete_managers.cpp src/fcl/fcl_utils.cpp)
target_link_libraries(${PROJECT_NAME}_fcl ${ROS_LIBRARIES} ${Boost_LIBRARIES} ${OCTOMAP_LIBRARIES} ${LIBFCL_LIBRARIES} ${console_bridge_LIBRARIES})
target_compile_options(${PROJECT_NAME}_fcl PRIVATE -Wsuggest-override -Wconversion -Wsign-conversion)

add_executable(create_convex_hull src/create_convex_hull.cpp)
target_link_libraries(create_convex_hull ${ROS_LIBRARIES} ${Boost_LIBRARIES} ${OCTOMAP_LIBRARIES} ${console_bridge_LIBRARIES})
target_compile_options(create_convex_hull PRIVATE -Wsuggest-override -Wconversion -Wsign-conversion)

if ($ENV{ROS_VERSION} VERSION_EQUAL "1")
    set(ROS_LIB_DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
    set(ROS_BIN_DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
    set(ROS_INCLUDE_DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
else()
    set(ROS_LIB_DESTINATION lib})
    set(ROS_BIN_DESTINATION bin)
    set(ROS_INCLUDE_DESTINATION include)
endif()

# Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}_bullet ${PROJECT_NAME}_fcl #create_convex_hull
  ARCHIVE DESTINATION ${ROS_LIB_DESTINATION}
  LIBRARY DESTINATION ${ROS_LIB_DESTINATION}
  RUNTIME DESTINATION ${ROS_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${ROS_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

if (ROS_BUILD_TESTING)

  # find_package(rostest REQUIRED)

  find_package(OpenMP REQUIRED)
  if(OPENMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  else(OPENMP_FOUND)
    message (STATUS "Not found OpenMP")
  endif()

  add_definitions(-DDATA_DIR="${CMAKE_SOURCE_DIR}/test")

  include_directories(${ROS_INCLUDE_DIRS})

  ros_add_gtest(${PROJECT_NAME}_box_sphere_unit test/collision_box_sphere_unit.cpp)
  target_link_libraries(${PROJECT_NAME}_box_sphere_unit ${PROJECT_NAME}_bullet ${PROJECT_NAME}_fcl ${ROS_LIBRARIES} ${Boost_LIBRARIES} ${OCTOMAP_LIBRARIES} ${LIBFCL_INCLUDE_DIRS} ${console_bridge_LIBRARIES})
  target_compile_options(${PROJECT_NAME}_box_sphere_unit PRIVATE -Wsuggest-override -Wconversion -Wsign-conversion)

  ros_add_gtest(${PROJECT_NAME}_box_cylinder_unit test/collision_box_cylinder_unit.cpp)
  target_link_libraries(${PROJECT_NAME}_box_cylinder_unit ${PROJECT_NAME}_bullet ${PROJECT_NAME}_fcl ${ROS_LIBRARIES} ${Boost_LIBRARIES} ${OCTOMAP_LIBRARIES} ${LIBFCL_INCLUDE_DIRS} ${console_bridge_LIBRARIES})
  target_compile_options(${PROJECT_NAME}_box_cylinder_unit PRIVATE -Wsuggest-override -Wconversion -Wsign-conversion)

  ros_add_gtest(${PROJECT_NAME}_box_cone_unit test/collision_box_cone_unit.cpp)
  target_link_libraries(${PROJECT_NAME}_box_cone_unit ${PROJECT_NAME}_bullet ${PROJECT_NAME}_fcl ${ROS_LIBRARIES} ${Boost_LIBRARIES} ${OCTOMAP_LIBRARIES} ${LIBFCL_INCLUDE_DIRS} ${console_bridge_LIBRARIES})
  target_compile_options(${PROJECT_NAME}_box_cone_unit PRIVATE -Wsuggest-override -Wconversion -Wsign-conversion)

  ros_add_gtest(${PROJECT_NAME}_box_box_unit test/collision_box_box_unit.cpp)
  target_link_libraries(${PROJECT_NAME}_box_box_unit ${PROJECT_NAME}_bullet ${PROJECT_NAME}_fcl ${ROS_LIBRARIES} ${Boost_LIBRARIES} ${OCTOMAP_LIBRARIES} ${LIBFCL_INCLUDE_DIRS} ${console_bridge_LIBRARIES})
  target_compile_options(${PROJECT_NAME}_box_box_unit PRIVATE -Wsuggest-override -Wconversion -Wsign-conversion)

  ros_add_gtest(${PROJECT_NAME}_large_dataset_unit test/collision_large_dataset_unit.cpp)
  target_link_libraries(${PROJECT_NAME}_large_dataset_unit ${PROJECT_NAME}_bullet ${PROJECT_NAME}_fcl ${ROS_LIBRARIES} ${Boost_LIBRARIES} ${OCTOMAP_LIBRARIES} ${LIBFCL_INCLUDE_DIRS} ${console_bridge_LIBRARIES})
  target_compile_options(${PROJECT_NAME}_large_dataset_unit PRIVATE -Wsuggest-override -Wconversion -Wsign-conversion)

  ros_add_gtest(${PROJECT_NAME}_sphere_sphere_unit test/collision_sphere_sphere_unit.cpp)
  target_link_libraries(${PROJECT_NAME}_sphere_sphere_unit ${PROJECT_NAME}_bullet ${PROJECT_NAME}_fcl ${ROS_LIBRARIES} ${Boost_LIBRARIES} ${OCTOMAP_LIBRARIES} ${LIBFCL_INCLUDE_DIRS} ${console_bridge_LIBRARIES})
  target_compile_options(${PROJECT_NAME}_sphere_sphere_unit PRIVATE -Wsuggest-override -Wconversion -Wsign-conversion)

  ros_add_gtest(${PROJECT_NAME}_mesh_mesh_unit test/collision_mesh_mesh_unit.cpp)
  target_link_libraries(${PROJECT_NAME}_mesh_mesh_unit ${PROJECT_NAME}_bullet ${PROJECT_NAME}_fcl ${ROS_LIBRARIES} ${Boost_LIBRARIES} ${OCTOMAP_LIBRARIES} ${LIBFCL_INCLUDE_DIRS} ${console_bridge_LIBRARIES})
  target_compile_options(${PROJECT_NAME}_mesh_mesh_unit PRIVATE -Wsuggest-override -Wconversion -Wsign-conversion)

  ros_add_gtest(${PROJECT_NAME}_multi_threaded_unit test/collision_multi_threaded_unit.cpp)
  target_link_libraries(${PROJECT_NAME}_multi_threaded_unit ${PROJECT_NAME}_bullet ${PROJECT_NAME}_fcl ${ROS_LIBRARIES} ${Boost_LIBRARIES} ${OCTOMAP_LIBRARIES} ${LIBFCL_INCLUDE_DIRS} ${console_bridge_LIBRARIES})
  target_compile_options(${PROJECT_NAME}_multi_threaded_unit PRIVATE -Wsuggest-override -Wconversion -Wsign-conversion)

  ros_add_gtest(${PROJECT_NAME}_octomap_sphere_unit test/collision_octomap_sphere_unit.cpp)
  target_link_libraries(${PROJECT_NAME}_octomap_sphere_unit ${PROJECT_NAME}_bullet ${PROJECT_NAME}_fcl ${ROS_LIBRARIES} ${Boost_LIBRARIES} ${OCTOMAP_LIBRARIES} ${LIBFCL_INCLUDE_DIRS} ${console_bridge_LIBRARIES})
  target_compile_options(${PROJECT_NAME}_octomap_sphere_unit PRIVATE -Wsuggest-override -Wconversion -Wsign-conversion)

  ros_add_gtest(${PROJECT_NAME}_clone_unit test/collision_clone_unit.cpp)
  target_link_libraries(${PROJECT_NAME}_clone_unit ${PROJECT_NAME}_bullet ${PROJECT_NAME}_fcl ${ROS_LIBRARIES} ${Boost_LIBRARIES} ${OCTOMAP_LIBRARIES} ${LIBFCL_INCLUDE_DIRS} ${console_bridge_LIBRARIES})
  target_compile_options(${PROJECT_NAME}_clone_unit PRIVATE -Wsuggest-override -Wconversion -Wsign-conversion)

  ros_add_gtest(${PROJECT_NAME}_box_box_cast_unit test/collision_box_box_cast_unit.cpp)
  target_link_libraries(${PROJECT_NAME}_box_box_cast_unit ${PROJECT_NAME}_bullet ${PROJECT_NAME}_fcl ${ROS_LIBRARIES} ${Boost_LIBRARIES} ${OCTOMAP_LIBRARIES} ${LIBFCL_INCLUDE_DIRS} ${console_bridge_LIBRARIES})
  target_compile_options(${PROJECT_NAME}_box_box_cast_unit PRIVATE -Wsuggest-override -Wconversion -Wsign-conversion)

  ros_add_gtest(${PROJECT_NAME}_compound_compound_unit test/collision_compound_compound_unit.cpp)
  target_link_libraries(${PROJECT_NAME}_compound_compound_unit ${PROJECT_NAME}_bullet ${PROJECT_NAME}_fcl ${ROS_LIBRARIES} ${Boost_LIBRARIES} ${OCTOMAP_LIBRARIES} ${LIBFCL_INCLUDE_DIRS} ${console_bridge_LIBRARIES})
  target_compile_options(${PROJECT_NAME}_compound_compound_unit PRIVATE -Wsuggest-override -Wconversion -Wsign-conversion)

#  ros_add_gtest(${PROJECT_NAME}_convex_concave_unit test/convex_concave_unit.cpp)
#  target_link_libraries(${PROJECT_NAME}_convex_concave_unit ${PROJECT_NAME}_bullet ${PROJECT_NAME}_fcl ${ROS_LIBRARIES} ${Boost_LIBRARIES} ${OCTOMAP_LIBRARIES} ${LIBFCL_INCLUDE_DIRS} ${console_bridge_LIBRARIES})
endif()
