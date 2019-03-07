cmake_minimum_required(VERSION 3.5.0)
project(bullet3_ros)

add_compile_options(-std=c++11 -w)

# This causes accuracy issues in the EPA algorithm and need to create an issue.
#set(BUILD_PYBULLET ON CACHE BOOL "Enable Bullet Python")
#set(BUILD_PYBULLET_NUMPY OFF CACHE BOOL "Disable Bullet Python Numpy")
#set(USE_DOUBLE_PRECISION ON CACHE BOOL "Enable Bullet Double Precision")

set(BUILD_UNIT_TESTS OFF CACHE BOOL "Disable Bullet Unit Tests" FORCE)

if ($ENV{ROS_VERSION} VERSION_EQUAL "1")

    file(MAKE_DIRECTORY "${CMAKE_INSTALL_PREFIX}/include/bullet")

    find_package(catkin REQUIRED COMPONENTS)

    catkin_package(
      INCLUDE_DIRS bullet3/src bullet3/Extras ${CMAKE_INSTALL_PREFIX}/include/bullet
      LIBRARIES BulletCollision BulletDynamics LinearMath ConvexDecomposition HACD
    )

else()

    find_package(ament_cmake REQUIRED)

    ament_export_include_directories(bullet3/src bullet3/Extras)

    ament_export_libraries(BulletCollision BulletDynamics LinearMath ConvexDecomposition HACD)

    ament_package()
endif()

add_subdirectory(bullet3)
