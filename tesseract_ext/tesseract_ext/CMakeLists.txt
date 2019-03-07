cmake_minimum_required(VERSION 3.5.0)
project(tesseract_ext)

if ($ENV{ROS_VERSION} VERSION_EQUAL "1")

  find_package(catkin REQUIRED)
  catkin_metapackage()

else()

  find_package(ament_cmake REQUIRED)  

  find_package(Bullet REQUIRED)
  find_package(ccd REQUIRED)
  find_package(fcl)
  
  # pkg_check_modules(LIBFCL REQUIRED fcl)
  # find_library(LIBFCL_LIBRARIES_FULL ${LIBFCL_LIBRARIES} ${LIBFCL_LIBRARY_DIRS})
  # set(LIBFCL_LIBRARIES "${LIBFCL_LIBRARIES_FULL}")
  # list(APPEND LIBFCL_LIBRARIES ${CCD_LIBRARIES})

  ament_export_include_directories(
    ${CCD_INCLUDE_DIRS}
    ${LIBFCL_INCLUDE_DIRS}
    ${BULLET_INCLUDE_DIRS})

  ament_export_libraries(
    ${PROJECT_NAME}_bullet
    ${PROJECT_NAME}_fcl
    ${LIBFCL_LIBRARIES}
    ${LIBFCL_LIBRARY_DIRS}
    ${CCD_LIBRARIES}
    ${BULLET_LIBRARIES})

  ament_export_dependencies(
    Bullet
    LIBFCL
    ccd)
  
  ament_package()

endif()