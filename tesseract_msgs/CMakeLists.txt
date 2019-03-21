cmake_minimum_required(VERSION 3.5.0)
project(tesseract_msgs)

add_compile_options(
  -std=c++11
  -Wall
  -Wextra
  -Wsuggest-override
  -Wconversion
  -Wsign-conversion)

if ($ENV{ROS_VERSION} VERSION_EQUAL "1")

  find_package(catkin REQUIRED COMPONENTS
    message_generation
    std_msgs
    sensor_msgs
    geometry_msgs
    shape_msgs
    octomap_msgs
    trajectory_msgs
  )

  add_message_files(
    DIRECTORY msg
    FILES
      AllowedCollisionEntry.msg
      AttachableObject.msg
      AttachedBodyInfo.msg
      VisualGeometry.msg
      CollisionGeometry.msg
      TesseractState.msg
      ObjectColor.msg
      Trajectory.msg
      ContactResult.msg
      ContactResultVector.msg
      Mesh.msg
    )

  add_service_files(
    DIRECTORY srv
    FILES
      ModifyTesseractEnv.srv
      ComputeContactResultVector.srv
  )

  generate_messages(
    DEPENDENCIES
      std_msgs
      sensor_msgs
      geometry_msgs
      shape_msgs
      octomap_msgs
      trajectory_msgs
  )

else()

  find_package(ament_cmake REQUIRED)
  find_package(builtin_interfaces REQUIRED)
  find_package(rosidl_default_generators REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(shape_msgs REQUIRED)
  find_package(trajectory_msgs REQUIRED)
  find_package(octomap_msgs REQUIRED)
  

  rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/AllowedCollisionEntry.msg"
    "msg/AttachableObject.msg"
    "msg/AttachedBodyInfo.msg"
    "msg/VisualGeometry.msg"
    "msg/CollisionGeometry.msg"
    "msg/TesseractState.msg"
    "msg/ObjectColor.msg"
    "msg/Trajectory.msg"
    "msg/ContactResult.msg"
    "msg/ContactResultVector.msg"
    "msg/Mesh.msg"
    "srv/ModifyTesseractEnv.srv"
    "srv/ComputeContactResultVector.srv"
    DEPENDENCIES 
      builtin_interfaces
      std_msgs
      sensor_msgs
      geometry_msgs
      shape_msgs
      trajectory_msgs
      octomap_msgs
  )

  ament_export_dependencies(rosidl_default_runtime)

  ament_package()

endif()