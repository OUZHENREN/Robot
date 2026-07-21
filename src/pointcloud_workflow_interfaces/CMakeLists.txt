cmake_minimum_required(VERSION 3.8)
project(pointcloud_workflow_interfaces)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/StartSampling.srv"
  "srv/StopSampling.srv"
  "srv/PreviewSamplingPath.srv"
  "srv/GetSamplingStatus.srv"
  "srv/ListAvailablePcds.srv"
  "srv/SetEnabledPcds.srv"
  "srv/RebuildMergedCloud.srv"
  "srv/GetPublisherStatus.srv"
  DEPENDENCIES geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
