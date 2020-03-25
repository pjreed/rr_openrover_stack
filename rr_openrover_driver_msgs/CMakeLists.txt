cmake_minimum_required(VERSION 3.10)
project(rr_openrover_driver_msgs)

set(CMAKE_CXX_STANDARD 14)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/RawRrOpenroverDriverFastRateData.msg
  msg/RawRrOpenroverDriverMedRateData.msg
  msg/RawRrOpenroverDriverSlowRateData.msg
  msg/SmartBatteryStatus.msg
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
