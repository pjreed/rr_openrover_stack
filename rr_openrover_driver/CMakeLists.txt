cmake_minimum_required(VERSION 3.5)
project(rr_openrover_driver)

set(CMAKE_CXX_STANDARD 14)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rr_openrover_driver_msgs REQUIRED)

include_directories(include)

###########
## Build ##
###########

add_library(odom_control src/odom_control.cpp)
ament_target_dependencies(odom_control
  rclcpp
)

add_executable(openrover_driver_node src/openrover_driver.cpp)
ament_target_dependencies(openrover_driver_node
  rclcpp
  rr_openrover_driver_msgs
  std_msgs
  geometry_msgs
  nav_msgs
  tf2_geometry_msgs
)

target_link_libraries(openrover_driver_node ${catkin_LIBRARIES} odom_control)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS odom_control
    openrover_driver_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

## Copy launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

## Copy config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY include/
   DESTINATION include
)

install(PROGRAMS scripts/diagnostics.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(ament_cmake)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}
  odom_control
)

ament_package()