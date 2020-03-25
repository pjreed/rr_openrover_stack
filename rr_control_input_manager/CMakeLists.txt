cmake_minimum_required(VERSION 3.10)
project(rr_control_input_manager)

find_package(ament_cmake REQUIRED)

#############
## Install ##
#############

install(PROGRAMS
    scripts/control_input_manager.py
    scripts/xbox_mapper.py
    test/test_functional.py
    test/test_latency.py
  DESTINATION lib/${PROJECT_NAME}
)

## Copy launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
  FILES_MATCHING PATTERN "*.launch"
)

## Copy config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
  FILES_MATCHING PATTERN "*.yaml"
)

## Copy test files
install(DIRECTORY test
  DESTINATION share/${PROJECT_NAME}/
  FILES_MATCHING PATTERN "*.test"
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_rr_control_input_manager.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(test/control_input_manager.test)
endif()

ament_export_dependencies(ament_cmake)
ament_package()
