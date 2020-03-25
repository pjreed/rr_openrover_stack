cmake_minimum_required(VERSION 3.10)
project(rr_openrover_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY meshes
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}/
  )

install(PROGRAMS scripts/env_run
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(ament_cmake)
ament_package()