cmake_minimum_required(VERSION 3.10.2)
project(mh2_description)

find_package(ament_cmake REQUIRED)

ament_package()

install(DIRECTORY
  launch meshes urdf rviz config
  DESTINATION share/${PROJECT_NAME}
)
