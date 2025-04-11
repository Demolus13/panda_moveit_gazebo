cmake_minimum_required(VERSION 3.10.2)
project(multi_panda_description)
find_package(ament_cmake REQUIRED)

ament_package()

install(DIRECTORY config meshes urdf worlds DESTINATION share/${PROJECT_NAME})
