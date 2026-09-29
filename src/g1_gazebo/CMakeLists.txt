cmake_minimum_required(VERSION 3.8)
project(g1_gazebo)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY launch config worlds models robots
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
