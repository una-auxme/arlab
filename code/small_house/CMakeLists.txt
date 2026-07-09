cmake_minimum_required(VERSION 3.8)
project(small_house)

find_package(ament_cmake REQUIRED)

install(DIRECTORY models maps worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
