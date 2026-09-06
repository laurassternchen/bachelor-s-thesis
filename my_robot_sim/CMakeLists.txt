cmake_minimum_required(VERSION 3.8)

project(my_robot_sim)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    launch
    maps
    worlds
    config
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/zone_accel_limiter.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()