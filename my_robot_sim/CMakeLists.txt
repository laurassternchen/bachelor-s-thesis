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
    scripts/drc/dynamic_risk_constrainer.py
    scripts/drc/pose_adapter.py
    scripts/drc/zone_detector.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()