cmake_minimum_required(VERSION 3.8)
project(sonoptix_echo_driver_ros)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/sonoptix_echo_driver.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
