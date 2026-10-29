cmake_minimum_required(VERSION 3.10)
project(sonoptix_echo_driver_ros LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc videoio)
find_package(nlohmann_json REQUIRED)

find_package(httplib CONFIG QUIET)
if(TARGET httplib::httplib)
  set(HTTPLIB_TARGET httplib::httplib)
else()
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(HTTPLIB REQUIRED IMPORTED_TARGET cpp-httplib)
  set(HTTPLIB_TARGET PkgConfig::HTTPLIB)
endif()

generate_parameter_library(sonoptix_echo_driver_parameters
  src/sonoptix_echo_driver_parameters.yaml
)

# The driver node, as a library that is also a loadable component
add_library(sonoptix_echo_driver_component SHARED
  src/sonoptix_echo_driver.cpp
  src/sonar_image_processing.cpp
  src/cone_projector.cpp
)

target_include_directories(sonoptix_echo_driver_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_link_libraries(sonoptix_echo_driver_component
  PUBLIC
    sonoptix_echo_driver_parameters
    rclcpp::rclcpp
    rclcpp_lifecycle::rclcpp_lifecycle
    ${std_srvs_TARGETS}
    ${sensor_msgs_TARGETS}
    ${OpenCV_LIBS}
  PRIVATE
    rclcpp_components::component
    ${lifecycle_msgs_TARGETS}
    ${std_msgs_TARGETS}
    cv_bridge::cv_bridge
    nlohmann_json::nlohmann_json
    ${HTTPLIB_TARGET}
)

rclcpp_components_register_nodes(sonoptix_echo_driver_component
  "sonoptix_echo_driver_ros::SonoptixEchoDriver"
)

add_executable(sonoptix_echo_driver src/main.cpp)
target_link_libraries(sonoptix_echo_driver PRIVATE sonoptix_echo_driver_component)

install(TARGETS sonoptix_echo_driver_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS sonoptix_echo_driver
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
