cmake_minimum_required(VERSION 3.8)
project(arm_control)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CURSES_NEED_NCURSES TRUE)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(PkgConfig REQUIRED)

pkg_check_modules(TINYXML2 REQUIRED tinyxml2)

include_directories(
  include
  ${TINYXML2_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} SHARED
  src/App/arm_control.cpp
  src/ros1_compat.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  ament_index_cpp
  geometry_msgs
  kdl_parser
  rclcpp
  sensor_msgs
  std_msgs
)
target_link_libraries(${PROJECT_NAME}
  ${TINYXML2_LIBRARIES}
)

add_executable(arm_node src/arx.cpp)
ament_target_dependencies(arm_node
  ament_index_cpp
  geometry_msgs
  kdl_parser
  rclcpp
  sensor_msgs
  std_msgs
)

target_link_libraries(arm_node
  ${PROJECT_NAME}
  ${TINYXML2_LIBRARIES}
)

if(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86_64)|(AMD64)|(^i686)")
  target_link_libraries(arm_node ${CMAKE_CURRENT_SOURCE_DIR}/src/arx_lib/libx86.so)
endif()

install(TARGETS
  ${PROJECT_NAME}
  arm_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  models
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
