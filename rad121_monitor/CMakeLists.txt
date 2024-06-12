cmake_minimum_required(VERSION 3.5)
project(rad121_monitor)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBFTDI REQUIRED libftdi1)

include_directories(include ${LIBFTDI_INCLUDE_DIRS})

add_executable(rad121_monitor_node src/rad121_monitor_node.cpp src/CUSB_RAD121.cpp)

target_link_libraries(rad121_monitor_node ${LIBFTDI_LIBRARIES})

ament_target_dependencies(rad121_monitor_node rclcpp std_msgs)

install(TARGETS
  rad121_monitor_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include/)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()

