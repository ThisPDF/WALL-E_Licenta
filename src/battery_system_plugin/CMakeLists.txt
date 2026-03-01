cmake_minimum_required(VERSION 3.8)
project(battery_system_plugin)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Gazebo Sim (gz-sim)
find_package(gz-sim8 REQUIRED)

add_library(diffdrive_battery_consumer SHARED
  src/diffdrive_battery_consumer.cc
)

target_link_libraries(diffdrive_battery_consumer
  gz-sim8::gz-sim8
)

ament_target_dependencies(diffdrive_battery_consumer
  rclcpp
  std_msgs
)

# ca să ajungă ca libdiffdrive_battery_consumer.so în install/
install(TARGETS diffdrive_battery_consumer
  LIBRARY DESTINATION lib
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()