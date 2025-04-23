cmake_minimum_required(VERSION 3.8)
project(my_package)

set(CMAKE_CXX_STANDARD 17)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(talker src/talker.cpp)
ament_target_dependencies(talker rclcpp std_msgs)

add_executable(subscriber src/subscriber.cpp)
ament_target_dependencies(subscriber rclcpp std_msgs)

add_executable(command_publisher src/command_publisher.cpp)
ament_target_dependencies(command_publisher rclcpp std_msgs geometry_msgs)

add_executable(command_publisher2 src/command_publisher2.cpp)
ament_target_dependencies(command_publisher2 rclcpp sensor_msgs geometry_msgs)

install(TARGETS
  talker
  subscriber
  command_publisher
  command_publisher2
  DESTINATION lib/${PROJECT_NAME}
)



ament_package()
