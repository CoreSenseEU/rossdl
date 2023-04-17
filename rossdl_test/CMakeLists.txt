cmake_minimum_required(VERSION 3.8)
project(rossdl_test)

set(CMAKE_BUILD_TYPE Debug)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rossdl_cmake REQUIRED)
find_package(backward_ros REQUIRED)

set(dependencies
  rclcpp
  rclcpp_components
  std_msgs
  sensor_msgs
  backward_ros
)

include_directories(include)

rossdl_generate_code(
  "description.sdl"
  ${dependencies}
)

rossdl_generate_system(
  "description.sdl"
  "system_1"
  ${dependencies}
)

add_library(${PROJECT_NAME} SHARED
  src/rossdl_test/ImageFilter.cpp
  src/rossdl_test/Consumer.cpp
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})
target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME}_generated)

rclcpp_components_register_nodes(${PROJECT_NAME} "rossdl_test::ImageFilter")
rclcpp_components_register_nodes(${PROJECT_NAME} "rossdl_test::Consumer")

install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  add_subdirectory(tests)
endif()

ament_package()
