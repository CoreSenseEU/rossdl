cmake_minimum_required(VERSION 3.8)
project(rossdl_test)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rossdl_cmake REQUIRED)

set(dependencies
  rclcpp
  std_msgs
  sensor_msgs
)

include_directories(include)

rossdl_generate_code(${PROJECT_NAME}
  "description.sdl"
  ${dependencies}
)

add_library(${PROJECT_NAME} SHARED
  src/rossdl_test/Nodes2.cpp
  src/rossdl_test/ImageFilter.cpp
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

add_executable(rossdl_test_main
  src/rossdl_test_main.cpp
)
ament_target_dependencies(rossdl_test_main ${dependencies})
target_link_libraries(rossdl_test_main ${PROJECT_NAME})

install(TARGETS
  ${PROJECT_NAME}
  rossdl_test_main
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
