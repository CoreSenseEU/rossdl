cmake_minimum_required(VERSION 3.8)
project(rossdl_test)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rossdl_cmake REQUIRED)


rossdl_generate_code(${PROJECT_NAME}
  "description.sdl"
)

ament_package()
