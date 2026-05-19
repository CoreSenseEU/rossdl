cmake_minimum_required(VERSION 3.8)
project(application_2)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rossdl_cmake REQUIRED)


rossdl_generate_system(
  "app_2.rossystem"
  "application_2"
)

ament_package()
