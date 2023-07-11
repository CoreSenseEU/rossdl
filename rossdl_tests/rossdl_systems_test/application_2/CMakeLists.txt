cmake_minimum_required(VERSION 3.8)
project(applications_1)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rossdl_cmake REQUIRED)

set(dependencies
  system_1
  system_2
)

rossdl_generate_system(
  "app_1.rossystem"
  "app_1_system_1"
)

rossdl_generate_system(
  "app_1.rossystem"
  "app_1_system_2"
)

rossdl_generate_system(
  "app_1.rossystem"
  "app_1_system_3"
)

# rossdl_generate_system(
#   "app_1.ros2"
#   "app_1_system_2"
#   ${dependencies}
# )
# 
# rossdl_generate_system(
#   "app_1.ros2"
#   "app_1_system_3"
#   ${dependencies}
# )

ament_package()
