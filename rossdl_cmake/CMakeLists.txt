cmake_minimum_required(VERSION 3.5)

project(rossdl_cmake NONE)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package(
  CONFIG_EXTRAS 
    "rossdl_cmake-extras.cmake"
    "rossdl_generator_cpp-extras.cmake.in"
)


install(
  PROGRAMS bin/sdl_generator_cpp
  DESTINATION lib/sdl_generator_cpp
)

install(
  DIRECTORY cmake
  DESTINATION share/${PROJECT_NAME}
)
