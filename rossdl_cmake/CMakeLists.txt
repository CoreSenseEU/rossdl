cmake_minimum_required(VERSION 3.5)

project(rossdl_cmake NONE)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(Python3 REQUIRED COMPONENTS Interpreter Development)

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
  PROGRAMS
    bin/sdl_generator_cpp
    bin/sdl_generator_launch

    DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY cmake resources
  DESTINATION share/${PROJECT_NAME}
)
