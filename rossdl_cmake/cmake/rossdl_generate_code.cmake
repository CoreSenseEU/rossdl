# Copyright 2014-2018 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


macro(rossdl_generate_code description_file)
  get_filename_component(_extension ${description_file} LAST_EXT)

  if("${_extension}" STREQUAL "")
    message(FATAL_ERROR "rossdl_generate_code() called without any "
      "description file")
  endif()

  if(_${PROJECT_NAME}_AMENT_PACKAGE)
    message(FATAL_ERROR
      "rossdl_generate_code() must be called before ament_package()")
  endif()

  _rossdl_cmake_register_package_hook()
  if(NOT _ARG_SKIP_INSTALL)
    ament_export_dependencies(${_ARG_DEPENDENCIES})
  endif()

  if(IS_ABSOLUTE "${description_file}")
    string(FIND "${description_file}" ":" _index)
    if(_index EQUAL -1)
      message(FATAL_ERROR "rossdl_generate_code() the passed absolute "
        "file '${description_file}' must be represented as an absolute base path "
        "separated by a colon from the relative path to the interface file")
    endif()
    string(REGEX REPLACE ":([^:]*)$" "/\\1" _abs_file "${description_file}")
    if(NOT EXISTS "${_abs_file}")
      message(FATAL_ERROR "rossdl_generate_code() the passed file "
        "'${_abs_file}' doesn't exist")
    endif()
    set(_code_tuple "${description_file}")
  else()
    set(_abs_file "${CMAKE_CURRENT_SOURCE_DIR}/${description_file}")
    if(NOT EXISTS "${_abs_file}")
      message(FATAL_ERROR "rossdl_generate_code() the passed file "
          "'${description_file}' doesn't exist relative to the "
          "CMAKE_CURRENT_SOURCE_DIR '${CMAKE_CURRENT_SOURCE_DIR}'")
    endif()
    set(_code_tuple "${CMAKE_CURRENT_SOURCE_DIR}:${description_file}")
  endif()

  ament_index_get_prefix_path(PACKAGES_INSTALL_PATHS)
  set(ROSSDL_CMAKE_PATH "")
  foreach(PACKAGES_INSTALL_PATH ${PACKAGES_INSTALL_PATHS})
    if(PACKAGES_INSTALL_PATH MATCHES "rossdl_cmake")
      set(ROSSDL_CMAKE_PATH ${PACKAGES_INSTALL_PATH})
    endif()
  endforeach()

  file(READ ${_abs_file} ARTIFACTS_DESCRIPTION)
  ament_index_register_resource(rossdl_artifact_descriptions CONTENT ${ARTIFACTS_DESCRIPTION})

  set(RESOURCE_CPP ${ROSSDL_CMAKE_PATH}/share/rossdl_cmake/resources/nodes.cpp.em)
  set(RESOURCE_HPP ${ROSSDL_CMAKE_PATH}/share/rossdl_cmake/resources/nodes.hpp.em)

  string(REGEX REPLACE ":([^:]*)$" "/\\1" _abs_file "${_code_tuple}")
  set(_header_out_file  ${CMAKE_CURRENT_BINARY_DIR}/include/${PROJECT_NAME}/Nodes.hpp)
  set(_source_out_file  ${CMAKE_CURRENT_BINARY_DIR}/src/${PROJECT_NAME}/Nodes.cpp)

  add_custom_command(
    OUTPUT ${_source_out_file} ${_header_out_file}
    COMMAND ros2
      ARGS run rossdl_cmake sdl_generator_cpp
        --package ${PROJECT_NAME}
        --description-file ${_abs_file}
        --header-out-file ${_header_out_file}
        --source-out-file ${_source_out_file}
    DEPENDS ${_abs_file} ${RESOURCE_CPP} ${RESOURCE_HPP}
    COMMENT "Generating code for ROS 2 System"
    VERBATIM
  )
  include_directories(${CMAKE_CURRENT_BINARY_DIR}/include)
  add_library(${PROJECT_NAME}_generated SHARED
    ${_source_out_file} ${_header_out_file}
  )
  ament_target_dependencies(${PROJECT_NAME}_generated ${ARGN})

  install(TARGETS
    ${PROJECT_NAME}_generated
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
  )

endmacro()
