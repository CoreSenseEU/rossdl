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


macro(rossdl_generate_system description_file system)
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

  set(RESOURCE_LAUNCH ${ROSSDL_CMAKE_PATH}/share/rossdl_cmake/resources/launcher.py.em)

  string(REGEX REPLACE ":([^:]*)$" "/\\1" _abs_file "${_code_tuple}")
  set(_launch_out_file  ${CMAKE_CURRENT_BINARY_DIR}/launch/${system}.launch.py)

  file(READ ${_abs_file} ROSSDL_SYSTEM_DESCRIPTION)
  ament_index_register_resource(rossdl_system_descriptions CONTENT ${ROSSDL_SYSTEM_DESCRIPTION})

  ament_index_get_resources(ROSSDL_ARTIFACTS rossdl_artifact_descriptions)
  ament_index_get_resources(ROSSDL_SYSTEMS rossdl_system_descriptions)

  if(ROSSDL_ARTIFACTS)
    set(ROSSDL_ARTIFACTS_NO_VOID ${ROSSDL_ARTIFACTS})
  endif()
  if(NOT ROSSDL_ARTIFACTS)
    set(ROSSDL_ARTIFACTS_NO_VOID "None")
  endif()

  if(ROSSDL_SYSTEMS)
    set(ROSSDL_SYSTEMS_NO_VOID ${ROSSDL_SYSTEMS})
  endif()
  if(NOT ROSSDL_SYSTEMS_NO_VOID)
    set(ROSSDL_SYSTEMS_NO_VOID "None")
  endif()

  if(NOT ${PROJECT_NAME} IN_LIST ROSSDL_ARTIFACTS)
    file(GLOB ROSSDL_LOCAL_ARTIFACTS_NO_VOID "${CMAKE_CURRENT_SOURCE_DIR}/*.ros2")
  endif()
  if(NOT ROSSDL_LOCAL_ARTIFACTS_NO_VOID)
    set(ROSSDL_LOCAL_ARTIFACTS_NO_VOID "None")
  endif()

  if(NOT ${PROJECT_NAME} IN_LIST ROSSDL_SYSTEMS)
    file(GLOB ROSSDL_LOCAL_SYSTEMS_NO_VOID "${CMAKE_CURRENT_SOURCE_DIR}/*.rossystem")
  endif()
  if(NOT ROSSDL_LOCAL_SYSTEMS_NO_VOID)
    set(ROSSDL_LOCAL_SYSTEMS_NO_VOID "None")
  endif()

  message(STATUS "ROSSDL_ARTIFACT_NO_VOID: " ${ROSSDL_ARTIFACTS_NO_VOID})
  message(STATUS "ROSSDL_LOCAL_ARTIFACTS_NO_VOID: " ${ROSSDL_LOCAL_ARTIFACTS_NO_VOID})
  message(STATUS "ROSSDL_SYSTEMS_NO_VOID: " ${ROSSDL_SYSTEMS_NO_VOID})
  message(STATUS "ROSSDL_LOCAL_SYSTEMS_NO_VOID: " ${ROSSDL_LOCAL_SYSTEMS_NO_VOID})

  add_custom_command(
    OUTPUT ${_launch_out_file}
    COMMAND ros2
    ARGS run rossdl_cmake sdl_generator_launch
        --package ${PROJECT_NAME}
        --description-file ${_abs_file}
        --artifacts ${ROSSDL_ARTIFACTS_NO_VOID}
        --local-artifacts ${ROSSDL_LOCAL_ARTIFACTS_NO_VOID}
        --systems ${ROSSDL_SYSTEMS_NO_VOID}
        --local-systems ${ROSSDL_LOCAL_SYSTEMS_NO_VOID}
        --launch-out-file ${_launch_out_file}
        --system ${system}
        DEPENDS ${_abs_file} ${RESOURCE_LAUNCH}
    COMMENT "Generating launcher for ROS 2 System: ${system}"
    VERBATIM
  )

  add_custom_target(file_toucher
  COMMAND ${CMAKE_COMMAND} -E touch_nocreate ${_abs_file})

  add_custom_target(${PROJECT_NAME}_run ALL
  DEPENDS ${_launch_out_file} file_toucher)

  install(FILES
    ${_launch_out_file}
    DESTINATION share/${PROJECT_NAME}/launch
  )

endmacro()
