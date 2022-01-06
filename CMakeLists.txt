# Copyright (2019-2020) Robotics Systems Lab - ETH Zurich:
# Jonas Junger, Johannes Pankert, Fabio Dubois, Lennart Nachtigall,
# Markus Staeuble
#
# This file is part of the elmo_ethercat_sdk.
# The elmo_ethercat_sdk is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# The elmo_ethercat_sdk is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with the elmo_ethercat_sdk. If not, see <https://www.gnu.org/licenses/>.

cmake_minimum_required(VERSION 3.5)
project(elmo_ethercat_sdk)

## Set the C++ standard.
set(CMAKE_CXX_STANDARD 17)

## Enable all compiler warnings and treat them as errors.
add_compile_options(-Wall -Wextra -Wpedantic)

## Export compile commands for clang.
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(message_logger REQUIRED)
find_package(ethercat_sdk_master REQUIRED)
find_package(yaml-cpp REQUIRED)

###########
## Build ##
###########

add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}/Elmo.cpp
  src/${PROJECT_NAME}/Configuration.cpp
  src/${PROJECT_NAME}/ConfigurationParser.cpp
  src/${PROJECT_NAME}/Reading.cpp
  src/${PROJECT_NAME}/Command.cpp
  src/${PROJECT_NAME}/Controlword.cpp
  src/${PROJECT_NAME}/Statusword.cpp
  src/${PROJECT_NAME}/DriveState.cpp
  src/${PROJECT_NAME}/PdoTypeEnum.cpp
)

target_include_directories(${PROJECT_NAME}
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

ament_target_dependencies(
  ${PROJECT_NAME}
    ethercat_sdk_master
    message_logger
    yaml-cpp
)

ament_export_dependencies(
    message_logger
    ethercat_sdk_master
    ament_cmake
)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
#############
## Install ##
#############

install(
    TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}Target
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)

install(
    DIRECTORY
    include/${PROJECT_NAME}/
    DESTINATION include
)
ament_package()