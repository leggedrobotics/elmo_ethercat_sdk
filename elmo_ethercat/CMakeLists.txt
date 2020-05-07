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

cmake_minimum_required(VERSION 2.8.3)
project(elmo_ethercat)

## Set the C++ standard.
set(CMAKE_CXX_STANDARD 14)

## Enable all compiler warnings and treat them as errors.
add_compile_options(-Wall -Wextra -Wpedantic)

## Export compile commands for clang.
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CATKIN_PACKAGE_DEPENDENCIES
  any_worker  
  elmo
  message_logger
  signal_handler
  soem_interface
)

find_package(catkin REQUIRED
  COMPONENTS
    ${CATKIN_PACKAGE_DEPENDENCIES}
)

find_package(yaml-cpp REQUIRED)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    ${CATKIN_PACKAGE_DEPENDENCIES}
)

###########
## Build ##
###########

include_directories(
  include
  SYSTEM
    ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}/Configuration.cpp
  src/${PROJECT_NAME}/ConfigurationParser.cpp
  src/${PROJECT_NAME}/ElmoEthercat.cpp
  src/${PROJECT_NAME}/ElmoEthercatMaster.cpp
  src/${PROJECT_NAME}/ElmoEthercatSlave.cpp
  src/${PROJECT_NAME}/Reading.cpp
)
add_dependencies(
  ${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)

#############
## Install ##
#############

install(
  TARGETS
    ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY
    include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

###########
## Clang ##
###########

find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND)
  add_clang_tooling(
    TARGETS
      ${PROJECT_NAME}
    SOURCE_DIRS
      ${CMAKE_CURRENT_SOURCE_DIR}/src
      ${CMAKE_CURRENT_SOURCE_DIR}/include
    CT_HEADER_DIRS
      ${CMAKE_CURRENT_SOURCE_DIR}/include
    CF_WERROR
    # CT_WERROR
    # CF_FIX
  )
endif(cmake_clang_tools_FOUND)
