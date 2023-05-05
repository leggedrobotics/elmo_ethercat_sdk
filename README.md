# Elmo EtherCAT SDK

## Overview
This is a C++ library providing a high-level interface for controlling [Elmo](https://www.elmomc.com/) motor controllers of the [Gold line](https://www.elmomc.com/products/harsh-environment/servo-drive-gold-family/) over EtherCAT (using the [CANopen over EtherCAT CoE](https://www.ethercat.org/en/technology.html#1.9.1) protocol).

The lower level EtherCAT communication is handled by the [soem_interface](https://github.com/leggedrobotics/soem_interface) library.

The source code is released under the GPLv3 license.
A copy of the license is available in the *COPYING* file.

**Authors:** Jonas Junger, Johannes Pankert

**Maintainer:** Johannes Pankert, pankertj@ethz.ch

**Contributors:** Fabio Dubois, Lennart Nachtigall, Markus Staeuble, Martin Wermelinger

## Installation

### Dependencies

#### Ament Packages

Use the `setup/setup_ws.repos` file together with [vcs](https://github.com/dirk-thomas/vcstool):
    
    vcs import < src/elmo_ethercat_sdk/setup/setup_ws.repos

#### System Dependencies (Ubuntu 20.04 LTS)

- [ROS Humble](https://docs.ros.org/en/humble/index.html) for colcon / ament build system.
- yaml-cpp

> Likely to work on Ubuntu 22.04

### Building from Source

To build the library from source, clone the latest version from this repository and from the dependencies into your catkin workspace and compile the package using

	mkdir -p elmo_test_workspace/src 
    cd elmo_test_workspace/src
    git clone -b feature/ament_cmake git@github.com:leggedrobotics/elmo_ethercat_sdk.git
    cd ..
    vcs import < src/elmo_ethercat_sdk/setup/setup_ws.repos --recursive
    <source your humble installation depends how you installed ros> source /opt/ros/humble/setup.bash 
	source src/elmo_ethercat_sdk/setup/build.sh

## Usage

The repos `ethercat_device_configurator` contains an example `standalone.cpp` of how to use one of the sdks.
It is build as part of the building process explained above.
**Note it command per default a constant velocity, so make sure your motor is freely spinning! Also check your hardware before running.**
Run it with:

    ros2 run ethercat_device_configurator standalone <absolute path to setup.yaml file, see example config dictonary> 

## Firmware version
This library is known to work with the following firmware versions:
- 01.01.15.00
- 01.01.16.00
