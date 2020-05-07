# Elmo EtherCAT SDK

## Overview
This is a C++ library providing a high-level interface for controlling [Elmo](https://www.elmomc.com/) motor controllers of the [Gold line](https://www.elmomc.com/products/harsh-environment/servo-drive-gold-family/) over EtherCAT (using the [CANopen over EtherCAT CoE](https://www.ethercat.org/en/technology.html#1.9.1) protocol).

The lower level EtherCAT communication is handled by the [soem_interface](https://github.com/leggedrobotics/soem_interface) library.

The elmo_ethercat_sdk is developed on Ubuntu 18.04 LTS with [ROS Melodic](https://wiki.ros.org/melodic).

The source code is released under the GPLv3 license.
A copy of the license is available in the *COPYING* file.

**Authors:** Jonas Junger, Johannes Pankert

**Maintainer:** Johannes Pankert, pankertj@ethz.ch

**Contributors:** Fabio Dubois, Lennart Nachtigall, Markus Staeuble, Martin Wermelinger

## Installation

### Dependencies

#### Catkin Packages

| Repo           | url                                                  | License      | Content                                          |
|:--------------:|:----------------------------------------------------:|:------------:|:------------------------------------------------:|
| soem_interface | https://github.com/leggedrobotics/soem_interface.git | GPLv3        | EtherCAT functionalities                         |
| any_node       | https://github.com/leggedrobotics/any_node.git       | BSD 3-Clause | multi-threading, signal handling functionalities |
| message_logger | https://github.com/leggedrobotics/message_logger.git | BSD 3-Clause | simple log streams                               |

#### System Dependencies (Ubuntu 18.04 LTS)

- [ROS Melodic](https://wiki.ros.org/melodic)
- catkin
- yaml-cpp

### Building from Source

To build the library from source, clone the latest version from this repository and from the dependencies into your catkin workspace and compile the package using

	cd catkin_workspace/src
	git clone https://github.com/leggedrobotics/soem_interface.git
    git clone https://github.com/leggedrobotics/any_node.git
    git clone https://github.com/leggedrobotics/message_logger.git
    git clone https://github.com/leggedrobotics/elmo_ethercat_sdk.git
	cd ../
	catkin build elmo_ethercat 

To build the examples, execute the following command inside of your catkin workspace:
	
	catkin build elmo_examples
	
## Examples on how to use the elmo\_ethercat\_sdk
There is an elmo_examples folder (separate catkin package) with two examples that can be used as a reference.
A more detailed documentation of the examples is currently being developed.

## Known bug
The mapping of PDOs currently leads to an unexpected behaviour of the state machine of the Elmo motor controllers.
A reliable workaround is to restart the connection after mapping new PDO types.
If you use the *ElmoEthercatMaster* in your project then you do not have to consider this issue since the restarting after the initial configuration is handled automatically.

## Firmware version
- This library works with the firmware version 01.01.15.00 (October 2018) for the motor controllers. *Other versions are not supported!*


## DISCLAIMER
Due to the current pandemic, we could not test this version of the elmo_ethercat_sdk on hardware.

Tests will be conducted as soon as possible.