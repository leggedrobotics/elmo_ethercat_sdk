/*
** Copyright (2019-2020) Robotics Systems Lab - ETH Zurich:
** Jonas Junger, Johannes Pankert, Fabio Dubois, Lennart Nachtigall,
** Markus Staeuble
**
** This file is part of the elmo_ethercat_sdk.
** The elmo_ethercat_sdk is free software: you can redistribute it and/or modify
** it under the terms of the GNU General Public License as published by
** the Free Software Foundation, either version 3 of the License, or
** (at your option) any later version.
**
** The elmo_ethercat_sdk is distributed in the hope that it will be useful,
** but WITHOUT ANY WARRANTY; without even the implied warranty of
** MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
** GNU General Public License for more details.
**
** You should have received a copy of the GNU General Public License
** along with the elmo_ethercat_sdk. If not, see <https://www.gnu.org/licenses/>.
 */

#include <iostream>
#include <string>

#include <elmo/DriveState.hpp>

namespace elmo {
void printState(const DriveState& driveState) {
  std::string returner = "";
  switch (driveState) {
    case DriveState::NA:
      returner = "NA";
      break;
    case DriveState::SwitchOnDisabled:
      returner = "Switch on Disabled";
      break;
    case DriveState::NotReadyToSwitchOn:
      returner = "not ready to switch on";
      break;
    case DriveState::SwitchedOn:
      returner = "switched on";
      break;
    case DriveState::Fault:
      returner = "fault_";
      break;
    case DriveState::ReadyToSwitchOn:
      returner = "ready to switch on";
      break;
    case DriveState::FaultReactionActive:
      returner = "fault_ reaction active";
      break;
    case DriveState::OperationEnabled:
      returner = "operation enabled";
      break;
    case DriveState::QuickStopActive:
      returner = "quickStopActive";
      break;
  }
  std::cout << returner << std::endl;
}
}  // namespace elmo
