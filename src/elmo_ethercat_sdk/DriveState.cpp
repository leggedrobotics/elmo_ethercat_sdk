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

#include "elmo_ethercat_sdk/DriveState.hpp"

std::ostream& operator<<(std::ostream& os, const elmo::DriveState& driveState) {
  switch (driveState) {
    case elmo::DriveState::NotReadyToSwitchOn:
      os << "NotReadyToSwitchOn";
      break;
    case elmo::DriveState::SwitchOnDisabled:
      os << "SwitchOnDisabled";
      break;
    case elmo::DriveState::ReadyToSwitchOn:
      os << "ReadyToSwitchOn";
      break;
    case elmo::DriveState::SwitchedOn:
      os << "SwitchedOn";
      break;
    case elmo::DriveState::OperationEnabled:
      os << "OperationEnabled";
      break;
    case elmo::DriveState::QuickStopActive:
      os << "QuickStopActive";
      break;
    case elmo::DriveState::FaultReactionActive:
      os << "FaultReactionActive";
      break;
    case elmo::DriveState::Fault:
      os << "Fault";
      break;
    case elmo::DriveState::NA:
      os << "NA";
      break;
  }
  return os;
}
