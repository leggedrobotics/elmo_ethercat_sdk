#include "elmo_ethercat_sdk/DriveState.hpp"

std::ostream& operator<<(std::ostream& os, const elmo::DriveState& driveState){
  switch(driveState){
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
