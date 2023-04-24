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

#include <algorithm>
#include <iomanip>
#include <string>
#include <utility>
#include <vector>

#include "elmo_ethercat_sdk/Configuration.hpp"

namespace elmo {

bool Configuration::sanityCheck(bool silent) {
  bool success = true;
  std::string message = "";
  auto check_and_inform = [&message, &success](std::pair<bool, std::string> test) {
    if (test.first) {
      message += "\033[32m✓\t";
      message += test.second;
      message += "\033[m\n";
      success &= true;
    } else {
      message += "\033[31m❌\t";
      message += test.second;
      message += "\033[m\n";
      success = false;
    }
  };

  const std::vector<std::pair<bool, std::string>> sanity_tests = {
      {(driveStateChangeMinTimeout <= driveStateChangeMaxTimeout), "drive_state_change_min_timeout ≤ drive_state_change_max_timeout"},
      {(motorConstant > 0), "motor_constant > 0"},
      {(motorRatedCurrentA > 0), "motor_rated_current > 0"},
      {(maxCurrentA > 0), "max_current > 0"},
      {(positionEncoderResolution > 0), "position_encoder_resolution > 0"},
      {(gearRatio > 0), "gear_ratio > 0"},
      {(direction == 1 || direction == -1), "direction ∈ {1, -1}"},
      {(encoderPosition == EncoderPosition::motor || encoderPosition == EncoderPosition::joint),
       "encoder_position ∈ {\"motor\", \"joint\"}"},
      {(modeOfOperationEnum == ModeOfOperationEnum::CyclicSynchronousVelocityMode ||
        modeOfOperationEnum == ModeOfOperationEnum::CyclicSynchronousTorqueMode),
       "mode_of_operation ∈ {\"CyclicSynchronousVelocityMode\", \"CyclicSynchronounsTorqueMode\"}"},
  };

  std::for_each(sanity_tests.begin(), sanity_tests.end(), check_and_inform);

  if (!silent) std::cout << message << std::endl;

  return success;
}

std::string modeOfOperationString(ModeOfOperationEnum modeOfOperation_) {
  switch (modeOfOperation_) {
    case ModeOfOperationEnum::ProfiledPositionMode:
      return "Profiled Position Mode";
    case ModeOfOperationEnum::ProfiledVelocityMode:
      return "Profiled Velocity Mode";
    case ModeOfOperationEnum::ProfiledTorqueMode:
      return "Profiled Torque Mode";
    case ModeOfOperationEnum::HomingMode:
      return "Homing Mode";
    case ModeOfOperationEnum::CyclicSynchronousPositionMode:
      return "Cyclic Synchronous Position Mode";
    case ModeOfOperationEnum::CyclicSynchronousVelocityMode:
      return "Cyclic Synchronous Velocity Mode";
    case ModeOfOperationEnum::CyclicSynchronousTorqueMode:
      return "Cyclic Synchronous Torque Mode";
    default:
      return "Unsupported Mode of Operation";
  }
}

std::string rxPdoString(RxPdoTypeEnum rxPdo) {
  switch (rxPdo) {
    case RxPdoTypeEnum::NA:
      return "NA";
    case RxPdoTypeEnum::RxPdoStandard:
      return "Rx PDO Standard";
    case RxPdoTypeEnum::RxPdoCST:
      return "Rx PDO CST";
    default:
      return "Unsupported Type";
  }
}

std::string txPdoString(TxPdoTypeEnum txPdo) {
  switch (txPdo) {
    case TxPdoTypeEnum::NA:
      return "NA";
    case TxPdoTypeEnum::TxPdoCST:
      return "Tx PDO CST";
    case TxPdoTypeEnum::TxPdoStandard:
      return "Tx PDO Standard";
    default:
      return "Unsupported Type";
  }
}

std::ostream& operator<<(std::ostream& os, const Configuration& configuration) {
  std::string modeOfOperation = modeOfOperationString(configuration.modeOfOperationEnum);
  std::string rxPdo = rxPdoString(configuration.rxPdoTypeEnum);
  std::string txPdo = txPdoString(configuration.txPdoTypeEnum);

  std::string encoderPosition = "NA";
  if (configuration.encoderPosition == Configuration::EncoderPosition::motor)
    encoderPosition = "motor";
  else if (configuration.encoderPosition == Configuration::EncoderPosition::joint)
    encoderPosition = "joint";

  std::string direction = "ERROR";
  if (configuration.direction == 1)
    direction = "+1";
  else if (configuration.direction == -1)
    direction = "-1";

  // The size of the second columne
  unsigned int tmp1 = rxPdo.size();
  unsigned int tmp2 = txPdo.size();
  unsigned int tmp3 = modeOfOperation.size();
  unsigned int len2 = tmp1 >= tmp2 ? tmp1 : tmp2;
  len2 = len2 >= tmp3 ? len2 : tmp3;
  len2++;

  os << std::boolalpha << std::left << std::setw(43) << std::setfill('-') << "|" << std::setw(len2 + 2) << "-"
     << "|\n"
     << std::setfill(' ') << std::setw(43 + len2 + 2) << "| Configuration"
     << "|\n"
     << std::setw(43) << std::setfill('-') << "|" << std::setw(len2 + 2) << "+"
     << "|\n"
     << std::setfill(' ') << std::setw(43) << "| Direction:"
     << "| " << std::setw(len2) << direction << "|\n"
     << std::setfill(' ') << std::setw(43) << "| Encoder Position:"
     << "| " << std::setw(len2) << encoderPosition << "|\n"
     << std::setfill(' ') << std::setw(43) << "| Max Current [A]:"
     << "| " << std::setw(len2) << configuration.maxCurrentA << "|\n"
     << std::setfill(' ') << std::setw(43) << "| Motor Rated Current [A]:"
     << "| " << std::setw(len2) << configuration.motorRatedCurrentA << "|\n"
     << std::setfill(' ') << std::setw(43) << "| Motor Constant:"
     << "| " << std::setw(len2) << configuration.motorConstant << "|\n"
     << std::setfill(' ') << std::setw(43) << "| Mode of Operation:"
     << "| " << std::setw(len2) << modeOfOperation << "|\n"
     << std::setfill(' ') << std::setw(43) << "| Multiple Modes of Operation:"
     << "| " << std::setw(len2) << configuration.useMultipleModeOfOperations << "|\n"
     << std::setw(43) << "| Rx PDO Type:"
     << "| " << std::setw(len2) << rxPdo << "|\n"
     << std::setw(43) << "| Tx PDO Type:"
     << "| " << std::setw(len2) << txPdo << "|\n"
     << std::setw(43) << "| Config Run SDO verify timeout:"
     << "| " << std::setw(len2) << configuration.configRunSdoVerifyTimeout << "|\n"
     << std::setw(43) << "| Print Debug Messages:"
     << "| " << std::setw(len2) << configuration.printDebugMessages << "|\n"
     << std::setw(43) << "| Drive State Change Min Timeout:"
     << "| " << std::setw(len2) << configuration.driveStateChangeMinTimeout << "|\n"
     << std::setw(43) << "| Drive State Change Max Timeout:"
     << "| " << std::setw(len2) << configuration.driveStateChangeMaxTimeout << "|\n"
     << std::setw(43) << "| Min Successful Target State Readings:"
     << "| " << std::setw(len2) << configuration.minNumberOfSuccessfulTargetStateReadings << "|\n"
     << std::setw(43) << "| Force Append Equal Error:"
     << "| " << std::setw(len2) << configuration.forceAppendEqualError << "|\n"
     << std::setw(43) << "| Force Append Equal Fault:"
     << "| " << std::setw(len2) << configuration.forceAppendEqualFault << "|\n"
     << std::setw(43) << "| Error Storage Capacity"
     << "| " << std::setw(len2) << configuration.errorStorageCapacity << "|\n"
     << std::setw(43) << "| Fault Storage Capacity"
     << "| " << std::setw(len2) << configuration.faultStorageCapacity << "|\n"
     << std::setw(43) << std::setfill('-') << "|" << std::setw(len2 + 2) << "+"
     << "|\n"
     << std::setfill(' ') << std::noboolalpha << std::right;
  return os;
}
}  // namespace elmo
