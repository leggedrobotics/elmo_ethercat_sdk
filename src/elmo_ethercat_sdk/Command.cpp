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

#include <iomanip>

#include "elmo_ethercat_sdk/Command.hpp"

namespace elmo {

Command::Command(const Command& other) {
  targetPositionUU_ = other.targetPositionUU_;
  targetVelocityUU_ = other.targetVelocityUU_;
  targetTorqueUU_ = other.targetTorqueUU_;
  targetCurrentUU_ = other.targetCurrentUU_;
  maxTorqueUU_ = other.maxTorqueUU_;
  maxCurrentUU_ = other.maxCurrentUU_;
  torqueOffsetUU_ = other.torqueOffsetUU_;

  targetPosition_ = other.targetPosition_;
  targetVelocity_ = other.targetVelocity_;
  targetTorque_ = other.targetTorque_;
  targetCurrent_ = other.targetCurrent_;
  maxTorque_ = other.maxTorque_;
  maxCurrent_ = other.maxCurrent_;
  torqueOffset_ = other.torqueOffset_;

  digitalOutputs_ = other.digitalOutputs_;

  positionFactorRadToInteger_ = other.positionFactorRadToInteger_;
  velocityFactorRadPerSecToIntegerPerSec_ = other.velocityFactorRadPerSecToIntegerPerSec_;
  torqueFactorNmToInteger_ = other.torqueFactorNmToInteger_;
  currentFactorAToInteger_ = other.currentFactorAToInteger_;

  modeOfOperation_ = other.modeOfOperation_;

  useRawCommands_ = other.useRawCommands_;
  targetTorqueCommandUsed_ = other.targetTorqueCommandUsed_;
}

Command& Command::operator=(const Command& other) {
  targetPositionUU_ = other.targetPositionUU_;
  targetVelocityUU_ = other.targetVelocityUU_;
  targetTorqueUU_ = other.targetTorqueUU_;
  targetCurrentUU_ = other.targetCurrentUU_;
  maxTorqueUU_ = other.maxTorqueUU_;
  maxCurrentUU_ = other.maxCurrentUU_;
  torqueOffsetUU_ = other.torqueOffsetUU_;

  targetPosition_ = other.targetPosition_;
  targetVelocity_ = other.targetVelocity_;
  targetTorque_ = other.targetTorque_;
  targetCurrent_ = other.targetCurrent_;
  maxTorque_ = other.maxTorque_;
  maxCurrent_ = other.maxCurrent_;
  torqueOffset_ = other.torqueOffset_;

  digitalOutputs_ = other.digitalOutputs_;

  positionFactorRadToInteger_ = other.positionFactorRadToInteger_;
  velocityFactorRadPerSecToIntegerPerSec_ = other.velocityFactorRadPerSecToIntegerPerSec_;
  torqueFactorNmToInteger_ = other.torqueFactorNmToInteger_;
  currentFactorAToInteger_ = other.currentFactorAToInteger_;

  modeOfOperation_ = other.modeOfOperation_;

  useRawCommands_ = other.useRawCommands_;
  targetTorqueCommandUsed_ = other.targetTorqueCommandUsed_;
  return *this;
}

std::ostream& operator<<(std::ostream& os, Command& command) {
  os << std::left << std::setw(25) << "Target Position:" << command.targetPositionUU_ << "\n"
     << std::setw(25) << "Target Velocity:" << command.targetVelocityUU_ << "\n"
     << std::setw(25) << "Target Torque:" << command.targetTorqueUU_ << "\n"
     << std::setw(25) << "Target Current:" << command.targetCurrentUU_ << "\n"
     << std::setw(25) << "Maximum Torque:" << command.maxTorqueUU_ << "\n"
     << std::setw(25) << "Maximum Current:" << command.maxCurrentUU_ << "\n"
     << std::setw(25) << "Torque Offset:" << command.torqueOffsetUU_ << "\n"
     << std::setw(25) << "Digital Outputs:" << command.getDigitalOutputString() << "\n"
     << std::right;

  return os;
}

uint32_t Command::getDigitalOutputs() const {
  return digitalOutputs_;
}

std::string Command::getDigitalOutputString() const {
  std::string outputs;
  for (unsigned int i = 0; i < 8 * sizeof(digitalOutputs_); i++) {
    if ((digitalOutputs_ & (1 << (sizeof(digitalOutputs_) * 8 - 1 - i))) != 0) {
      outputs += "1";
    } else {
      outputs += "0";
    }
    if (((i + 1) % 8) == 0) {
      outputs += " ";
    }
  }
  outputs.erase(outputs.end() - 1);
  return outputs;
}

/*!
 * Raw set methods
 */
void Command::setTargetPositionRaw(int32_t targetPosition) {
  targetPosition_ = targetPosition;
}
void Command::setTargetVelocityRaw(int32_t targetVelocity) {
  targetVelocity_ = targetVelocity;
}
void Command::setTargetCurrentRaw(int16_t targetCurrent) {
  targetCurrent_ = targetCurrent;
}
void Command::setTorqueOffsetRaw(int16_t torqueOffset) {
  torqueOffset_ = torqueOffset;
}

/*!
 * user unit set methods
 */
void Command::setTargetPosition(double targetPosition) {
  targetPositionUU_ = targetPosition;
}
void Command::setTargetVelocity(double targetVelocity) {
  targetVelocityUU_ = targetVelocity;
}
void Command::setTargetTorque(double targetTorque) {
  // lock for thread safety
  std::lock_guard<std::mutex> lockGuard(targetTorqueCommandMutex_);
  targetTorqueUU_ = targetTorque;
  targetTorqueCommandUsed_ = true;
}
void Command::setTargetCurrent(double targetCurrent) {
  // lock for thread safety
  std::lock_guard<std::mutex> lockGuard(targetTorqueCommandMutex_);
  targetCurrentUU_ = targetCurrent;
  targetTorqueCommandUsed_ = false;
}
void Command::setTorqueOffset(double torqueOffset) {
  torqueOffsetUU_ = torqueOffset;
}
void Command::setMaxCurrent(double maxCurrent) {
  maxCurrentUU_ = maxCurrent;
}
void Command::setMaxTorque(double maxTorque) {
  maxTorqueUU_ = maxTorque;
}

/*!
 * factors set methods
 */
void Command::setPositionFactorRadToInteger(double factor) {
  positionFactorRadToInteger_ = factor;
}
void Command::setVelocityFactorRadPerSecToIntegerPerSec(double factor) {
  velocityFactorRadPerSecToIntegerPerSec_ = factor;
}
void Command::setTorqueFactorNmToInteger(double factor) {
  torqueFactorNmToInteger_ = factor;
}
void Command::setCurrentFactorAToInteger(double factor) {
  currentFactorAToInteger_ = factor;
}

/*!
 * other set methods
 */
void Command::setDigitalOutputs(uint32_t digitalOutputs) {
  digitalOutputs_ = digitalOutputs;
}
void Command::setUseRawCommands(bool useRawCommands) {
  useRawCommands_ = useRawCommands;
}
void Command::setModeOfOperation(const ModeOfOperationEnum modeOfOperation) {
  modeOfOperation_ = modeOfOperation;
}

/*
 * get methods (raw units)
 */
int32_t Command::getTargetPositionRaw() const {
  return targetPosition_;
}
int32_t Command::getTargetVelocityRaw() const {
  return targetVelocity_;
}
int16_t Command::getTargetTorqueRaw() const {
  return targetTorque_;
}
int16_t Command::getTargetCurrentRaw() const {
  return targetCurrent_;
}
uint16_t Command::getMaxTorqueRaw() const {
  return maxTorque_;
}
uint16_t Command::getMaxCurrentRaw() const {
  return maxCurrent_;
}
int16_t Command::getTorqueOffsetRaw() const {
  return torqueOffset_;
}

/*
 * get methods (user units)
 */
double Command::getTargetPosition() const {
  return targetPositionUU_;
}
double Command::getTargetVelocity() const {
  return targetVelocityUU_;
}
double Command::getTargetTorque() const {
  return targetTorqueUU_;
}
double Command::getTargetCurrent() const {
  return targetCurrentUU_;
}
double Command::getMaxTorque() const {
  return maxTorqueUU_;
}
double Command::getMaxCurrent() const {
  return maxCurrentUU_;
}
double Command::getTorqueOffset() const {
  return torqueOffsetUU_;
}

void Command::doUnitConversion() {
  if (!useRawCommands_) {
    targetPosition_ = static_cast<int32_t>(positionFactorRadToInteger_ * targetPositionUU_);
    targetVelocity_ = static_cast<int32_t>(velocityFactorRadPerSecToIntegerPerSec_ * targetVelocityUU_);
    targetTorque_ = static_cast<int16_t>(torqueFactorNmToInteger_ * targetTorqueUU_);
    {
      std::lock_guard<std::mutex> lockGuard(targetTorqueCommandMutex_);
      if (targetTorqueCommandUsed_) {
        targetCurrent_ = static_cast<int16_t>(torqueFactorNmToInteger_ * targetTorqueUU_);
      } else {
        targetCurrent_ = static_cast<int16_t>(currentFactorAToInteger_ * targetCurrentUU_);
      }
    }
    maxTorque_ = static_cast<uint16_t>(torqueFactorNmToInteger_ * maxTorqueUU_);
    maxCurrent_ = static_cast<uint16_t>(currentFactorAToInteger_ * maxCurrentUU_);
    torqueOffset_ = static_cast<int16_t>(torqueFactorNmToInteger_ * torqueOffsetUU_);
  }
}

/// other get methods
ModeOfOperationEnum Command::getModeOfOperation() const {
  return modeOfOperation_;
}

}  // namespace elmo
