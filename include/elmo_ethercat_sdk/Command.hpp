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

#pragma once

#include <cstdint>
#include <iostream>
#include <mutex>
#include <string>

#include "elmo_ethercat_sdk/ModeOfOperationEnum.hpp"

namespace elmo {

class Command {
 public:
  Command() = default;
  Command(const Command& other);
  virtual ~Command() = default;

  Command& operator=(const Command& other);

  /*!
   * Set raw commands
   * This requires the "SET_USE_RAW_COMMANDS" variable of the config file to be
   * set to "true"
   */

  void setTargetPositionRaw(int32_t targetPosition);
  void setTargetVelocityRaw(int32_t targetVelocity);
  void setTargetCurrentRaw(int16_t targetCurrent);
  void setTorqueOffsetRaw(int16_t torqueOffset);

  /// set factors
  void setPositionFactorRadToInteger(double factor);
  void setVelocityFactorRadPerSecToIntegerPerSec(double factor);
  void setTorqueFactorNmToInteger(double factor);
  void setCurrentFactorAToInteger(double factor);

  /// set user units
  void setTargetPosition(double targetPosition);
  void setTargetVelocity(double targetVelocity);
  void setTargetTorque(double targetTorque);
  void setTargetCurrent(double targetCurrent);
  void setTorqueOffset(double torqueOffset);

  /// other
  void setDigitalOutputs(uint32_t digitalOutputs);
  void setUseRawCommands(bool useRawCommands);
  void setModeOfOperation(const ModeOfOperationEnum modeOfOperation);

  /// get (raw)
  int32_t getTargetPositionRaw() const;
  int32_t getTargetVelocityRaw() const;
  int16_t getTargetTorqueRaw() const;
  int16_t getTargetCurrentRaw() const;
  uint16_t getMaxTorqueRaw() const;
  uint16_t getMaxCurrentRaw() const;
  int16_t getTorqueOffsetRaw() const;

  /// get (user units)
  double getTargetPosition() const;
  double getTargetVelocity() const;
  double getTargetTorque() const;
  double getTargetCurrent() const;
  double getMaxTorque() const;
  double getMaxCurrent() const;
  double getTorqueOffset() const;

  /*!
   * Get the digital outputs.
   * Only available as integer value
   * Use the "getDigitalOutputString" method to print out the state of the
   * target state of the individual pins.
   */
  uint32_t getDigitalOutputs() const;

  /// get (other)
  std::string getDigitalOutputString() const;
  ModeOfOperationEnum getModeOfOperation() const;

  /// Convert the units
  void doUnitConversion();

  /// only works if commands in user units (A, Nm, rad/s,..) are used
  friend std::ostream& operator<<(std::ostream& os, Command& command);

  // Set command max current/torque
  void setMaxCurrent(double maxCurrent);
  void setMaxTorque(double maxTorque);

 private:
  double targetPositionUU_{0};
  double targetVelocityUU_{0};
  double targetTorqueUU_{0};
  double targetCurrentUU_{0};
  double maxTorqueUU_{0};
  double maxCurrentUU_{0};
  double torqueOffsetUU_{0};

  int32_t targetPosition_{0};
  int32_t targetVelocity_{0};
  int16_t targetTorque_{0};
  int16_t targetCurrent_{0};
  uint16_t maxTorque_{0};
  uint16_t maxCurrent_{0};
  int16_t torqueOffset_{0};

  std::mutex targetTorqueCommandMutex_;

  uint32_t digitalOutputs_{0};

  double positionFactorRadToInteger_{1};
  double velocityFactorRadPerSecToIntegerPerSec_{1};
  double torqueFactorNmToInteger_{1};
  double currentFactorAToInteger_{1};

  ModeOfOperationEnum modeOfOperation_{ModeOfOperationEnum::NA};

  /*!
   * set this to true if raw commands have been used and therefore no unit
   * conversion should be done
   */
  bool useRawCommands_{false};

  bool targetTorqueCommandUsed_{false};
};

}  // namespace elmo
