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

#include <chrono>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <string>

#include <elmo/DriveState.hpp>
#include <elmo/Statusword.hpp>

namespace elmo {

/*!
 * aliases for time_points, durations and clocks
 */
using ReadingClock = std::chrono::steady_clock;
using ReadingDuration = std::chrono::duration<double, std::milli>;
using ReadingTimePoint = std::chrono::time_point<ReadingClock>;

class Reading {
 public:
  /*!
   * raw get methods
   */
  int32_t getActualPositionRaw() const;
  int32_t getActualVelocityRaw() const;
  uint16_t getRawStatusword() const;
  int16_t getActualCurrentRaw() const;
  uint16_t getAnalogInputRaw() const;
  uint32_t getBusVoltageRaw() const;

  /*!
   * User units get methods
   */
  double getActualPosition() const;
  double getActualVelocity() const;
  double getActualCurrent() const;
  double getActualTorque() const;
  double getAnalogInput() const;
  double getAgeOfLastReadingInMicroseconds() const;
  double getBusVoltage() const;

  /*!
   * Other get methods
   */
  int32_t getDigitalInputs() const;
  Statusword getStatusword() const;
  std::string getDigitalInputString() const;
  DriveState getDriveState() const;

  /*!
   * set methods (only raw)
   */
  void setActualPosition(int32_t actualPosition);

  void setDigitalInputs(int32_t digitalInputs);

  void setActualVelocity(int32_t actualVelocity);

  void setStatusword(uint16_t statusword);

  void setAnalogInput(int16_t analogInput);

  void setActualCurrent(int16_t actualCurrent);

  void setBusVoltage(uint32_t busVoltage);

  void setTimePointNow();

  void setPositionFactorIntegerToRad(double positionFactor);

  void setVelocityFactorIntegerPerSecToRadPerSec(double velocityFactor);

  void setCurrentFactorIntegerToAmp(double currentFactor);

  void setTorqueFactorIntegerToNm(double torqueFactor);

  /*!
   * stream operator
   */
  friend std::ostream& operator<<(std::ostream& os, Reading& reading);

 protected:
  int32_t actualPosition_{0};
  int32_t digitalInputs_{0};
  int32_t actualVelocity_{0};
  uint16_t statusword_{0};
  int16_t analogInput_{0};
  int16_t actualCurrent_{0};
  uint32_t busVoltage_{0};

  double positionFactorIntegerToRad_{1};
  double velocityFactorIntegerPerSecToRadPerSec_{1};
  double currentFactorIntegerToAmp_{1};
  double torqueFactorIntegerToNm_{1};

  ReadingTimePoint lastReadingTimePoint_;
};

}  // namespace elmo
