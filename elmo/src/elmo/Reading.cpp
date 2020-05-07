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

#include <elmo/Reading.hpp>

namespace elmo {

std::ostream& operator<<(std::ostream& os, Reading& reading) {
  // TODO(duboisf) make table, remove statusword
  os << std::left << std::setw(30) << "Actual Position:" << reading.getActualPosition() << "\n"
     << std::setw(30) << "Actual Velocity:" << reading.getActualVelocity() << "\n"
     << std::setw(30) << "Actual Torque:" << reading.getActualTorque() << "\n"
     << std::setw(30) << "Analog input" << reading.getAnalogInput() << "\n"
     << std::setw(30) << "Actual Current:" << reading.getActualCurrent() << "\n"
     << std::setw(30) << "Digital Inputs:" << reading.getDigitalInputString() << "\n"
     << std::setw(30) << "Bus Voltage:" << reading.getBusVoltage() << "\n"
     << std::setw(30) << "\nStatusword:"
     << "\n"
     << reading.getStatusword() << std::right;
  return os;
}

std::string Reading::getDigitalInputString() const {
  std::string binString;
  for (unsigned int i = 0; i < 8 * sizeof(digitalInputs_); i++) {
    if ((digitalInputs_ & (1 << (8 * sizeof(digitalInputs_) - 1 - i))) != 0) {
      binString += "1";
    } else {
      binString += "0";
    }
    if ((i + 1) % 8 == 0) {
      binString += " ";
    }
  }
  binString.erase(binString.end() - 1);
  return binString;
}

DriveState Reading::getDriveState() const {
  return getStatusword().getDriveState();
}

double Reading::getAgeOfLastReadingInMicroseconds() const {
  ReadingDuration readingDuration = ReadingClock::now() - lastReadingTimePoint_;
  return readingDuration.count();
}

/*!
 * Raw get methods
 */
int32_t Reading::getActualPositionRaw() const {
  return actualPosition_;
}
int32_t Reading::getActualVelocityRaw() const {
  return actualVelocity_;
}
uint16_t Reading::getRawStatusword() const {
  return statusword_;
}
int16_t Reading::getActualCurrentRaw() const {
  return actualCurrent_;
}
uint16_t Reading::getAnalogInputRaw() const {
  return analogInput_;
}
uint32_t Reading::getBusVoltageRaw() const {
  return busVoltage_;
}

/*!
 * User unit get methods
 */
double Reading::getActualPosition() const {
  return static_cast<double>(actualPosition_) * positionFactorIntegerToRad_;
}
double Reading::getActualVelocity() const {
  return static_cast<double>(actualVelocity_) * velocityFactorIntegerPerSecToRadPerSec_;
}
double Reading::getActualCurrent() const {
  return static_cast<double>(actualCurrent_) * currentFactorIntegerToAmp_;
}
double Reading::getActualTorque() const {
  return static_cast<double>(actualCurrent_) * torqueFactorIntegerToNm_;
}
double Reading::getAnalogInput() const {
  return static_cast<double>(analogInput_) * 0.001;
}

/*!
 * Other readings
 */
int32_t Reading::getDigitalInputs() const {
  return digitalInputs_;
}
Statusword Reading::getStatusword() const {
  Statusword statusword;
  statusword.setFromRawStatusword(statusword_);
  return statusword;
}
double Reading::getBusVoltage() const {
  return 0.001 * static_cast<double>(busVoltage_);
}

/*!
 * Raw set methods
 */
void Reading::setActualPosition(int32_t actualPosition) {
  actualPosition_ = actualPosition;
}
void Reading::setDigitalInputs(int32_t digitalInputs) {
  digitalInputs_ = digitalInputs;
}
void Reading::setActualVelocity(int32_t actualVelocity) {
  actualVelocity_ = actualVelocity;
}
void Reading::setStatusword(uint16_t statusword) {
  statusword_ = statusword;
}

void Reading::setAnalogInput(int16_t analogInput) {
  analogInput_ = analogInput;
}
void Reading::setActualCurrent(int16_t actualCurrent) {
  actualCurrent_ = actualCurrent;
}
void Reading::setBusVoltage(uint32_t busVoltage) {
  busVoltage_ = busVoltage;
}
void Reading::setTimePointNow() {
  lastReadingTimePoint_ = ReadingClock::now();
}

void Reading::setPositionFactorIntegerToRad(double positionFactor) {
  positionFactorIntegerToRad_ = positionFactor;
}
void Reading::setVelocityFactorIntegerPerSecToRadPerSec(double velocityFactor) {
  velocityFactorIntegerPerSecToRadPerSec_ = velocityFactor;
}
void Reading::setCurrentFactorIntegerToAmp(double currentFactor) {
  currentFactorIntegerToAmp_ = currentFactor;
}
void Reading::setTorqueFactorIntegerToNm(double torqueFactor) {
  torqueFactorIntegerToNm_ = torqueFactor;
}

}  // namespace elmo
