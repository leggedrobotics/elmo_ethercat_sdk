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

#define _USE_MATH_DEFINES  // for M_PI
#include <cmath>

#include <elmo_ethercat/Reading.hpp>

namespace elmo {
namespace ethercat {

double Reading::getAgeOfLastErrorInMicroseconds() const {
  ReadingDuration errorDuration = ReadingClock::now() - lastError_.second;
  return errorDuration.count();
}

double Reading::getAgeOfLastFaultInMicroseconds() const {
  ReadingDuration faultDuration = ReadingClock::now() - lastFault_.second;
  return faultDuration.count();
}

void Reading::addError(ErrorType errorType) {
  ErrorPair errorPair;
  errorPair.first = errorType;
  errorPair.second = ReadingClock::now();
  if (lastError_.first == errorType) {
    if (forceAppendEqualError_) {
      errors_.push_front(errorPair);
    } else {
      errors_.pop_front();
      errors_.push_front(errorPair);
    }
  } else {
    errors_.push_front(errorPair);
  }
  lastError_ = errorPair;
  if (errors_.size() > errorStorageCapacity_) {
    errors_.pop_back();
  }
  hasUnreadError_ = true;
}

void Reading::addFault(uint16_t faultCode) {
  FaultPair faultPair;
  faultPair.first = faultCode;
  faultPair.second = ReadingClock::now();
  if (lastFault_.first == faultCode) {
    if (forceAppendEqualFault_) {
      faults_.push_front(faultPair);
    } else {
      faults_.pop_front();
      faults_.push_front(faultPair);
    }
  } else {
    faults_.push_front(faultPair);
  }
  lastFault_ = faultPair;
  if (faults_.size() > faultStorageCapacity_) {
    faults_.pop_back();
  }
  hasUnreadFault_ = true;
}

ErrorTimePairDeque Reading::getErrors() const {
  ReadingTimePoint now = ReadingClock::now();
  ErrorTimePairDeque errors;
  errors.resize(errors_.size());
  ReadingDuration duration;
  for (unsigned int i = 0; i < errors_.size(); i++) {
    errors[i].first = errors_[i].first;
    duration = now - errors_[i].second;
    errors[i].second = duration.count();
  }
  hasUnreadError_ = false;
  return errors;
}

FaultTimePairDeque Reading::getFaults() const {
  ReadingTimePoint now = ReadingClock::now();
  FaultTimePairDeque faults;
  faults.resize(faults_.size());
  ReadingDuration duration;
  for (unsigned int i = 0; i < faults_.size(); i++) {
    faults[i].first = faults_[i].first;
    duration = now - faults_[i].second;
    faults[i].second = duration.count();
  }
  hasUnreadFault_ = false;
  return faults;
}

ErrorType Reading::getLastError() const {
  hasUnreadError_ = false;
  return lastError_.first;
}
uint16_t Reading::getLastFault() const {
  // return 0 if no fault occured
  if (!hasUnreadFault_) {
    return 0;
  }
  hasUnreadFault_ = false;
  return lastFault_.first;
}

void Reading::configureReading(const Configuration& configuration) {
  errorStorageCapacity_ = configuration.getErrorStorageCapacity();
  faultStorageCapacity_ = configuration.getFaultStorageCapacity();
  forceAppendEqualError_ = configuration.getForceAppendEqualError();
  forceAppendEqualFault_ = configuration.getForceAppendEqualFault();

  positionFactorIntegerToRad_ = (2.0 * M_PI) / static_cast<double>(configuration.getPositionEncoderResolution());

  velocityFactorIntegerPerSecToRadPerSec_ = (2.0 * M_PI) / static_cast<double>(configuration.getPositionEncoderResolution());

  double currentFactor = configuration.getMotorRatedCurrentA() / 1000.0;

  currentFactorIntegerToAmp_ = currentFactor;

  torqueFactorIntegerToNm_ = currentFactor * configuration.getMotorConstant() * configuration.getGearRatio();
}

}  // namespace ethercat
}  // namespace elmo
