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

#include <elmo_ethercat/Configuration.hpp>

namespace elmo {
namespace ethercat {

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
  std::string modeOfOperation_ = modeOfOperationString(configuration.modeOfOperationEnum_);
  std::string rxPdo = rxPdoString(configuration.rxPdoTypeEnum_);
  std::string txPdo = txPdoString(configuration.txPdoTypeEnum_);

  // The size of the second columne
  unsigned int tmp1 = rxPdo.size();
  unsigned int tmp2 = txPdo.size();
  unsigned int tmp3 = modeOfOperation_.size();
  unsigned int len2 = tmp1 >= tmp2 ? tmp1 : tmp2;
  len2 = len2 >= tmp3 ? len2 : tmp3;
  len2++;

  os << std::boolalpha << std::left << std::setw(43) << std::setfill('-') << "|" << std::setw(len2 + 2) << "-"
     << "|\n"
     << std::setfill(' ') << std::setw(43 + len2 + 2) << "| Configuration"
     << "|\n"
     << std::setw(43) << std::setfill('-') << "|" << std::setw(len2 + 2) << "+"
     << "|\n"
     << std::setfill(' ') << std::setw(43) << "| Mode of Operation:"
     << "| " << std::setw(len2) << modeOfOperation_ << "|\n"
     << std::setw(43) << "| Rx PDO Type:"
     << "| " << std::setw(len2) << rxPdo << "|\n"
     << std::setw(43) << "| Tx PDO Type:"
     << "| " << std::setw(len2) << txPdo << "|\n"
     << std::setw(43) << "| SDO Verify Timeout:"
     << "| " << std::setw(len2) << configuration.sdoVerifyTimeout_ << "|\n"
     << std::setw(43) << "| Print Error Message (ElmoEthercat):"
     << "| " << std::setw(len2) << configuration.printErrorMessageElmoEthercat_ << "|\n"
     << std::setw(43) << "| Print Error Message (ElmoEthercatSlave):"
     << "| " << std::setw(len2) << configuration.printErrorMessageElmoEthercatSlave_ << "|\n"
     << std::setw(43) << "| Drive State Change Min Timeout:"
     << "| " << std::setw(len2) << configuration.driveStateChangeMinTimeout_ << "|\n"
     << std::setw(43) << "| Drive State Change Max Timeout:"
     << "| " << std::setw(len2) << configuration.driveStateChangeMaxTimeout_ << "|\n"
     << std::setw(43) << "| Min Successful Target State Readings:"
     << "| " << std::setw(len2) << configuration.minNumberOfSuccessfulTargetStateReadings_ << "|\n"
     << std::setw(43) << "| Force Append Equal Error:"
     << "| " << std::setw(len2) << configuration.forceAppendEqualError_ << "|\n"
     << std::setw(43) << "| Force Append Equal Fault:"
     << "| " << std::setw(len2) << configuration.forceAppendEqualFault_ << "|\n"
     << std::setw(43) << "| Error Storage Capacity"
     << "| " << std::setw(len2) << configuration.errorStorageCapacity_ << "|\n"
     << std::setw(43) << "| Fault Storage Capacity"
     << "| " << std::setw(len2) << configuration.faultStorageCapacity_ << "|\n"
     << std::setw(43) << std::setfill('-') << "|" << std::setw(len2 + 2) << "+"
     << "|\n"
     << std::setfill(' ') << std::noboolalpha << std::right;
  return os;
}

// set methods
void Configuration::setRxPdoTypeEnum(RxPdoTypeEnum rxPdoTypeEnum) {
  rxPdoTypeEnum_ = rxPdoTypeEnum;
}

void Configuration::setTxPdoTypeEnum(TxPdoTypeEnum txPdoTypeEnum) {
  txPdoTypeEnum_ = txPdoTypeEnum;
}
void Configuration::setModeOfOperation(ModeOfOperationEnum modeOfOperationEnum) {
  modeOfOperationEnum_ = modeOfOperationEnum;
}

void Configuration::setSdoVerifyTimeout(unsigned int sdoVerifyTimeout) {
  sdoVerifyTimeout_ = sdoVerifyTimeout;
}
void Configuration::setPrintErrorMessageElmoEthercat(bool printErrorMessageElmoEthercat) {
  printErrorMessageElmoEthercat_ = printErrorMessageElmoEthercat;
}
void Configuration::setDriveStateChangeMinTimeout(unsigned int driveStateChangeMinTimeout) {
  driveStateChangeMinTimeout_ = driveStateChangeMinTimeout;
}
void Configuration::setMinNumberOfSuccessfulTargetStateReadings(unsigned int minNumberOfSuccessfulTargetStateReadings) {
  minNumberOfSuccessfulTargetStateReadings_ = minNumberOfSuccessfulTargetStateReadings;
}
void Configuration::setDriveStateChangeMaxTimeout(unsigned int driveStateChangeMaxTimeout) {
  driveStateChangeMaxTimeout_ = driveStateChangeMaxTimeout;
}
void Configuration::setPrintErrorMessageElmoEthercatSlave(bool printErrorMessageElmoEthercatSlave) {
  printErrorMessageElmoEthercatSlave_ = printErrorMessageElmoEthercatSlave;
}
void Configuration::setForceAppendEqualError(bool forceAppendEqualError) {
  forceAppendEqualError_ = forceAppendEqualError;
}
void Configuration::setForceAppendEqualFault(bool forceAppendEqualFault) {
  forceAppendEqualFault_ = forceAppendEqualFault;
}
void Configuration::setErrorStorageCapacity(unsigned int errorStorageCapacity) {
  errorStorageCapacity_ = errorStorageCapacity;
}
void Configuration::setFaultStorageCapacity(unsigned int faultStorageCapacity) {
  faultStorageCapacity_ = faultStorageCapacity;
}
void Configuration::setPositionEncoderResolution(uint32_t positionEncoderResolution) {
  positionEncoderResolution_ = positionEncoderResolution;
}
void Configuration::setUseRawCommands(bool useRawCommands) {
  useRawCommands_ = useRawCommands;
}
void Configuration::setGearRatio(std::pair<unsigned int, unsigned int> gearRatio) {
  gearRatio_ = static_cast<double>(gearRatio.first) / static_cast<double>(gearRatio.second);
}
void Configuration::setMotorConstant(double motorConstant) {
  motorConstant_ = motorConstant;
}
void Configuration::setMotorRatedCurrentA(double motorRatedCurrentA) {
  motorRatedCurrentA_ = motorRatedCurrentA;
}
void Configuration::setMaxCurrentA(double maxCurrentA) {
  maxCurrentA_ = maxCurrentA;
}
void Configuration::setUseMultipleModeOfOperations(bool useMultipleModeOfOperations) {
  useMultipleModeOfOperations_ = useMultipleModeOfOperations;
}

// get methods
ModeOfOperationEnum Configuration::getModeOfOperation() const {
  return modeOfOperationEnum_;
}

RxPdoTypeEnum Configuration::getRxPdoTypeEnum() const {
  return rxPdoTypeEnum_;
}

TxPdoTypeEnum Configuration::getTxPdoTypeEnum() const {
  return txPdoTypeEnum_;
}
unsigned int Configuration::getSdoVerifyTimeout() const {
  return sdoVerifyTimeout_;
}
bool Configuration::getPrintErrorMessageElmoEthercat() const {
  return printErrorMessageElmoEthercat_;
}
unsigned int Configuration::getDriveStateChangeMinTimeout() const {
  return driveStateChangeMinTimeout_;
}
unsigned int Configuration::getMinNumberOfSuccessfulTargetStateReadings() const {
  return minNumberOfSuccessfulTargetStateReadings_;
}
unsigned int Configuration::getDriveStateChangeMaxTimeout() const {
  return driveStateChangeMaxTimeout_;
}
bool Configuration::getPrintErrorMessageElmoEthercatSlave() const {
  return printErrorMessageElmoEthercatSlave_;
}
bool Configuration::getForceAppendEqualError() const {
  return forceAppendEqualError_;
}
bool Configuration::getForceAppendEqualFault() const {
  return forceAppendEqualFault_;
}
unsigned int Configuration::getErrorStorageCapacity() const {
  return errorStorageCapacity_;
}
unsigned int Configuration::getFaultStorageCapacity() const {
  return faultStorageCapacity_;
}
uint32_t Configuration::getPositionEncoderResolution() const {
  return positionEncoderResolution_;
}
bool Configuration::getUseRawCommands() const {
  return useRawCommands_;
}
double Configuration::getGearRatio() const {
  return gearRatio_;
}
double Configuration::getMotorConstant() const {
  return motorConstant_;
}
double Configuration::getMotorRatedCurrentA() const {
  return motorRatedCurrentA_;
}
double Configuration::getMaxCurrentA() const {
  return maxCurrentA_;
}
bool Configuration::getUseMultipleModeOfOperations() const {
  return useMultipleModeOfOperations_;
}

}  // namespace ethercat
}  // namespace elmo
