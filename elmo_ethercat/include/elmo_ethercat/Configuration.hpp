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
#include <utility>

#include <elmo/ModeOfOperationEnum.hpp>
#include <elmo_ethercat/PdoTypeEnum.hpp>

namespace elmo {
namespace ethercat {

class Configuration {
 public:
  Configuration() = default;

  void setModeOfOperation(ModeOfOperationEnum modeOfOperationEnum);

  // set methods for pdo type enums
  void setRxPdoTypeEnum(RxPdoTypeEnum rxPdoTypeEnum);

  void setTxPdoTypeEnum(TxPdoTypeEnum txPdoTypeEnum);

  void setSdoVerifyTimeout(unsigned int sdoVerifyTimeout);

  void setPrintErrorMessageElmoEthercat(bool printErrorMessageElmoEthercat);

  void setDriveStateChangeMinTimeout(unsigned int driveStateChangeMinTimeout);

  void setMinNumberOfSuccessfulTargetStateReadings(unsigned int minNumberOfSuccessfulTargetStateReadings);

  void setDriveStateChangeMaxTimeout(unsigned int driveStateChangeMaxTimeout);

  void setPrintErrorMessageElmoEthercatSlave(bool printErrorMessageElmoEthercatSlave);

  void setForceAppendEqualError(bool forceAppendEqualError);

  void setForceAppendEqualFault(bool forceAppendEqualFault);

  void setErrorStorageCapacity(unsigned int errorStorageCapacity);

  void setFaultStorageCapacity(unsigned int faultStorageCapacity);

  void setPositionEncoderResolution(uint32_t positionEncoderResolution);

  void setUseRawCommands(bool useRawCommands);

  void setGearRatio(std::pair<unsigned int, unsigned int> gearRatio);

  void setMotorConstant(double motorConstant);

  void setMaxCurrentA(double maxCurrentA);

  void setMotorRatedCurrentA(double motorRatedCurrentA);

  void setUseMultipleModeOfOperations(bool useMultipleModeOfOperations);

  // get methods
  ModeOfOperationEnum getModeOfOperation() const;

  RxPdoTypeEnum getRxPdoTypeEnum() const;

  TxPdoTypeEnum getTxPdoTypeEnum() const;

  unsigned int getSdoVerifyTimeout() const;

  bool getPrintErrorMessageElmoEthercat() const;

  unsigned int getDriveStateChangeMinTimeout() const;

  unsigned int getMinNumberOfSuccessfulTargetStateReadings() const;

  unsigned int getDriveStateChangeMaxTimeout() const;

  bool getPrintErrorMessageElmoEthercatSlave() const;

  bool getForceAppendEqualError() const;

  bool getForceAppendEqualFault() const;

  unsigned int getErrorStorageCapacity() const;

  unsigned int getFaultStorageCapacity() const;

  uint32_t getPositionEncoderResolution() const;

  bool getUseRawCommands() const;

  double getGearRatio() const;

  double getMotorConstant() const;

  double getMotorRatedCurrentA() const;

  double getMaxCurrentA() const;

  bool getUseMultipleModeOfOperations() const;

  // stream operator
  friend std::ostream& operator<<(std::ostream& os, const Configuration& configuration);

 protected:
  ModeOfOperationEnum modeOfOperationEnum_{ModeOfOperationEnum::NA};
  RxPdoTypeEnum rxPdoTypeEnum_{RxPdoTypeEnum::NA};
  TxPdoTypeEnum txPdoTypeEnum_{TxPdoTypeEnum::NA};
  unsigned int sdoVerifyTimeout_{20000};
  bool printErrorMessageElmoEthercat_{true};
  unsigned int driveStateChangeMinTimeout_{20000};
  unsigned int minNumberOfSuccessfulTargetStateReadings_{10};
  unsigned int driveStateChangeMaxTimeout_{300000};
  bool printErrorMessageElmoEthercatSlave_{true};
  bool forceAppendEqualError_{true};
  bool forceAppendEqualFault_{false};
  unsigned int errorStorageCapacity_{100};
  unsigned int faultStorageCapacity_{100};
  int32_t positionEncoderResolution_{1};
  bool useRawCommands_{false};
  double gearRatio_{1};
  double motorConstant_{1};
  double motorRatedCurrentA_{0};
  double maxCurrentA_{0};
  bool useMultipleModeOfOperations_{false};
};

}  // namespace ethercat
}  // namespace elmo
