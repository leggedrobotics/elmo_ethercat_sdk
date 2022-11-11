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

#include "elmo_ethercat_sdk/Command.hpp"
#include "elmo_ethercat_sdk/Controlword.hpp"
#include "elmo_ethercat_sdk/DriveState.hpp"
#include "elmo_ethercat_sdk/Reading.hpp"

#include <ethercat_sdk_master/EthercatDevice.hpp>

#include <yaml-cpp/yaml.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <string>

namespace elmo {
class Elmo : public ecat_master::EthercatDevice {
 public:
  typedef std::shared_ptr<Elmo> SharedPtr;

  // create Elmo Drive from setup file
  static SharedPtr deviceFromFile(const std::string& configFile, const std::string& name, const uint32_t address);
  // constructor
  Elmo() = default;
  Elmo(const std::string& name, const uint32_t address);

  // pure virtual overwrites
 public:
  bool startup() override;
  void shutdown() override;
  void updateWrite() override;
  void updateRead() override;
  PdoInfo getCurrentPdoInfo() const override { return pdoInfo_; }

 public:
  void stageCommand(const Command& command);
  Reading getReading() const;
  void getReading(Reading& reading) const;

  bool loadConfigFile(const std::string& fileName);
  bool loadConfigNode(YAML::Node configNode);
  bool loadConfiguration(const Configuration& configuration);
  Configuration getConfiguration() const;

  // SDO
 public:
  bool getStatuswordViaSdo(Statusword& statusword);
  bool setControlwordViaSdo(Controlword& controlword);
  bool setDriveStateViaSdo(const DriveState& driveState);

  // PDO
 public:
  bool setDriveStateViaPdo(const DriveState& driveState, bool waitForState);
  bool lastPdoStateChangeSuccessful() const { return stateChangeSuccessful_; }

  // Other
  double getActual5vVoltage() { return actual5vVoltage_; }

 private:
  void engagePdoStateMachine();
  bool stateTransitionViaSdo(const StateTransition& stateTransition);
  bool mapPdos(RxPdoTypeEnum rxPdoTypeEnum, TxPdoTypeEnum txPdoTypeEnum);
  Controlword getNextStateTransitionControlword(const DriveState& requestedDriveState, const DriveState& currentDriveState);
  void autoConfigurePdoSizes();
  uint16_t getTxPdoSize();
  uint16_t getRxPdoSize();

  mutable std::mutex stagedCommandMutex_;
  Command stagedCommand_;

  mutable std::mutex readingMutex_;
  Reading reading_;

  Configuration configuration_;
  Controlword controlword_;
  PdoInfo pdoInfo_;
  uint16_t numberOfSuccessfulTargetStateReadings_{0};

  DriveState prevDriveState_{DriveState::NA};
  mutable std::mutex targetStateMutex_;
  DriveState targetDriveState_{DriveState::NA};
  std::atomic<bool> stateChangeSuccessful_{false};
  std::atomic<bool> conductStateChange_{false};

  std::mutex driveStateMachineSyncMutex_;  // only for blocking call to setDriveStateViaPdo
  std::condition_variable cvDriveStateMachineSync_;

  // actual voltage on 5v line (e.g. to configure analog sensors)
  double actual5vVoltage_{5.0};

  bool allowModeChange_{false};
  ModeOfOperationEnum modeOfOperation_{ModeOfOperationEnum::NA};
};
}  // namespace elmo
