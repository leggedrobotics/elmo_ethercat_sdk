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

#include "elmo_ethercat_sdk/Elmo.hpp"
#include "elmo_ethercat_sdk/ConfigurationParser.hpp"
#include "elmo_ethercat_sdk/ObjectDictionary.hpp"
#include "elmo_ethercat_sdk/RxPdo.hpp"
#include "elmo_ethercat_sdk/TxPdo.hpp"

#include <chrono>
#include <cmath>
#include <stdexcept>
#include <thread>

namespace elmo {
std::string binstring(uint16_t var) {
  std::string s = "0000000000000000";
  for (int i = 0; i < 16; i++) {
    if (var & (1 << (15 - i))) {
      s[i] = '1';
    }
  }
  return s;
}
std::string binstring(int8_t var) {
  std::string s = "00000000";
  for (int i = 0; i < 8; i++) {
    if (var & (1 << (7 - i))) {
      s[i] = '1';
    }
  }
  return s;
}

Elmo::SharedPtr Elmo::deviceFromFile(const std::string& configFile, const std::string& name, const uint32_t address) {
  auto elmo = std::make_shared<Elmo>(name, address);
  if (!elmo->loadConfigFile(configFile)) {
    MELO_ERROR_STREAM("[elmo_ethercat_sdk:Elmo::deviceFromFile] loading config file '" << configFile << "' for '" << name
                                                                                       << "' not successful.");
    throw std::runtime_error("[elmo_ethercat_sdk:Elmo::deviceFromFile] config file loading error");
  }
  return elmo;
}

Elmo::Elmo(const std::string& name, const uint32_t address) {
  address_ = address;
  name_ = name;
}

bool Elmo::startup() {
  bool success = true;
  success &= bus_->waitForState(EC_STATE_PRE_OP, address_, 50, 0.05);
  bus_->syncDistributedClock0(address_, true, timeStep_, timeStep_ / 2.f);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // use hardware motor rated current value if necessary
  if (configuration_.motorRatedCurrentA == 0.0) {
    uint32_t motorRatedCurrent;
    success &= sendSdoRead(OD_INDEX_MOTOR_RATED_CURRENT, 0, false, motorRatedCurrent);
    // update the configuration to accomodate the new motor rated current value
    configuration_.motorRatedCurrentA = static_cast<double>(motorRatedCurrent) / 1000.0;
    // update the reading_ object to ensure correct unit conversion
    reading_.configureReading(configuration_);
  }
  success &= setDriveStateViaSdo(DriveState::ReadyToSwitchOn);
  // PDO mapping
  success &= mapPdos(configuration_.rxPdoTypeEnum, configuration_.txPdoTypeEnum);
  // Set initial mode of operation
  success &= sdoVerifyWrite(OD_INDEX_MODES_OF_OPERATION, 0, false, static_cast<int8_t>(configuration_.modeOfOperationEnum),
                            configuration_.configRunSdoVerifyTimeout);
  // To be on the safe side: set currect PDO sizes
  autoConfigurePdoSizes();

  // write the motor rated current / torque to the drives
  uint32_t motorRatedCurrent = static_cast<uint32_t>(round(1000.0 * configuration_.motorRatedCurrentA));
  success &= sdoVerifyWrite(OD_INDEX_MOTOR_RATED_CURRENT, 0, false, motorRatedCurrent);
  success &= sdoVerifyWrite(OD_INDEX_MOTOR_RATED_TORQUE, 0, false, motorRatedCurrent);

  // Write maximum current to drive
  uint16_t maxCurrent = static_cast<uint16_t>(floor(1000.0 * configuration_.maxCurrentA));
  success &= sdoVerifyWrite(OD_INDEX_MAX_CURRENT, 0, false, maxCurrent);

  // Actual voltage on 5v bus (e.g. for connected analog sensors)
  uint16_t actual5vVoltage = 5000;
  success &= sendSdoRead(OD_INDEX_5VDC_SUPPLY, 0, false, actual5vVoltage);  // [mV]
  actual5vVoltage_ = static_cast<double>(actual5vVoltage) / 1000.0;         // [V]

  if (!success) {
    MELO_ERROR_STREAM("[elmo_ethercat_sdk:Elmo::preStartupOnlineConfiguration] hardware configuration of '" << name_
                                                                                                            << "' not successful!");
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  return success;
}

void Elmo::shutdown() {
  bus_->setState(EC_STATE_INIT, address_);
}

void Elmo::updateWrite() {
  /*
  ** Check if the Mode of Operation has been set properly
  */
  if (modeOfOperation_ == ModeOfOperationEnum::NA) {
    reading_.addError(ErrorType::ModeOfOperationError);
    MELO_ERROR_STREAM("[elmo_ethercat_sdk:Elmo::updateWrite] Mode of operation for '" << name_ << "' has not been set.");
    return;
  }

  /*!
   * engage the state machine if a state change is requested
   */
  if (conductStateChange_) {
    engagePdoStateMachine();
  }

  switch (configuration_.rxPdoTypeEnum) {
    case RxPdoTypeEnum::RxPdoStandard: {
      RxPdoStandard rxPdo{};
      {
        std::lock_guard<std::mutex> stagedCmdLock(stagedCommandMutex_);
        rxPdo.targetPosition_ = stagedCommand_.getTargetPositionRaw() * configuration_.direction;
        rxPdo.targetVelocity_ = stagedCommand_.getTargetVelocityRaw() * configuration_.direction;
        rxPdo.targetTorque_ = stagedCommand_.getTargetTorqueRaw() * configuration_.direction;
        rxPdo.maxTorque_ = stagedCommand_.getMaxTorqueRaw();
        rxPdo.modeOfOperation_ = static_cast<int8_t>(modeOfOperation_);
        rxPdo.torqueOffset_ = stagedCommand_.getTorqueOffsetRaw() * configuration_.direction;
        rxPdo.controlWord_ = controlword_.getRawControlword();
      }

      // actually writing to the hardware
      bus_->writeRxPdo(address_, rxPdo);
    } break;
    case RxPdoTypeEnum::RxPdoCST: {
      RxPdoCST rxPdo{};
      {
        std::lock_guard<std::mutex> stagedCmdLock(stagedCommandMutex_);
        rxPdo.targetTorque_ = stagedCommand_.getTargetTorqueRaw() * configuration_.direction;
        rxPdo.modeOfOperation_ = static_cast<int8_t>(modeOfOperation_);
        rxPdo.controlWord_ = controlword_.getRawControlword();
      }
      // actually writing to the hardware
      bus_->writeRxPdo(address_, rxPdo);
    } break;

    default:
      MELO_ERROR_STREAM("[elmo_ethercat_sdk:Elmo::updateWrite] Unsupported Rx Pdo type for '" << name_ << "'");
  }
}

void Elmo::updateRead() {
  switch (configuration_.txPdoTypeEnum) {
    case TxPdoTypeEnum::TxPdoStandard: {
      TxPdoStandard txPdo{};
      // reading from the bus
      bus_->readTxPdo(address_, txPdo);
      reading_.setActualPosition(txPdo.actualPosition_ * configuration_.direction);
      reading_.setDigitalInputs(txPdo.digitalInputs_);
      reading_.setActualVelocity(txPdo.actualVelocity_ * configuration_.direction);
      reading_.setStatusword(txPdo.statusword_);
      reading_.setAnalogInput(txPdo.analogInput_);
      reading_.setActualCurrent(txPdo.actualCurrent_ * configuration_.direction);
      reading_.setBusVoltage(txPdo.busVoltage_);
    } break;
    case TxPdoTypeEnum::TxPdoCST: {
      TxPdoCST txPdo{};
      // reading from the bus
      bus_->readTxPdo(address_, txPdo);
      reading_.setActualPosition(txPdo.actualPosition_ * configuration_.direction);
      reading_.setActualCurrent(txPdo.actualTorque_ * configuration_.direction);  /// torque readings are actually current readings,
                                                                                  /// the conversion is handled later
      reading_.setStatusword(txPdo.statusword_);
      reading_.setActualVelocity(txPdo.actualVelocity_ * configuration_.direction);
    } break;

    default:
      MELO_ERROR_STREAM("[elmo_ethercat_sdk:Elmo::updateRrite] Unsupported Tx Pdo type for '" << name_ << "'");
      reading_.addError(ErrorType::TxPdoTypeError);
  }

  // Print warning if drive is in Fault state.
  if (reading_.getDriveState() == DriveState::Fault) {
    MELO_ERROR_STREAM("[elmo_ethercat_sdk:Elmo::updateRead] '" << name_ << "' is in drive state 'Fault'");
  }
}

void Elmo::stageCommand(const Command& command) {
  std::lock_guard<std::mutex> lock(stagedCommandMutex_);
  stagedCommand_ = command;
  if (configuration_.encoderPosition == Configuration::EncoderPosition::joint) {
    stagedCommand_.setPositionFactorRadToInteger(static_cast<double>(configuration_.positionEncoderResolution) / (2.0 * M_PI));
    stagedCommand_.setVelocityFactorRadPerSecToIntegerPerSec(static_cast<double>(configuration_.positionEncoderResolution) / (2.0 * M_PI));
  } else if (configuration_.encoderPosition == Configuration::EncoderPosition::motor) {
    stagedCommand_.setPositionFactorRadToInteger(static_cast<double>(configuration_.positionEncoderResolution) / (2.0 * M_PI) *
                                                 configuration_.gearRatio);
    stagedCommand_.setVelocityFactorRadPerSecToIntegerPerSec(static_cast<double>(configuration_.positionEncoderResolution) / (2.0 * M_PI) *
                                                             configuration_.gearRatio);
  } else {
    stagedCommand_.setPositionFactorRadToInteger(0.0);
    stagedCommand_.setVelocityFactorRadPerSecToIntegerPerSec(0.0);
  }

  double currentFactorAToInt = 1000.0 / configuration_.motorRatedCurrentA;
  stagedCommand_.setCurrentFactorAToInteger(currentFactorAToInt);
  stagedCommand_.setTorqueFactorNmToInteger(currentFactorAToInt / configuration_.motorConstant / configuration_.gearRatio);

  stagedCommand_.setMaxCurrent(configuration_.maxCurrentA);
  stagedCommand_.setMaxTorque(configuration_.maxCurrentA * configuration_.motorConstant * configuration_.gearRatio);

  stagedCommand_.setUseRawCommands(configuration_.useRawCommands);

  stagedCommand_.doUnitConversion();

  if (allowModeChange_ && command.getModeOfOperation() != ModeOfOperationEnum::NA) {
    modeOfOperation_ = command.getModeOfOperation();
  } else {
    if (modeOfOperation_ != command.getModeOfOperation() && command.getModeOfOperation() != ModeOfOperationEnum::NA) {
      MELO_ERROR_STREAM("[elmo_ethercat_sdk:Elmo::stageCommand] Changing the mode of operation of '"
                        << name_ << "' is not allowed for the active configuration.");
    }
  }
}

Reading Elmo::getReading() const {
  std::lock_guard<std::mutex> lock(readingMutex_);
  return reading_;
}

void Elmo::getReading(Reading& reading) const {
  std::lock_guard<std::mutex> lock(readingMutex_);
  reading = reading_;
}

bool Elmo::loadConfigFile(const std::string& fileName) {
  ConfigurationParser configurationParser(fileName);
  return loadConfiguration(configurationParser.getConfiguration());
}

bool Elmo::loadConfigNode(YAML::Node configNode) {
  ConfigurationParser configurationParser(configNode);
  return loadConfiguration(configurationParser.getConfiguration());
}

bool Elmo::loadConfiguration(const Configuration& configuration) {
  reading_.configureReading(configuration);

  // Check if changing mode of operation will be allowed
  allowModeChange_ = true;
  allowModeChange_ &= configuration.useMultipleModeOfOperations;
  allowModeChange_ &= (configuration.rxPdoTypeEnum == RxPdoTypeEnum::RxPdoStandard);
  allowModeChange_ &= (configuration.txPdoTypeEnum == TxPdoTypeEnum::TxPdoStandard);

  modeOfOperation_ = configuration.modeOfOperationEnum;

  configuration_ = configuration;
  MELO_INFO_STREAM("Configuration Sanity Check of Elmo '" << getName() << "':");
  return configuration_.sanityCheck();
}

Configuration Elmo::getConfiguration() const {
  return configuration_;
}

bool Elmo::getStatuswordViaSdo(Statusword& statusword) {
  uint16_t statuswordValue = 0;
  bool success = sendSdoRead(OD_INDEX_STATUSWORD, 0, false, statuswordValue);
  statusword.setFromRawStatusword(statuswordValue);
  return success;
}

bool Elmo::setControlwordViaSdo(Controlword& controlword) {
  return sendSdoWrite(OD_INDEX_CONTROLWORD, 0, false, controlword.getRawControlword());
}

bool Elmo::setDriveStateViaSdo(const DriveState& driveState) {
  bool success = true;
  Statusword currentStatusword;
  success &= getStatuswordViaSdo(currentStatusword);
  DriveState currentDriveState = currentStatusword.getDriveState();

  // do the adequate state changes (via sdo) depending on the requested and
  // current drive states
  switch (driveState) {
    // Target: switch on disabled
    // This is the lowest state in which the state machine can be brought over
    // EtherCAT
    case DriveState::SwitchOnDisabled:
      switch (currentDriveState) {
        case DriveState::SwitchOnDisabled:
          success &= true;
          break;
        case DriveState::ReadyToSwitchOn:
          success &= stateTransitionViaSdo(StateTransition::_7);
          break;
        case DriveState::SwitchedOn:
          success &= stateTransitionViaSdo(StateTransition::_10);
          break;
        case DriveState::OperationEnabled:
          success &= stateTransitionViaSdo(StateTransition::_9);
          break;
        case DriveState::QuickStopActive:
          success &= stateTransitionViaSdo(StateTransition::_12);
          break;
        case DriveState::Fault:
          success &= stateTransitionViaSdo(StateTransition::_15);
          break;
        default:
          MELO_ERROR_STREAM("[elmo_ethercat_sdk:Elmo::setDriveStateViaSdo] State Transition not implemented");
          success = false;
      }
      break;

    case DriveState::ReadyToSwitchOn:
      switch (currentDriveState) {
        case DriveState::SwitchOnDisabled:
          success &= stateTransitionViaSdo(StateTransition::_2);
          break;
        case DriveState::ReadyToSwitchOn:
          success &= true;
          break;
        case DriveState::SwitchedOn:
          success &= stateTransitionViaSdo(StateTransition::_6);
          break;
        case DriveState::OperationEnabled:
          success &= stateTransitionViaSdo(StateTransition::_8);
          break;
        case DriveState::QuickStopActive:
          success &= stateTransitionViaSdo(StateTransition::_12);
          success &= stateTransitionViaSdo(StateTransition::_2);
          break;
        case DriveState::Fault:
          success &= stateTransitionViaSdo(StateTransition::_15);
          success &= stateTransitionViaSdo(StateTransition::_2);
          break;
        default:
          MELO_ERROR_STREAM("[elmo_ethercat_sdk:Elmo::setDriveStateViaSdo] State Transition not implemented");
          success = false;
      }
      break;

    case DriveState::SwitchedOn:
      switch (currentDriveState) {
        case DriveState::SwitchOnDisabled:
          success &= stateTransitionViaSdo(StateTransition::_2);
          success &= stateTransitionViaSdo(StateTransition::_3);
          break;
        case DriveState::ReadyToSwitchOn:
          success &= stateTransitionViaSdo(StateTransition::_3);
          break;
        case DriveState::SwitchedOn:
          success &= true;
          break;
        case DriveState::OperationEnabled:
          success &= stateTransitionViaSdo(StateTransition::_5);
          break;
        case DriveState::QuickStopActive:
          success &= stateTransitionViaSdo(StateTransition::_12);
          success &= stateTransitionViaSdo(StateTransition::_2);
          success &= stateTransitionViaSdo(StateTransition::_3);
          break;
        case DriveState::Fault:
          success &= stateTransitionViaSdo(StateTransition::_15);
          success &= stateTransitionViaSdo(StateTransition::_2);
          success &= stateTransitionViaSdo(StateTransition::_3);
          break;
        default:
          MELO_ERROR_STREAM("[elmo_ethercat_sdk:Elmo::setDriveStateViaSdo] State Transition not implemented");
          success = false;
      }
      break;

    case DriveState::OperationEnabled:
      switch (currentDriveState) {
        case DriveState::SwitchOnDisabled:
          success &= stateTransitionViaSdo(StateTransition::_2);
          success &= stateTransitionViaSdo(StateTransition::_3);
          success &= stateTransitionViaSdo(StateTransition::_4);
          break;
        case DriveState::ReadyToSwitchOn:
          success &= stateTransitionViaSdo(StateTransition::_3);
          success &= stateTransitionViaSdo(StateTransition::_4);
          break;
        case DriveState::SwitchedOn:
          success &= stateTransitionViaSdo(StateTransition::_4);
          break;
        case DriveState::OperationEnabled:
          success &= true;
          break;
        case DriveState::QuickStopActive:
          success &= stateTransitionViaSdo(StateTransition::_12);
          success &= stateTransitionViaSdo(StateTransition::_2);
          success &= stateTransitionViaSdo(StateTransition::_3);
          success &= stateTransitionViaSdo(StateTransition::_4);
          break;
        case DriveState::Fault:
          success &= stateTransitionViaSdo(StateTransition::_15);
          success &= stateTransitionViaSdo(StateTransition::_2);
          success &= stateTransitionViaSdo(StateTransition::_3);
          success &= stateTransitionViaSdo(StateTransition::_4);
          break;
        default:
          MELO_ERROR_STREAM("[elmo_ethercat_sdk:Elmo::setDriveStateViaSdo] State Transition not implemented");
          success = false;
      }
      break;

    case DriveState::QuickStopActive:
      switch (currentDriveState) {
        case DriveState::SwitchOnDisabled:
          success &= stateTransitionViaSdo(StateTransition::_2);
          success &= stateTransitionViaSdo(StateTransition::_3);
          success &= stateTransitionViaSdo(StateTransition::_4);
          success &= stateTransitionViaSdo(StateTransition::_11);
          break;
        case DriveState::ReadyToSwitchOn:
          success &= stateTransitionViaSdo(StateTransition::_3);
          success &= stateTransitionViaSdo(StateTransition::_4);
          success &= stateTransitionViaSdo(StateTransition::_11);
          break;
        case DriveState::SwitchedOn:
          success &= stateTransitionViaSdo(StateTransition::_4);
          success &= stateTransitionViaSdo(StateTransition::_11);
          break;
        case DriveState::OperationEnabled:
          success &= stateTransitionViaSdo(StateTransition::_11);
          break;
        case DriveState::QuickStopActive:
          success &= true;
          break;
        case DriveState::Fault:
          success &= stateTransitionViaSdo(StateTransition::_15);
          success &= stateTransitionViaSdo(StateTransition::_2);
          success &= stateTransitionViaSdo(StateTransition::_3);
          success &= stateTransitionViaSdo(StateTransition::_4);
          success &= stateTransitionViaSdo(StateTransition::_11);
          break;
        default:
          MELO_ERROR_STREAM("[elmo_ethercat_sdk:Elmo::setDriveStateViaSdo] State Transition not implemented");
          success = false;
      }
      break;

    default:
      MELO_ERROR_STREAM("[elmo_ethercat_sdk:Elmo::setDriveStateViaSdo] State Transition not implemented");
      success = false;
  }
  return success;
}

bool Elmo::stateTransitionViaSdo(const StateTransition& stateTransition) {
  Controlword controlword;
  switch (stateTransition) {
    case StateTransition::_2:
      controlword.setStateTransition2();
      return setControlwordViaSdo(controlword);
      break;
    case StateTransition::_3:
      controlword.setStateTransition3();
      return setControlwordViaSdo(controlword);
      break;
    case StateTransition::_4:
      controlword.setStateTransition4();
      return setControlwordViaSdo(controlword);
      break;
    case StateTransition::_5:
      controlword.setStateTransition5();
      return setControlwordViaSdo(controlword);
      break;
    case StateTransition::_6:
      controlword.setStateTransition6();
      return setControlwordViaSdo(controlword);
      break;
    case StateTransition::_7:
      controlword.setStateTransition7();
      return setControlwordViaSdo(controlword);
      break;
    case StateTransition::_8:
      controlword.setStateTransition8();
      return setControlwordViaSdo(controlword);
      break;
    case StateTransition::_9:
      controlword.setStateTransition9();
      return setControlwordViaSdo(controlword);
      break;
    case StateTransition::_10:
      controlword.setStateTransition10();
      return setControlwordViaSdo(controlword);
      break;
    case StateTransition::_11:
      controlword.setStateTransition11();
      return setControlwordViaSdo(controlword);
      break;
    case StateTransition::_12:
      controlword.setStateTransition12();
      return setControlwordViaSdo(controlword);
      break;
    case StateTransition::_15:
      controlword.setStateTransition15();
      return setControlwordViaSdo(controlword);
      break;
    default:
      MELO_ERROR_STREAM("[elmo_ethercat_sdk:Elmo::stateTransitionViaSdo] State Transition not implemented");
      return false;
  }
}

bool Elmo::setDriveStateViaPdo(const DriveState& driveState, const bool waitForState) {
  // reset the "stateChangeSuccessful_" flag to false such that a new successful
  // state change can be detected
  stateChangeSuccessful_ = false;

  // make the state machine realize that a state change will have to happen
  conductStateChange_ = true;
  {
    std::unique_lock<std::mutex> targetDriveStateLock(targetStateMutex_);
    targetDriveState_ = driveState;
  }

  // return if no waiting is requested
  if (!waitForState) {
    return true;
  }
  std::unique_lock<std::mutex> targetDriveStateLock(driveStateMachineSyncMutex_);
  if (!cvDriveStateMachineSync_.wait_for(targetDriveStateLock, std::chrono::milliseconds(configuration_.driveStateChangeMaxTimeout),
                                         [this]() -> bool { return stateChangeSuccessful_; })) {
    MELO_WARN_STREAM("[ElmoEtherCAT_sdk] setDriveStateViaPdo Timeout after waiting "
                     << configuration_.driveStateChangeMaxTimeout << "ms for state change. is PDO update thread running?")
    conductStateChange_ = false;  // give up this change.
    return false;
  }
  return true;
}

bool Elmo::mapPdos(RxPdoTypeEnum rxPdoTypeEnum, TxPdoTypeEnum txPdoTypeEnum) {
  bool rxSuccess = true;
  switch (rxPdoTypeEnum) {
    case RxPdoTypeEnum::RxPdoStandard:
      std::this_thread::sleep_for(std::chrono::microseconds(configuration_.configRunSdoVerifyTimeout));
      rxSuccess &= sdoVerifyWrite(OD_INDEX_RX_PDO_ASSIGNMENT, 0, false, static_cast<uint8_t>(0), configuration_.configRunSdoVerifyTimeout);
      std::this_thread::sleep_for(std::chrono::microseconds(configuration_.configRunSdoVerifyTimeout));
      rxSuccess &=
          sdoVerifyWrite(OD_INDEX_RX_PDO_ASSIGNMENT, 1, false, static_cast<uint16_t>(0x1605), configuration_.configRunSdoVerifyTimeout);
      std::this_thread::sleep_for(std::chrono::microseconds(configuration_.configRunSdoVerifyTimeout));
      rxSuccess &=
          sdoVerifyWrite(OD_INDEX_RX_PDO_ASSIGNMENT, 2, false, static_cast<uint16_t>(0x1618), configuration_.configRunSdoVerifyTimeout);
      std::this_thread::sleep_for(std::chrono::microseconds(configuration_.configRunSdoVerifyTimeout));
      rxSuccess &= sdoVerifyWrite(OD_INDEX_RX_PDO_ASSIGNMENT, 0, false, static_cast<uint8_t>(2), configuration_.configRunSdoVerifyTimeout);
      std::this_thread::sleep_for(std::chrono::microseconds(configuration_.configRunSdoVerifyTimeout));
      break;
    case RxPdoTypeEnum::RxPdoCST:
      std::this_thread::sleep_for(std::chrono::microseconds(configuration_.configRunSdoVerifyTimeout));
      rxSuccess &= sdoVerifyWrite(OD_INDEX_RX_PDO_ASSIGNMENT, 0, false, static_cast<uint8_t>(0), configuration_.configRunSdoVerifyTimeout);
      std::this_thread::sleep_for(std::chrono::microseconds(configuration_.configRunSdoVerifyTimeout));
      rxSuccess &=
          sdoVerifyWrite(OD_INDEX_RX_PDO_ASSIGNMENT, 1, false, static_cast<uint16_t>(0x1602), configuration_.configRunSdoVerifyTimeout);
      std::this_thread::sleep_for(std::chrono::microseconds(configuration_.configRunSdoVerifyTimeout));
      rxSuccess &=
          sdoVerifyWrite(OD_INDEX_RX_PDO_ASSIGNMENT, 2, false, static_cast<uint16_t>(0x160b), configuration_.configRunSdoVerifyTimeout);
      rxSuccess &= sdoVerifyWrite(OD_INDEX_RX_PDO_ASSIGNMENT, 0, false, static_cast<uint8_t>(2), configuration_.configRunSdoVerifyTimeout);
      std::this_thread::sleep_for(std::chrono::microseconds(configuration_.configRunSdoVerifyTimeout));
      break;

    case RxPdoTypeEnum::NA:
      MELO_ERROR_STREAM("[elmo_ethercat_sdk:Elmo::mapPdos] Cannot map RxPdoTypeEnum::NA, PdoType not configured properly");
      rxSuccess = false;
      break;
    default:  // Non-implemented type
      MELO_ERROR_STREAM("[elmo_ethercat_sdk:Elmo::mapPdos] Cannot map unimplemented RxPdo, PdoType not configured properly");
      rxSuccess = false;
      break;
  }

  bool txSuccess = true;
  switch (txPdoTypeEnum) {
    case TxPdoTypeEnum::TxPdoStandard:
      txSuccess &= sdoVerifyWrite(OD_INDEX_TX_PDO_ASSIGNMENT, 0, false, static_cast<uint8_t>(0), configuration_.configRunSdoVerifyTimeout);
      std::this_thread::sleep_for(std::chrono::microseconds(configuration_.configRunSdoVerifyTimeout));
      txSuccess &=
          sdoVerifyWrite(OD_INDEX_TX_PDO_ASSIGNMENT, 1, false, static_cast<uint16_t>(0x1a03), configuration_.configRunSdoVerifyTimeout);
      std::this_thread::sleep_for(std::chrono::microseconds(configuration_.configRunSdoVerifyTimeout));
      txSuccess &=
          sdoVerifyWrite(OD_INDEX_TX_PDO_ASSIGNMENT, 2, false, static_cast<uint16_t>(0x1a1d), configuration_.configRunSdoVerifyTimeout);
      std::this_thread::sleep_for(std::chrono::microseconds(configuration_.configRunSdoVerifyTimeout));
      txSuccess &=
          sdoVerifyWrite(OD_INDEX_TX_PDO_ASSIGNMENT, 3, false, static_cast<uint16_t>(0x1a1f), configuration_.configRunSdoVerifyTimeout);
      std::this_thread::sleep_for(std::chrono::microseconds(configuration_.configRunSdoVerifyTimeout));
      txSuccess &=
          sdoVerifyWrite(OD_INDEX_TX_PDO_ASSIGNMENT, 4, false, static_cast<uint16_t>(0x1a18), configuration_.configRunSdoVerifyTimeout);
      std::this_thread::sleep_for(std::chrono::microseconds(configuration_.configRunSdoVerifyTimeout));
      txSuccess &= sdoVerifyWrite(OD_INDEX_TX_PDO_ASSIGNMENT, 0, false, static_cast<uint8_t>(4), configuration_.configRunSdoVerifyTimeout);
      std::this_thread::sleep_for(std::chrono::microseconds(configuration_.configRunSdoVerifyTimeout));
      break;

    case TxPdoTypeEnum::TxPdoCST:
      txSuccess &= sdoVerifyWrite(OD_INDEX_TX_PDO_ASSIGNMENT, 0, false, static_cast<uint8_t>(0), configuration_.configRunSdoVerifyTimeout);
      std::this_thread::sleep_for(std::chrono::microseconds(configuration_.configRunSdoVerifyTimeout));
      txSuccess &=
          sdoVerifyWrite(OD_INDEX_TX_PDO_ASSIGNMENT, 1, false, static_cast<uint16_t>(0x1a02), configuration_.configRunSdoVerifyTimeout);
      std::this_thread::sleep_for(std::chrono::microseconds(configuration_.configRunSdoVerifyTimeout));
      txSuccess &=
          sdoVerifyWrite(OD_INDEX_TX_PDO_ASSIGNMENT, 2, false, static_cast<uint16_t>(0x1a11), configuration_.configRunSdoVerifyTimeout);
      std::this_thread::sleep_for(std::chrono::microseconds(configuration_.configRunSdoVerifyTimeout));
      txSuccess &= sdoVerifyWrite(OD_INDEX_TX_PDO_ASSIGNMENT, 0, false, static_cast<uint8_t>(2), configuration_.configRunSdoVerifyTimeout);
      std::this_thread::sleep_for(std::chrono::microseconds(configuration_.configRunSdoVerifyTimeout));
      break;

    case TxPdoTypeEnum::NA:
      MELO_ERROR_STREAM("[elmo_ethercat_sdk:Elmo::mapPdos] Cannot map TxPdoTypeEnum::NA, PdoType not configured properly");
      txSuccess = false;
      break;
    default:  // if any case was forgotten
      MELO_ERROR_STREAM("[elmo_ethercat_sdk:Elmo::mapPdos] Cannot map undefined TxPdo, PdoType not configured properly");
      txSuccess = false;
      break;
  }
  return (txSuccess && rxSuccess);
}
Controlword Elmo::getNextStateTransitionControlword(const DriveState& requestedDriveState, const DriveState& currentDriveState) {
  Controlword controlword;
  controlword.setAllFalse();
  switch (requestedDriveState) {
    case DriveState::SwitchOnDisabled:
      switch (currentDriveState) {
        case DriveState::SwitchOnDisabled:
          MELO_ERROR_STREAM("[elmo_ethercat_sdk:Elmo::getNextStateTransitionControlword] "
                            << "drive state has already been reached for '" << name_ << "'");
          break;
        case DriveState::ReadyToSwitchOn:
          controlword.setStateTransition7();
          break;
        case DriveState::SwitchedOn:
          controlword.setStateTransition10();
          break;
        case DriveState::OperationEnabled:
          controlword.setStateTransition9();
          break;
        case DriveState::QuickStopActive:
          controlword.setStateTransition12();
          break;
        case DriveState::Fault:
          controlword.setStateTransition15();
          break;
        default:
          MELO_ERROR_STREAM("[elmo_ethercat_sdk:Elmo::getNextStateTransitionControlword] "
                            << "PDO state transition not implemented for '" << name_ << "'");
      }
      break;

    case DriveState::ReadyToSwitchOn:
      switch (currentDriveState) {
        case DriveState::SwitchOnDisabled:
          controlword.setStateTransition2();
          break;
        case DriveState::ReadyToSwitchOn:
          MELO_ERROR_STREAM("[elmo_ethercat_sdk:Elmo::getNextStateTransitionControlword] "
                            << "drive state has already been reached for '" << name_ << "'");
          break;
        case DriveState::SwitchedOn:
          controlword.setStateTransition6();
          break;
        case DriveState::OperationEnabled:
          controlword.setStateTransition8();
          break;
        case DriveState::QuickStopActive:
          controlword.setStateTransition12();
          break;
        case DriveState::Fault:
          controlword.setStateTransition15();
          break;
        default:
          MELO_ERROR_STREAM("[elmo_ethercat_sdk:Elmo::getNextStateTransitionControlword] "
                            << "PDO state transition not implemented for '" << name_ << "'");
      }
      break;

    case DriveState::SwitchedOn:
      switch (currentDriveState) {
        case DriveState::SwitchOnDisabled:
          controlword.setStateTransition2();
          break;
        case DriveState::ReadyToSwitchOn:
          controlword.setStateTransition3();
          break;
        case DriveState::SwitchedOn:
          MELO_ERROR_STREAM("[elmo_ethercat_sdk:Elmo::getNextStateTransitionControlword] "
                            << "drive state has already been reached for '" << name_ << "'");
          break;
        case DriveState::OperationEnabled:
          controlword.setStateTransition5();
          break;
        case DriveState::QuickStopActive:
          controlword.setStateTransition12();
          break;
        case DriveState::Fault:
          controlword.setStateTransition15();
          break;
        default:
          MELO_ERROR_STREAM("[elmo_ethercat_sdk:Elmo::getNextStateTransitionControlword] "
                            << "PDO state transition not implemented for '" << name_ << "'");
      }
      break;

    case DriveState::OperationEnabled:
      switch (currentDriveState) {
        case DriveState::SwitchOnDisabled:
          controlword.setStateTransition2();
          break;
        case DriveState::ReadyToSwitchOn:
          controlword.setStateTransition3();
          break;
        case DriveState::SwitchedOn:
          controlword.setStateTransition4();
          break;
        case DriveState::OperationEnabled:
          MELO_ERROR_STREAM("[elmo_ethercat_sdk:Elmo::getNextStateTransitionControlword] "
                            << "drive state has already been reached for '" << name_ << "'");
          break;
        case DriveState::QuickStopActive:
          controlword.setStateTransition12();
          break;
        case DriveState::Fault:
          controlword.setStateTransition15();
          break;
        default:
          MELO_ERROR_STREAM("[elmo_ethercat_sdk:Elmo::getNextStateTransitionControlword] "
                            << "PDO state transition not implemented for '" << name_ << "'");
      }
      break;

    case DriveState::QuickStopActive:
      switch (currentDriveState) {
        case DriveState::SwitchOnDisabled:
          controlword.setStateTransition2();
          break;
        case DriveState::ReadyToSwitchOn:
          controlword.setStateTransition3();
          break;
        case DriveState::SwitchedOn:
          controlword.setStateTransition4();
          break;
        case DriveState::OperationEnabled:
          controlword.setStateTransition11();
          break;
        case DriveState::QuickStopActive:
          MELO_ERROR_STREAM("[elmo_ethercat_sdk:Elmo::getNextStateTransitionControlword] "
                            << "drive state has already been reached for '" << name_ << "'");
          break;
        case DriveState::Fault:
          controlword.setStateTransition15();
          break;
        default:
          MELO_ERROR_STREAM("[elmo_ethercat_sdk:Elmo::getNextStateTransitionControlword] "
                            << "PDO state transition not implemented for '" << name_ << "'");
      }
      break;

    default:
      MELO_ERROR_STREAM("[elmo_ethercat_sdk:Elmo::getNextStateTransitionControlword] "
                        << "PDO state cannot be reached for '" << name_ << "'");
  }

  return controlword;
}

void Elmo::autoConfigurePdoSizes() {
  auto pdoSizes = bus_->getHardwarePdoSizes(static_cast<uint16_t>(address_));
  pdoInfo_.rxPdoSize_ = pdoSizes.first;
  pdoInfo_.txPdoSize_ = pdoSizes.second;
}

uint16_t Elmo::getTxPdoSize() {
  return pdoInfo_.txPdoSize_;
}

uint16_t Elmo::getRxPdoSize() {
  return pdoInfo_.rxPdoSize_;
}

void Elmo::engagePdoStateMachine() {
  // get the current state
  // read value
  DriveState currentDriveState;
  {
    std::lock_guard<std::mutex> readingLock(readingMutex_);
    currentDriveState = reading_.getDriveState();
  }

  // check if the state change already was successful:
  {
    std::lock_guard<std::mutex> targetDriveStateLock(targetStateMutex_);
    if (currentDriveState == targetDriveState_) {
      numberOfSuccessfulTargetStateReadings_++;  // todo remove.
      if (numberOfSuccessfulTargetStateReadings_ >= configuration_.minNumberOfSuccessfulTargetStateReadings) {
        // disable the state machine
        conductStateChange_ = false;
        numberOfSuccessfulTargetStateReadings_ = 0;
        stateChangeSuccessful_ = true;
        cvDriveStateMachineSync_.notify_one();
      }
    } else if (prevDriveState_ != currentDriveState) {
      prevDriveState_ = currentDriveState;
      controlword_ = getNextStateTransitionControlword(targetDriveState_, currentDriveState);
    }
  }
}

}  // namespace elmo
