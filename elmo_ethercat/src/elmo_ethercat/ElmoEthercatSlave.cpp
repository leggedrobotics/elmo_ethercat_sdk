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

#define _USE_MATH_DEFINES
#include <cmath>
#include <elmo_ethercat/ElmoEthercatSlave.hpp>
#include <elmo_ethercat/ObjectDictionary.hpp>

namespace elmo {
namespace ethercat {

// constructor
ElmoEthercatSlave::ElmoEthercatSlave(std::string name, soem_interface::EthercatBusBase* bus, uint32_t address,
                                     const RxPdoTypeEnum rxPdoTypeEnum, const TxPdoTypeEnum txPdoTypeEnum)
    : soem_interface::EthercatSlaveBase(bus, address),
      name_(std::move(name)),
      rxPdoTypeEnum_(rxPdoTypeEnum),
      txPdoTypeEnum_(txPdoTypeEnum),
      currentRxPdoTypeEnum_(RxPdoTypeEnum::NA),
      currentTxPdoTypeEnum_(TxPdoTypeEnum::NA) {
  // some further initialization
  controlword_.setAllFalse();
  // TODO(duboisf) set the correct PDO sizes as soon as the PDOs are definitive
  // For now: start the bus with the sizeCheck parameter set to false
  // When loading a configuration these values get overwritten anyway.
  pdoInfo_.rxPdoSize_ = 10;
  pdoInfo_.txPdoSize_ = 10;
}

bool ElmoEthercatSlave::startup() {
  // sets currentPdoTypeEnum_ to pdoTypeEnum_
  return (static_cast<int>(configureRxPdo(rxPdoTypeEnum_)) & static_cast<int>(configureTxPdo(txPdoTypeEnum_))) != 0;
}

void ElmoEthercatSlave::setCommand(const Command& command) {
  // lock guard for thread safety
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  command_ = command;

  /*!
   * set unit conversion factors
   */
  command_.setPositionFactorRadToInteger(static_cast<double>(configuration_.getPositionEncoderResolution()) / (2.0 * M_PI));

  command_.setVelocityFactorRadPerSecToIntegerPerSec(static_cast<double>(configuration_.getPositionEncoderResolution()) / (2.0 * M_PI));

  double currentFactorAToInt = 1000.0 / configuration_.getMotorRatedCurrentA();
  command_.setCurrentFactorAToInteger(currentFactorAToInt);
  command_.setTorqueFactorNmToInteger(currentFactorAToInt / configuration_.getMotorConstant() / configuration_.getGearRatio());

  /*!
   * Set the maximum current and corresponding torque.
   * Used for PDO communication.
   */
  command_.setMaxCurrent(configuration_.getMaxCurrentA());
  command_.setMaxTorque(configuration_.getMaxCurrentA() * configuration_.getMotorConstant() * configuration_.getGearRatio());

  /*!
   * do not change the integer values if raw inputs are used
   */
  command_.setUseRawCommands(configuration_.getUseRawCommands());
  /*!
   * convert from floating point to integer values (including scaling)
   */
  command_.doUnitConversion();

  /*!
   * Set a new mode of operation if all necessary conditions are met (correct
   * PDOs, enabling mode switching in config file)
   */
  if (allowModeChange_) {
    modeOfOperation_ = command.getModeOfOperation();
  }
}

Configuration ElmoEthercatSlave::getActiveConfiguration() {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return configuration_;
}

void ElmoEthercatSlave::getReading(Reading& reading) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  reading = reading_;
  // TODO(duboisf) implement setting of unit conversion factors
}

void ElmoEthercatSlave::updateRead() {
  // locking for thread safety
  std::lock_guard<std::recursive_mutex> lock(mutex_);

  // TODO(duboisf): implement some sort of time stamp
  switch (currentTxPdoTypeEnum_) {
    case TxPdoTypeEnum::TxPdoStandard: {
      TxPdoStandard txPdo{};
      // reading from the bus
      bus_->readTxPdo(address_, txPdo);
      reading_.setActualPosition(txPdo.actualPosition_);
      reading_.setDigitalInputs(txPdo.digitalInputs_);
      reading_.setActualVelocity(txPdo.actualVelocity_);
      reading_.setStatusword(txPdo.statusword_);
      reading_.setAnalogInput(txPdo.analogInput_);
      reading_.setActualCurrent(txPdo.actualCurrent_);
      reading_.setBusVoltage(txPdo.busVoltage_);
    } break;
    case TxPdoTypeEnum::TxPdoCST: {
      TxPdoCST txPdo{};
      // reading from the bus
      bus_->readTxPdo(address_, txPdo);
      reading_.setActualPosition(txPdo.actualPosition_);
      reading_.setActualCurrent(txPdo.actualTorque_);  /// torque readings are actually current readings,
                                                       /// the conversion is handled later
      reading_.setStatusword(txPdo.statusword_);
      reading_.setActualVelocity(txPdo.actualVelocity_);
    } break;

    default:
      if (printErrorMessage_) MELO_ERROR_STREAM("Unsupported Tx Pdo type, not able to read (" << getName() << "::updateRead())");
      reading_.addError(ErrorType::TxPdoTypeError);
  }

  // set the hasRead_ variable to true since a nes reading was read
  if (!hasRead_) {
    hasRead_ = true;
  }

  /*!
   * Check whether the state has changed to "FAULT"
   */
  if (reading_.getDriveState() == DriveState::Fault) {
    uint16_t fault;
    if (sendSdoReadUInt16(OD_INDEX_ERROR_CODE, 0, false, fault)) {  // TODO(duboisf) check
      reading_.addFault(fault);
    } else {
      reading_.addError(ErrorType::ErrorReadingError);
    }
  }
}

void ElmoEthercatSlave::updateWrite() {
  /*! locking the mutex_
   * This is necessary since "updateWrite" is called from an external thread
   */
  std::lock_guard<std::recursive_mutex> lock(mutex_);

  /*!
   * Check if the Mode of Operation has been set properly
   */
  if (modeOfOperation_ == ModeOfOperationEnum::NA) {
    reading_.addError(ErrorType::ModeOfOperationError);
    if (printErrorMessage_)
      MELO_ERROR_STREAM(
          "The Mode of Operation has not been set. PDO "
          "communication is not possible! (Name:\t"
          << getName() << ")");
    return;
  }

  /*!
   * engage the state machine if a state change is requested
   */
  if (conductStateChange_ && hasRead_) {
    engagePdoStateMachine();
  }

  switch (currentRxPdoTypeEnum_) {
    case RxPdoTypeEnum::RxPdoStandard: {
      RxPdoStandard rxPdo{};
      rxPdo.targetPosition_ = command_.getTargetPositionRaw();
      rxPdo.targetVelocity_ = command_.getTargetVelocityRaw();
      rxPdo.targetTorque_ = command_.getTargetTorqueRaw();
      rxPdo.maxTorque_ = command_.getMaxTorqueRaw();
      rxPdo.modeOfOperation_ = static_cast<int8_t>(modeOfOperation_);
      rxPdo.torqueOffset_ = command_.getTorqueOffsetRaw();
      rxPdo.controlWord_ = controlword_.getRawControlword();

      // actually writing to the hardware
      bus_->writeRxPdo(address_, rxPdo);
    } break;
    case RxPdoTypeEnum::RxPdoCST: {
      RxPdoCST rxPdo{};
      rxPdo.targetTorque_ = command_.getTargetTorqueRaw();
      rxPdo.modeOfOperation_ = static_cast<int8_t>(modeOfOperation_);
      rxPdo.controlWord_ = controlword_.getRawControlword();

      // actually writing to the hardware
      bus_->writeRxPdo(address_, rxPdo);
    } break;

    default:
      MELO_ERROR_STREAM("Unsupported Rx Pdo type, not able to write (" << getName() << "::updateWrite())");
      reading_.addError(ErrorType::RxPdoTypeError);
  }
}

void ElmoEthercatSlave::engagePdoStateMachine() {
  // locking the mutex
  std::lock_guard<std::recursive_mutex> lock(mutex_);

  // elapsed time since the last new controlword
  auto microsecondsSinceChange =
      (std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - driveStateChangeTimePoint_)).count();

  // get the current state
  // since we wait until "hasRead" is true, this is guaranteed to be a newly
  // read value
  const DriveState currentDriveState = reading_.getDriveState();

  // check if the state change already vas successful:
  if (currentDriveState == targetDriveState_) {
    numberOfSuccessfulTargetStateReadings_++;
    if (numberOfSuccessfulTargetStateReadings_ >= minNumberOfSuccessfulTargetStateReadings_) {
      // disable the state machine
      conductStateChange_ = false;
      numberOfSuccessfulTargetStateReadings_ = 0;
      stateChangeSuccessful_ = true;
      return;
    }
  } else if (microsecondsSinceChange > driveStateChangeMinTimeout_) {
    // get the next controlword from the state machine
    controlword_ = getNextStateTransitionControlword(targetDriveState_, currentDriveState);
    driveStateChangeTimePoint_ = std::chrono::steady_clock::now();
  }

  // set the "hasRead" variable to false such that there will definitely be a
  // new reading when this method is called again
  hasRead_ = false;
}

bool ElmoEthercatSlave::setDriveStateViaPdo(const DriveState& targetDriveState, const bool waitForDriveStateChange) {
  bool success;
  /*!
   * locking the mutex_
   * This is not done with a lock_guard here because during the waiting time the
   * mutex_ must be unlocked periodically such that PDO writing (and thus state
   * changes) may occur at all!
   */
  mutex_.lock();

  // reset the "stateChangeSuccessful_" flag to false such that a new successful
  // state change can be detected
  stateChangeSuccessful_ = false;

  // make the state machine realize that a state change will have to happen
  conductStateChange_ = true;

  // overwrite the target drive state
  targetDriveState_ = targetDriveState;

  // set the hasRead flag to false such that at least one new reading will be
  // available when starting the state change
  hasRead_ = false;

  // set the time point of the last pdo change to now
  driveStateChangeTimePoint_ = std::chrono::steady_clock::now();

  // set a temporary time point to prevent getting caught in an infinite loop
  auto driveStateChangeStartTimePoint = std::chrono::steady_clock::now();

  // return if no waiting is requested
  if (!waitForDriveStateChange) {
    // unlock the mutex
    mutex_.unlock();
    // return true if no waiting is requested
    return true;
  }

  // Wait for the state change to be successful
  // during the waiting time the mutex MUST be unlocked!

  while (true) {
    // break loop as soon as the state change was successful
    if (stateChangeSuccessful_) {
      success = true;
      break;
    }

    // break the loop if the state change takes too long
    // this prevents a freezing of the end user's program if the hardware is not
    // able to change it's state.
    if ((std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - driveStateChangeStartTimePoint)).count() >
        driveStateChangeMaxTimeout_) {
      break;
    }
    // unlock the mutex during sleep time
    mutex_.unlock();
    usleep(1000);
    // lock the mutex to be able to check the success flag
    mutex_.lock();
  }
  // unlock the mutex one last time
  mutex_.unlock();
  return success;
}

Controlword ElmoEthercatSlave::getNextStateTransitionControlword(const DriveState& requestedDriveState,
                                                                 const DriveState& currentDriveState) {
  // no global lock guard for the entire method
  // it is only needed if an error occurs
  // This optimizes the performance
  Controlword controlword;
  controlword.setAllFalse();
  switch (requestedDriveState) {
    case DriveState::SwitchOnDisabled:
      switch (currentDriveState) {
        case DriveState::SwitchOnDisabled:
          if (printErrorMessage_)
            MELO_WARN_STREAM(
                "The requested drive state has been reached! Do not "
                "use this controlword!!");
          // locking the mutex_ because the reading_ object is altered
          {
            std::lock_guard<std::recursive_mutex> lock(mutex_);
            reading_.addError(ErrorType::PdoStateTransitionError);
          }
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
          if (printErrorMessage_) MELO_ERROR_STREAM("This Pdo state Transition has not yet been implemented.");
          {
            std::lock_guard<std::recursive_mutex> lock(mutex_);
            reading_.addError(ErrorType::PdoStateTransitionError);
          }
      }
      break;

    case DriveState::ReadyToSwitchOn:
      switch (currentDriveState) {
        case DriveState::SwitchOnDisabled:
          controlword.setStateTransition2();
          break;
        case DriveState::ReadyToSwitchOn:
          if (printErrorMessage_)
            MELO_WARN_STREAM(
                "The requested drive state has been reached! Do not "
                "use this controlword!!");
          {
            std::lock_guard<std::recursive_mutex> lock(mutex_);
            reading_.addError(ErrorType::PdoStateTransitionError);
          }
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
          if (printErrorMessage_) MELO_ERROR_STREAM("This state Transition has not yet been implemented.");
          {
            std::lock_guard<std::recursive_mutex> lock(mutex_);
            reading_.addError(ErrorType::PdoStateTransitionError);
          }
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
          if (printErrorMessage_)
            MELO_WARN_STREAM(
                "The requested drive state has been reached! Do not "
                "use this controlword!!");
          {
            std::lock_guard<std::recursive_mutex> lock(mutex_);
            reading_.addError(ErrorType::PdoStateTransitionError);
          }
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
          if (printErrorMessage_) MELO_ERROR_STREAM("This state Transition has not yet been implemented.");
          {
            std::lock_guard<std::recursive_mutex> lock(mutex_);
            reading_.addError(ErrorType::PdoStateTransitionError);
          }
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
          if (printErrorMessage_)
            MELO_WARN_STREAM(
                "The requested drive state has been reached! Do not "
                "use this controlword!!");
          {
            std::lock_guard<std::recursive_mutex> lock(mutex_);
            reading_.addError(ErrorType::PdoStateTransitionError);
          }
          break;
        case DriveState::QuickStopActive:
          controlword.setStateTransition12();
          break;
        case DriveState::Fault:
          controlword.setStateTransition15();
          break;
        default:
          if (printErrorMessage_) MELO_ERROR_STREAM("This state Transition has not yet been implemented.");
          {
            std::lock_guard<std::recursive_mutex> lock(mutex_);
            reading_.addError(ErrorType::PdoStateTransitionError);
          }
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
          if (printErrorMessage_)
            MELO_WARN_STREAM(
                "The requested drive state has been reached! Do not "
                "use this controlword!!");
          {
            std::lock_guard<std::recursive_mutex> lock(mutex_);
            reading_.addError(ErrorType::PdoStateTransitionError);
          }
          break;
        case DriveState::Fault:
          controlword.setStateTransition15();
          break;
        default:
          if (printErrorMessage_) MELO_ERROR_STREAM("This state Transition has not yet been implemented.");
          {
            std::lock_guard<std::recursive_mutex> lock(mutex_);
            reading_.addError(ErrorType::PdoStateTransitionError);
          }
      }
      break;

    default:
      if (printErrorMessage_) MELO_ERROR_STREAM("This state cannot be reached!");
      {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        reading_.addError(ErrorType::PdoStateTransitionError);
      }
  }

  return controlword;
}

void ElmoEthercatSlave::shutdown() {
  // go back to INIT state
  // don't wait for the state since it's not that important
  bus_->setState(EC_STATE_INIT, address_);
}

bool ElmoEthercatSlave::configureRxPdo(const RxPdoTypeEnum rxPdoTypeEnum) {
  // lock guard
  std::lock_guard<std::recursive_mutex> lock(mutex_);

  // invalid Type
  if (rxPdoTypeEnum == RxPdoTypeEnum::NA) {
    if (printErrorMessage_) MELO_ERROR_STREAM("Invalid EtherCAT Rx PDO Type.");
    reading_.addError(ErrorType::RxPdoTypeError);
    return false;
  }

  // the types already coincide
  if (rxPdoTypeEnum == getCurrentRxPdoTypeEnum()) {
    return true;
  }

  // set the current Pdo type
  else {
    currentRxPdoTypeEnum_ = rxPdoTypeEnum;
    return true;
  }
}

bool ElmoEthercatSlave::configureTxPdo(const TxPdoTypeEnum txPdoTypeEnum) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);

  // invalid Type
  if (txPdoTypeEnum == TxPdoTypeEnum::NA) {
    // print Warning ( in message_logger )
    if (printErrorMessage_) MELO_ERROR_STREAM("Invalid EtherCAT Tx PDO Type.");
    reading_.addError(ErrorType::TxPdoTypeError);
    return false;
  }

  // the types already coincide
  if (txPdoTypeEnum == getCurrentTxPdoTypeEnum()) {
    return true;
  }

  // set the current Pdo type
  else {
    currentTxPdoTypeEnum_ = txPdoTypeEnum;
    return true;
  }
}

void ElmoEthercatSlave::configureElmoEthercatSlave(const Configuration& configuration) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  driveStateChangeMinTimeout_ = configuration.getDriveStateChangeMinTimeout();

  driveStateChangeMaxTimeout_ = configuration.getDriveStateChangeMaxTimeout();

  minNumberOfSuccessfulTargetStateReadings_ = configuration.getMinNumberOfSuccessfulTargetStateReadings();

  printErrorMessage_ = configuration.getPrintErrorMessageElmoEthercatSlave();

  modeOfOperation_ = configuration.getModeOfOperation();

  reading_.configureReading(configuration);

  configuration_ = configuration;

  /// Check whether changing the mode will be allowed
  allowModeChange_ = true;
  allowModeChange_ &= configuration_.getUseMultipleModeOfOperations();
  allowModeChange_ &= (configuration_.getRxPdoTypeEnum() == RxPdoTypeEnum::RxPdoStandard);
  allowModeChange_ &= (configuration_.getTxPdoTypeEnum() == TxPdoTypeEnum::TxPdoStandard);
}

void ElmoEthercatSlave::autoConfigurePdoSizes() {
  auto pdoSizes = bus_->getHardwarePdoSizes(static_cast<uint16_t>(address_));
  setRxPdoSize(pdoSizes.first);
  setTxPdoSize(pdoSizes.second);
}

// because of some errors:
uint16_t ElmoEthercatSlave::getTxPdoSize() {
  return pdoInfo_.txPdoSize_;
}
uint16_t ElmoEthercatSlave::getRxPdoSize() {
  return pdoInfo_.rxPdoSize_;
}
void ElmoEthercatSlave::setTxPdoSize(uint16_t size) {
  pdoInfo_.txPdoSize_ = size;
}
void ElmoEthercatSlave::setRxPdoSize(uint16_t size) {
  pdoInfo_.rxPdoSize_ = size;
}

void ElmoEthercatSlave::addErrorToReading(const ErrorType& errorType) {
  // locking the mutex_ to prevent data races
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  reading_.addError(errorType);
}

}  // namespace ethercat
}  // namespace elmo
