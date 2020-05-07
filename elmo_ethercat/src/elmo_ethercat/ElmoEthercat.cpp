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

#include <cmath>

#include <elmo_ethercat/ConfigurationParser.hpp>
#include <elmo_ethercat/ElmoEthercat.hpp>
#include <elmo_ethercat/ObjectDictionary.hpp>

namespace elmo {
namespace ethercat {

void ElmoEthercat::startupWithCommunication() {
  slavePtr_->startup();
}

void ElmoEthercat::updateSendStagedCommand() {
  if (!commandIsStaged_) {
    return;  // return if no command is currently staged
  }

  // lock the staged Command object
  std::lock_guard<std::recursive_mutex> lock(stagedCommandMutex_);

  // TODO(duboisf): implement time handling here and/or in Command object
  // Decide if the Command is young enough to be sent

  // transmitting the command to the slave
  slavePtr_->setCommand(stagedCommand_);
  // no command is staged anymore
  commandIsStaged_ = false;

  // TODO(duboisf): check necessety of auto staging the last command
}

Reading ElmoEthercat::getReading() const {
  std::lock_guard<std::recursive_mutex> lock(readingMutex_);
  return reading_;
}

void ElmoEthercat::getReading(Reading& reading) const {
  std::lock_guard<std::recursive_mutex> lock(readingMutex_);
  reading = reading_;
}

bool ElmoEthercat::loadConfigNode(YAML::Node configNode) {
  Configuration configuration;
  ConfigurationParser configParser(configNode);
  configuration = configParser.getConfiguration();
  return configureAll(configuration);
}

bool ElmoEthercat::loadConfigFile(const std::string& fileName) {
  Configuration configuration;
  ConfigurationParser configParser(fileName);       /// The file is being parsed...
  configuration = configParser.getConfiguration();  /// get the parsed configuration parameters
  return configureAll(configuration);
}

bool ElmoEthercat::configureAll(Configuration& configuration) {
  bool success = true;
  /*!
   * Check whether a custom rated current value shall be used
   */
  if (configuration.getMotorRatedCurrentA() == 0.0) {
    /*!
     * Read the default value automatically set by the servo drive
     */
    uint32_t motorRatedCurrent;
    success &= sendSdoRead(OD_INDEX_MOTOR_RATED_CURRENT, 0, false, motorRatedCurrent);
    configuration.setMotorRatedCurrentA(static_cast<double>(motorRatedCurrent) / 1000.0);
  }

  /// Configure the Elmo object
  configureElmoEthercat(configuration);

  /// Configure the The ElmoEthercatSlave pointed to by SlavePtr_
  slavePtr_->configureElmoEthercatSlave(configuration);

  /*!
   * configure the connected Elmo servo drive
   * This could go wrong because it involves writing / reading SDOs
   * We return the success value of ElmoEthercat::configureHardware
   * Note: Checking the reading objects for errors after calling this method is
   * recommended.
   */
  success &= configureHardware(configuration);

  if (printErrorMessage_) MELO_INFO_STREAM("Configuration was:\t" << (success ? "successful." : "not successful."));
  return success;
}

void ElmoEthercat::configureElmoEthercat(const Configuration& configuration) {
  sdoVerifyTimeout_ = configuration.getSdoVerifyTimeout();
  printErrorMessage_ = configuration.getPrintErrorMessageElmoEthercat();
  reading_.configureReading(configuration);
}

bool ElmoEthercat::configureHardware(const Configuration& configuration) {
  bool success = true;

  success &= setDriveStateViaSdo(DriveState::ReadyToSwitchOn);

  // map the pdos
  success &= mapPdos(configuration.getRxPdoTypeEnum(), configuration.getTxPdoTypeEnum());

  // set mode of operation
  success &=
      sdoVerifyWrite(OD_INDEX_MODES_OF_OPERATION, 0, false, static_cast<int8_t>(configuration.getModeOfOperation()), sdoVerifyTimeout_);

  // just to be on the safe side: set the correct PdoSizes
  slavePtr_->autoConfigurePdoSizes();

  /*!
   * Write the requested motor rated current
   * The motor rated torque is set to the same value since current torque
   * conversion is handled in the library and not on the hardware
   */
  auto motorRatedCurrent = static_cast<uint32_t>(round(1000.0 * configuration.getMotorRatedCurrentA()));  // [mA]
  success &= sdoVerifyWrite(OD_INDEX_MOTOR_RATED_CURRENT, 0, false, motorRatedCurrent);
  success &= sdoVerifyWrite(OD_INDEX_MOTOR_RATED_TORQUE, 0, false, motorRatedCurrent);

  /*!
   * Write the maximum current
   */
  auto maxCurrent = static_cast<uint16_t>(floor(1000.0 * configuration.getMaxCurrentA()));  // [mA]
  success &= sdoVerifyWrite(OD_INDEX_MAX_CURRENT, 0, false, maxCurrent);

  if (!success) {
    if (printErrorMessage_) MELO_WARN_STREAM("Configuration of the elmo hardware not successful");

    /*!
     * Add the error to the slave's reading
     * This is necessary to make sure that the Error history is not cleared once
     * new readings from the slave are copied into this object during PDO
     * communication
     */
    slavePtr_->addErrorToReading(ErrorType::ConfigurationError);
    /*!
     * Add the error to this object's Reading object such that error checking ca
     * be done with ElmoEthercat::getReading() without having to call
     * ElmoEthercat::UpdateProcessReading()
     */
    reading_.addError(ErrorType::ConfigurationError);
  }
  return success;
}

Configuration ElmoEthercat::getActiveConfiguration() {
  return slavePtr_->getActiveConfiguration();
}

bool ElmoEthercat::getStatuswordViaSdo(Statusword& statusword) {
  uint16_t statuswordValue = 0;
  bool success = sendSdoRead(OD_INDEX_STATUSWORD, 0, false, statuswordValue);
  statusword.setFromRawStatusword(statuswordValue);
  return success;
}

bool ElmoEthercat::setControlwordViaSdo(Controlword& controlword) {
  return sendSdoWrite(OD_INDEX_CONTROLWORD, 0, false, controlword.getRawControlword());
}

void ElmoEthercat::updateProcessReading() {
  std::lock_guard<std::recursive_mutex> lock(readingMutex_);

  // read the Pdo
  slavePtr_->getReading(reading_);

  // TODO(duboisf): statuswords, callbacks
}

bool ElmoEthercat::setDriveStateViaSdo(const DriveState& driveState) {
  bool success = true;

  // get the actual current drive state
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
          if (printErrorMessage_) MELO_ERROR_STREAM("This state Transition has not yet been implemented.");
          /*!
           * Add the error to the slave's reading
           * This is necessary to make sure that the Error history is not cleared
           * once new readings from the slave are copied into this object during PDO
           * communication
           */
          slavePtr_->addErrorToReading(ErrorType::SdoStateTransitionError);
          /*!
           * Add the error to this object's Reading object such that error checking
           * ca be done with ElmoEthercat::getReading() without having to call
           * ElmoEthercat::UpdateProcessReading()
           */
          reading_.addError(ErrorType::SdoStateTransitionError);
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
          if (printErrorMessage_) MELO_ERROR_STREAM("This state Transition has not yet been implemented.");
          /*!
           * Add the error to the slave's reading
           * This is necessary to make sure that the Error history is not cleared
           * once new readings from the slave are copied into this object during PDO
           * communication
           */
          slavePtr_->addErrorToReading(ErrorType::SdoStateTransitionError);
          /*!
           * Add the error to this object's Reading object such that error checking
           * ca be done with ElmoEthercat::getReading() without having to call
           * ElmoEthercat::UpdateProcessReading()
           */
          reading_.addError(ErrorType::SdoStateTransitionError);
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
          if (printErrorMessage_) MELO_ERROR_STREAM("This state Transition has not yet been implemented.");
          /*!
           * Add the error to the slave's reading
           * This is necessary to make sure that the Error history is not cleared
           * once new readings from the slave are copied into this object during PDO
           * communication
           */
          slavePtr_->addErrorToReading(ErrorType::SdoStateTransitionError);
          /*!
           * Add the error to this object's Reading object such that error checking
           * ca be done with ElmoEthercat::getReading() without having to call
           * ElmoEthercat::UpdateProcessReading()
           */
          reading_.addError(ErrorType::SdoStateTransitionError);
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
          if (printErrorMessage_) MELO_ERROR_STREAM("This state Transition has not yet been implemented.");
          /*!
           * Add the error to the slave's reading
           * This is necessary to make sure that the Error history is not cleared
           * once new readings from the slave are copied into this object during PDO
           * communication
           */
          slavePtr_->addErrorToReading(ErrorType::SdoStateTransitionError);
          /*!
           * Add the error to this object's Reading object such that error checking
           * ca be done with ElmoEthercat::getReading() without having to call
           * ElmoEthercat::UpdateProcessReading()
           */
          reading_.addError(ErrorType::SdoStateTransitionError);
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
          if (printErrorMessage_) MELO_ERROR_STREAM("This state Transition has not yet been implemented.");
          /*!
           * Add the error to the slave's reading
           * This is necessary to make sure that the Error history is not cleared
           * once new readings from the slave are copied into this object during PDO
           * communication
           */
          slavePtr_->addErrorToReading(ErrorType::SdoStateTransitionError);
          /*!
           * Add the error to this object's Reading object such that error checking
           * ca be done with ElmoEthercat::getReading() without having to call
           * ElmoEthercat::UpdateProcessReading()
           */
          reading_.addError(ErrorType::SdoStateTransitionError);
          success = false;
      }
      break;

    default:
      if (printErrorMessage_) MELO_ERROR_STREAM("This state has not yet been implemented.");
      /*!
       * Add the error to the slave's reading
       * This is necessary to make sure that the Error history is not cleared once
       * new readings from the slave are copied into this object during PDO
       * communication
       */
      slavePtr_->addErrorToReading(ErrorType::SdoStateTransitionError);
      /*!
       * Add the error to this object's Reading object such that error checking ca
       * be done with ElmoEthercat::getReading() without having to call
       * ElmoEthercat::UpdateProcessReading()
       */
      reading_.addError(ErrorType::SdoStateTransitionError);
      success = false;
  }

  return success;
}

bool ElmoEthercat::stateTransitionViaSdo(const StateTransition& stateTransition) {
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
      if (printErrorMessage_) MELO_ERROR_STREAM("This state Transition has not yet been implemented.");
      /*!
       * Add the error to the slave's reading
       * This is necessary to make sure that the Error history is not cleared once
       * new readings from the slave are copied into this object during PDO
       * communication
       */
      slavePtr_->addErrorToReading(ErrorType::SdoStateTransitionError);
      /*!
       * Add the error to this object's Reading object such that error checking ca
       * be done with ElmoEthercat::getReading() without having to call
       * ElmoEthercat::UpdateProcessReading()
       */
      reading_.addError(ErrorType::SdoStateTransitionError);
      return false;
  }
}

bool ElmoEthercat::setDriveStateViaPdo(const DriveState& driveState, const bool waitForState) {
  return slavePtr_->setDriveStateViaPdo(driveState, waitForState);
}

bool ElmoEthercat::getReady(bool waitForOperationEnabled) {
  // conduct a PDO state change
  return slavePtr_->setDriveStateViaPdo(DriveState::OperationEnabled, waitForOperationEnabled);
}

void ElmoEthercat::shutdown() {
  slavePtr_->setDriveStateViaPdo(DriveState::SwitchOnDisabled);
}

uint16_t ElmoEthercat::getAddressOfSlave() const {
  return slavePtr_->getAddress();
}

std::string ElmoEthercat::getNameOfSlave() const {
  return slavePtr_->getName();
}

// TODO(duboisf): put the pdo addresses in the object dictionary
bool ElmoEthercat::mapPdos(RxPdoTypeEnum rxPdoTypeEnum, TxPdoTypeEnum txPdoTypeEnum) {
  bool rxSuccess = true;
  switch (rxPdoTypeEnum) {
    case RxPdoTypeEnum::RxPdoStandard:
      usleep(sdoVerifyTimeout_);
      rxSuccess &= sdoVerifyWrite(OD_INDEX_RX_PDO_ASSIGNMENT, 0, false, static_cast<uint8_t>(0), sdoVerifyTimeout_);
      usleep(sdoVerifyTimeout_);
      rxSuccess &= sdoVerifyWrite(OD_INDEX_RX_PDO_ASSIGNMENT, 1, false, static_cast<uint16_t>(0x1605), sdoVerifyTimeout_);
      usleep(sdoVerifyTimeout_);
      rxSuccess &= sdoVerifyWrite(OD_INDEX_RX_PDO_ASSIGNMENT, 2, false, static_cast<uint16_t>(0x1618), sdoVerifyTimeout_);
      usleep(sdoVerifyTimeout_);
      rxSuccess &= sdoVerifyWrite(OD_INDEX_RX_PDO_ASSIGNMENT, 0, false, static_cast<uint8_t>(2), sdoVerifyTimeout_);
      usleep(sdoVerifyTimeout_);
      break;
    case RxPdoTypeEnum::RxPdoCST:
      usleep(sdoVerifyTimeout_);
      rxSuccess &= sdoVerifyWrite(OD_INDEX_RX_PDO_ASSIGNMENT, 0, false, static_cast<uint8_t>(0), sdoVerifyTimeout_);
      usleep(sdoVerifyTimeout_);
      rxSuccess &= sdoVerifyWrite(OD_INDEX_RX_PDO_ASSIGNMENT, 1, false, static_cast<uint16_t>(0x1602), sdoVerifyTimeout_);
      usleep(sdoVerifyTimeout_);
      rxSuccess &= sdoVerifyWrite(OD_INDEX_RX_PDO_ASSIGNMENT, 2, false, static_cast<uint16_t>(0x160b), sdoVerifyTimeout_);
      rxSuccess &= sdoVerifyWrite(OD_INDEX_RX_PDO_ASSIGNMENT, 0, false, static_cast<uint8_t>(2), sdoVerifyTimeout_);
      usleep(sdoVerifyTimeout_);
      break;

    case RxPdoTypeEnum::NA:
      if (printErrorMessage_) MELO_ERROR_STREAM("Cannot map RxPdo, PdoType not configured properly");
      slavePtr_->addErrorToReading(ErrorType::PdoMappingError);
      reading_.addError(ErrorType::PdoMappingError);

      rxSuccess = false;
      break;
    default:  // just to be prevent errors when a new type has been defined but not
              // yet implemented
      if (printErrorMessage_) MELO_ERROR_STREAM("Cannot map RxPdo, PdoType not configured properly");
      slavePtr_->addErrorToReading(ErrorType::PdoMappingError);
      reading_.addError(ErrorType::PdoMappingError);

      rxSuccess = false;
      break;
  }

  bool txSuccess = true;
  switch (txPdoTypeEnum) {
    case TxPdoTypeEnum::TxPdoStandard:
      txSuccess &= sdoVerifyWrite(OD_INDEX_TX_PDO_ASSIGNMENT, 0, false, static_cast<uint8_t>(0), sdoVerifyTimeout_);
      usleep(sdoVerifyTimeout_);
      txSuccess &= sdoVerifyWrite(OD_INDEX_TX_PDO_ASSIGNMENT, 1, false, static_cast<uint16_t>(0x1a03), sdoVerifyTimeout_);
      usleep(sdoVerifyTimeout_);
      txSuccess &= sdoVerifyWrite(OD_INDEX_TX_PDO_ASSIGNMENT, 2, false, static_cast<uint16_t>(0x1a1d), sdoVerifyTimeout_);
      usleep(sdoVerifyTimeout_);
      txSuccess &= sdoVerifyWrite(OD_INDEX_TX_PDO_ASSIGNMENT, 3, false, static_cast<uint16_t>(0x1a1f), sdoVerifyTimeout_);
      usleep(sdoVerifyTimeout_);
      txSuccess &= sdoVerifyWrite(OD_INDEX_TX_PDO_ASSIGNMENT, 4, false, static_cast<uint16_t>(0x1a18), sdoVerifyTimeout_);
      usleep(sdoVerifyTimeout_);
      txSuccess &= sdoVerifyWrite(OD_INDEX_TX_PDO_ASSIGNMENT, 0, false, static_cast<uint8_t>(4), sdoVerifyTimeout_);
      usleep(sdoVerifyTimeout_);
      break;

    case TxPdoTypeEnum::TxPdoCST:
      txSuccess &= sdoVerifyWrite(OD_INDEX_TX_PDO_ASSIGNMENT, 0, false, static_cast<uint8_t>(0), sdoVerifyTimeout_);
      usleep(sdoVerifyTimeout_);
      txSuccess &= sdoVerifyWrite(OD_INDEX_TX_PDO_ASSIGNMENT, 1, false, static_cast<uint16_t>(0x1a02), sdoVerifyTimeout_);
      usleep(sdoVerifyTimeout_);
      txSuccess &= sdoVerifyWrite(OD_INDEX_TX_PDO_ASSIGNMENT, 2, false, static_cast<uint16_t>(0x1a11), sdoVerifyTimeout_);
      usleep(sdoVerifyTimeout_);
      txSuccess &= sdoVerifyWrite(OD_INDEX_TX_PDO_ASSIGNMENT, 0, false, static_cast<uint8_t>(2), sdoVerifyTimeout_);
      usleep(sdoVerifyTimeout_);
      break;

    case TxPdoTypeEnum::NA:
      if (printErrorMessage_) MELO_ERROR_STREAM("Cannot map TxPdo, PdoType not configured properly");
      slavePtr_->addErrorToReading(ErrorType::TxPdoMappingError);
      reading_.addError(ErrorType::TxPdoMappingError);

      txSuccess = false;
      break;
    default:  // if any case was forgotten
      if (printErrorMessage_) MELO_ERROR_STREAM("Cannot map TxPdo, PdoType not configured properly");
      slavePtr_->addErrorToReading(ErrorType::TxPdoMappingError);
      reading_.addError(ErrorType::TxPdoMappingError);

      txSuccess = false;
      break;
  }

  if (rxSuccess) {
    rxSuccess &= slavePtr_->configureRxPdo(rxPdoTypeEnum);
  }
  if (txSuccess) {
    txSuccess &= slavePtr_->configureTxPdo(txPdoTypeEnum);
  }

  return (static_cast<int>(txSuccess) & static_cast<int>(rxSuccess)) != 0;
}

// Sdo write template specialization
template <>
bool ElmoEthercat::sendSdoWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess, const int8_t value) {
  bool success = slavePtr_->sendSdoWriteInt8(index, subindex, completeAccess, value);
  if (!success) {
    std::lock_guard<std::recursive_mutex> lock(readingMutex_);
    reading_.addError(ErrorType::SdoWriteError);
  }
  return success;
}

template <>
bool ElmoEthercat::sendSdoWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess, const int16_t value) {
  bool success = slavePtr_->sendSdoWriteInt16(index, subindex, completeAccess, value);
  if (!success) {
    std::lock_guard<std::recursive_mutex> lock(readingMutex_);
    reading_.addError(ErrorType::SdoWriteError);
  }
  return success;
}

template <>
bool ElmoEthercat::sendSdoWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess, const int32_t value) {
  bool success = slavePtr_->sendSdoWriteInt32(index, subindex, completeAccess, value);
  if (!success) {
    std::lock_guard<std::recursive_mutex> lock(readingMutex_);
    reading_.addError(ErrorType::SdoWriteError);
  }
  return success;
}

template <>
bool ElmoEthercat::sendSdoWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess, const int64_t value) {
  bool success = slavePtr_->sendSdoWriteInt64(index, subindex, completeAccess, value);
  if (!success) {
    std::lock_guard<std::recursive_mutex> lock(readingMutex_);
    reading_.addError(ErrorType::SdoWriteError);
  }
  return success;
}

template <>
bool ElmoEthercat::sendSdoWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess, const uint8_t value) {
  bool success = slavePtr_->sendSdoWriteUInt8(index, subindex, completeAccess, value);
  if (!success) {
    std::lock_guard<std::recursive_mutex> lock(readingMutex_);
    reading_.addError(ErrorType::SdoWriteError);
  }
  return success;
}

template <>
bool ElmoEthercat::sendSdoWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess, const uint16_t value) {
  bool success = slavePtr_->sendSdoWriteUInt16(index, subindex, completeAccess, value);
  if (!success) {
    std::lock_guard<std::recursive_mutex> lock(readingMutex_);
    reading_.addError(ErrorType::SdoWriteError);
  }
  return success;
}

template <>
bool ElmoEthercat::sendSdoWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess, const uint32_t value) {
  bool success = slavePtr_->sendSdoWriteUInt32(index, subindex, completeAccess, value);
  if (!success) {
    std::lock_guard<std::recursive_mutex> lock(readingMutex_);
    reading_.addError(ErrorType::SdoWriteError);
  }
  return success;
}

template <>
bool ElmoEthercat::sendSdoWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess, const uint64_t value) {
  bool success = slavePtr_->sendSdoWriteUInt64(index, subindex, completeAccess, value);
  if (!success) {
    std::lock_guard<std::recursive_mutex> lock(readingMutex_);
    reading_.addError(ErrorType::SdoWriteError);
  }
  return success;
}

template <>
bool ElmoEthercat::sendSdoWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess, const float value) {
  bool success = slavePtr_->sendSdoWriteFloat(index, subindex, completeAccess, value);
  if (!success) {
    std::lock_guard<std::recursive_mutex> lock(readingMutex_);
    reading_.addError(ErrorType::SdoWriteError);
  }
  return success;
}

template <>
bool ElmoEthercat::sendSdoWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess, const double value) {
  bool success = slavePtr_->sendSdoWriteDouble(index, subindex, completeAccess, value);
  if (!success) {
    std::lock_guard<std::recursive_mutex> lock(readingMutex_);
    reading_.addError(ErrorType::SdoWriteError);
  }
  return success;
}

// Sdo read specialization
template <>
bool ElmoEthercat::sendSdoRead(const uint16_t index, const uint8_t subindex, const bool completeAccess, int8_t& value) {
  bool success = slavePtr_->sendSdoReadInt8(index, subindex, completeAccess, value);
  if (!success) {
    std::lock_guard<std::recursive_mutex> lock(readingMutex_);
    reading_.addError(ErrorType::SdoReadError);
  }
  return success;
}

template <>
bool ElmoEthercat::sendSdoRead(const uint16_t index, const uint8_t subindex, const bool completeAccess, int16_t& value) {
  bool success = slavePtr_->sendSdoReadInt16(index, subindex, completeAccess, value);
  if (!success) {
    std::lock_guard<std::recursive_mutex> lock(readingMutex_);
    reading_.addError(ErrorType::SdoReadError);
  }
  return success;
}

template <>
bool ElmoEthercat::sendSdoRead(const uint16_t index, const uint8_t subindex, const bool completeAccess, int32_t& value) {
  bool success = slavePtr_->sendSdoReadInt32(index, subindex, completeAccess, value);
  if (!success) {
    std::lock_guard<std::recursive_mutex> lock(readingMutex_);
    reading_.addError(ErrorType::SdoReadError);
  }
  return success;
}

template <>
bool ElmoEthercat::sendSdoRead(const uint16_t index, const uint8_t subindex, const bool completeAccess, int64_t& value) {
  bool success = slavePtr_->sendSdoReadInt64(index, subindex, completeAccess, value);
  if (!success) {
    std::lock_guard<std::recursive_mutex> lock(readingMutex_);
    reading_.addError(ErrorType::SdoReadError);
  }
  return success;
}

template <>
bool ElmoEthercat::sendSdoRead(const uint16_t index, const uint8_t subindex, const bool completeAccess, uint8_t& value) {
  bool success = slavePtr_->sendSdoReadUInt8(index, subindex, completeAccess, value);
  if (!success) {
    std::lock_guard<std::recursive_mutex> lock(readingMutex_);
    reading_.addError(ErrorType::SdoReadError);
  }
  return success;
}

template <>
bool ElmoEthercat::sendSdoRead(const uint16_t index, const uint8_t subindex, const bool completeAccess, uint16_t& value) {
  bool success = slavePtr_->sendSdoReadUInt16(index, subindex, completeAccess, value);
  if (!success) {
    std::lock_guard<std::recursive_mutex> lock(readingMutex_);
    reading_.addError(ErrorType::SdoReadError);
  }
  return success;
}

template <>
bool ElmoEthercat::sendSdoRead(const uint16_t index, const uint8_t subindex, const bool completeAccess, uint32_t& value) {
  bool success = slavePtr_->sendSdoReadUInt32(index, subindex, completeAccess, value);
  if (!success) {
    std::lock_guard<std::recursive_mutex> lock(readingMutex_);
    reading_.addError(ErrorType::SdoReadError);
  }
  return success;
}

template <>
bool ElmoEthercat::sendSdoRead(const uint16_t index, const uint8_t subindex, const bool completeAccess, uint64_t& value) {
  bool success = slavePtr_->sendSdoReadUInt64(index, subindex, completeAccess, value);
  if (!success) {
    std::lock_guard<std::recursive_mutex> lock(readingMutex_);
    reading_.addError(ErrorType::SdoReadError);
  }
  return success;
}

template <>
bool ElmoEthercat::sendSdoRead(const uint16_t index, const uint8_t subindex, const bool completeAccess, float& value) {
  bool success = slavePtr_->sendSdoReadFloat(index, subindex, completeAccess, value);
  if (!success) {
    std::lock_guard<std::recursive_mutex> lock(readingMutex_);
    reading_.addError(ErrorType::SdoReadError);
  }
  return success;
}

template <>
bool ElmoEthercat::sendSdoRead(const uint16_t index, const uint8_t subindex, const bool completeAccess, double& value) {
  bool success = slavePtr_->sendSdoReadDouble(index, subindex, completeAccess, value);
  if (!success) {
    std::lock_guard<std::recursive_mutex> lock(readingMutex_);
    reading_.addError(ErrorType::SdoReadError);
  }
  return success;
}

template <>
bool ElmoEthercat::sendSdoRead(const uint16_t index, const uint8_t subindex, const bool completeAccess, std::string& value) {
  bool success = slavePtr_->sendSdoReadString(index, subindex, completeAccess, value);
  if (!success) {
    std::lock_guard<std::recursive_mutex> lock(readingMutex_);
    reading_.addError(ErrorType::SdoReadError);
  }
  return success;
}

// Sdo Verify write
template <>
bool ElmoEthercat::sdoVerifyWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess, int8_t value, float delay) {
  int8_t testVal;
  slavePtr_->sendSdoWriteInt8(index, subindex, completeAccess, value);
  usleep(static_cast<unsigned int>(delay));
  slavePtr_->sendSdoReadInt8(index, subindex, completeAccess, testVal);
  return (value == testVal);
}

template <>
bool ElmoEthercat::sdoVerifyWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess, int16_t value, float delay) {
  int16_t testVal;
  slavePtr_->sendSdoWriteInt16(index, subindex, completeAccess, value);
  usleep(static_cast<unsigned int>(delay));
  slavePtr_->sendSdoReadInt16(index, subindex, completeAccess, testVal);
  return (value == testVal);
}

template <>
bool ElmoEthercat::sdoVerifyWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess, int32_t value, float delay) {
  int32_t testVal;
  slavePtr_->sendSdoWriteInt32(index, subindex, completeAccess, value);
  usleep(static_cast<unsigned int>(delay));
  slavePtr_->sendSdoReadInt32(index, subindex, completeAccess, testVal);
  return (value == testVal);
}

template <>
bool ElmoEthercat::sdoVerifyWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess, int64_t value, float delay) {
  int64_t testVal;
  slavePtr_->sendSdoWriteInt64(index, subindex, completeAccess, value);
  usleep(static_cast<unsigned int>(delay));
  slavePtr_->sendSdoReadInt64(index, subindex, completeAccess, testVal);
  return (value == testVal);
}

template <>
bool ElmoEthercat::sdoVerifyWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess, uint8_t value, float delay) {
  uint8_t testVal;
  slavePtr_->sendSdoWriteUInt8(index, subindex, completeAccess, value);
  usleep(static_cast<unsigned int>(delay));
  slavePtr_->sendSdoReadUInt8(index, subindex, completeAccess, testVal);
  return (value == testVal);
}

template <>
bool ElmoEthercat::sdoVerifyWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess, uint16_t value, float delay) {
  uint16_t testVal;
  slavePtr_->sendSdoWriteUInt16(index, subindex, completeAccess, value);
  usleep(static_cast<unsigned int>(delay));
  slavePtr_->sendSdoReadUInt16(index, subindex, completeAccess, testVal);
  return (value == testVal);
}

template <>
bool ElmoEthercat::sdoVerifyWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess, uint32_t value, float delay) {
  uint32_t testVal;
  slavePtr_->sendSdoWriteUInt32(index, subindex, completeAccess, value);
  usleep(static_cast<unsigned int>(delay));
  slavePtr_->sendSdoReadUInt32(index, subindex, completeAccess, testVal);
  return (value == testVal);
}

template <>
bool ElmoEthercat::sdoVerifyWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess, uint64_t value, float delay) {
  uint64_t testVal;
  slavePtr_->sendSdoWriteUInt64(index, subindex, completeAccess, value);
  usleep(static_cast<unsigned int>(delay));
  slavePtr_->sendSdoReadUInt64(index, subindex, completeAccess, testVal);
  return (value == testVal);
}

template <>
bool ElmoEthercat::sdoVerifyWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess, float value, float delay) {
  float testVal;
  slavePtr_->sendSdoWriteFloat(index, subindex, completeAccess, value);
  usleep(static_cast<unsigned int>(delay));
  slavePtr_->sendSdoReadFloat(index, subindex, completeAccess, testVal);
  return (value == testVal);
}

template <>
bool ElmoEthercat::sdoVerifyWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess, double value, float delay) {
  double testVal;
  slavePtr_->sendSdoWriteDouble(index, subindex, completeAccess, value);
  usleep(static_cast<unsigned int>(delay));
  slavePtr_->sendSdoReadDouble(index, subindex, completeAccess, testVal);
  return (value == testVal);
}

}  // namespace ethercat
}  // namespace elmo
