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

#include <cstdint>

#include <elmo_ethercat/ConfigurationParser.hpp>

namespace elmo {
namespace ethercat {

/*!
 * Function template for convenience
 * @param[in] yamlNode	the current node containing the requested variable
 * @param[in] varName	The name of the variable
 * @param[out] var	The variable which shall be read
 * @return	true on success
 */
template <typename T>
bool getValueFromFile(YAML::Node& yamlNode, const std::string& varName, T& var) {
  if (!yamlNode[varName].IsDefined()) {
    return false;
  }
  try {
    T tmpVar = yamlNode[varName].as<T>();
    var = tmpVar;
    return true;
  } catch (...) {
    MELO_ERROR_STREAM("Error while parsing value \"" << varName << "\", default values will be used");
    return false;
  }
}
/*!
 * Function to read a RxPdo enum from the yaml file
 * @param[in] yamlNode	the node containing the requested value
 * @param [in] varName	The name of the variable
 * @param [out] rxPdo	The read Rx PDO type
 * @return	true on success
 */
bool getRxPdoFromFile(YAML::Node& yamlNode, const std::string& varName, RxPdoTypeEnum& rxPdo) {
  if (!yamlNode[varName].IsDefined()) {
    return false;
  }
  try {
    std::string str = yamlNode[varName].as<std::string>();
    /// no switch statements with std::string possible
    if (str == "NA") {
      rxPdo = RxPdoTypeEnum::NA;
      return true;
    } else if (str == "RxPdoStandard") {
      rxPdo = RxPdoTypeEnum::RxPdoStandard;
      return true;
    } else if (str == "RxPdoCST") {
      rxPdo = RxPdoTypeEnum::RxPdoCST;
      return true;
    } else {
      MELO_ERROR_STREAM("Unsupported Rx PDO Type");
      return false;
    }
  } catch (...) {
    MELO_ERROR_STREAM("Error while parsing value \"" << varName << "\", default values will be used");
    return false;
  }
}

/*!
 * Function to read a TxPdo enum from the yaml file
 * @param[in] yamlNode	the node containing the requested value
 * @param [in] varName	The name of the variable
 * @param [out] txPdo	The read Tx PDO type
 * @return	true on success
 */
bool getTxPdoFromFile(YAML::Node& yamlNode, const std::string& varName, TxPdoTypeEnum& txPdo) {
  if (!yamlNode[varName].IsDefined()) {
    return false;
  }
  try {
    std::string str = yamlNode[varName].as<std::string>();
    /// no switch statements with std::string
    if (str == "NA") {
      txPdo = TxPdoTypeEnum::NA;
      return true;
    } else if (str == "TxPdoCST") {
      txPdo = TxPdoTypeEnum::TxPdoCST;
      return true;
    } else if (str == "TxPdoStandard") {
      txPdo = TxPdoTypeEnum::TxPdoStandard;
      return true;
    } else {
      MELO_ERROR_STREAM("Unsupported Tx PDO Type");
      return false;
    }

  } catch (...) {
    MELO_ERROR_STREAM("Error while parsing value \"" << varName << "\", default values will be used");
    return false;
  }
}

/*!
 * Function to read a Mode of Operation enum from the yaml file
 * @param[in] yamlNode	the node containing the requested value
 * @param [in] varName	The name of the variable
 * @param [out] mode	The read mode of operation
 * @return	true on success
 */
bool getModeFromFile(YAML::Node& yamlNode, const std::string& varName, ModeOfOperationEnum& mode) {
  if (!yamlNode[varName].IsDefined()) {
    return false;
  }
  try {
    std::string str = yamlNode[varName].as<std::string>();
    /// no switch statements with std::string
    if (str == "ProfiledPositionMode") {
      mode = ModeOfOperationEnum::ProfiledPositionMode;
      return true;
    } else if (str == "ProfiledVelocityMode") {
      mode = ModeOfOperationEnum::ProfiledVelocityMode;
      return true;
    } else if (str == "ProfiledTorqueMode") {
      mode = ModeOfOperationEnum::ProfiledTorqueMode;
      return true;
    } else if (str == "HomingMode") {
      mode = ModeOfOperationEnum::HomingMode;
      return true;
    } else if (str == "CyclicSynchronousPositionMode") {
      mode = ModeOfOperationEnum::CyclicSynchronousPositionMode;
      return true;
    } else if (str == "CyclicSynchronousVelocityMode") {
      mode = ModeOfOperationEnum::CyclicSynchronousVelocityMode;
      return true;
    } else if (str == "CyclicSynchronousTorqueMode") {
      mode = ModeOfOperationEnum::CyclicSynchronousTorqueMode;
      return true;
    } else {
      MELO_ERROR_STREAM("Unsupported Mode Of Operation");
      return false;
    }
  } catch (...) {
    MELO_ERROR_STREAM("Error while parsing value \"" << varName << "\", default values will be used");
    return false;
  }
}

ConfigurationParser::ConfigurationParser(const std::string& filename) {
  YAML::Node configNode;
  try{
    configNode = YAML::LoadFile(filename);
  }catch(...){
    MELO_FATAL_STREAM("Loading YAML configuration file '" << filename << "' failed.");
  }
  parseConfiguration(configNode);
}

ConfigurationParser::ConfigurationParser(YAML::Node configNode) {
  parseConfiguration(configNode);
}

void ConfigurationParser::parseConfiguration(YAML::Node configNode) {
  /// The configuration options for the elmo::ethercat::ElmoEthercat class
  if (configNode["ElmoEthercat"].IsDefined()) {
    /// A new node for the ElmoEthercat class
    YAML::Node elmoEthercatNode = configNode["ElmoEthercat"];

    unsigned int sdoVerifyTimeout;
    if (getValueFromFile(elmoEthercatNode, "SDO_VERIFY_TIMEOUT", sdoVerifyTimeout)) {
      configuration_.setSdoVerifyTimeout(sdoVerifyTimeout);
    }

    bool printErrorMessage;
    if (getValueFromFile(elmoEthercatNode, "PRINT_ERROR_MESSAGE", printErrorMessage)) {
      configuration_.setPrintErrorMessageElmoEthercat(printErrorMessage);
    }

    bool useRawCommands;
    if (getValueFromFile(elmoEthercatNode, "USE_RAW_COMMANDS", useRawCommands)) {
      configuration_.setUseRawCommands(useRawCommands);
    }
  }

  /// The configuration options for the elmo::ethercat::ElmoEthercatSlave class
  if (configNode["ElmoEthercatSlave"].IsDefined()) {
    YAML::Node elmoEthercatSlaveNode = configNode["ElmoEthercatSlave"];

    unsigned int driveStateChangeMinTimeout;
    if (getValueFromFile(elmoEthercatSlaveNode, "DRIVE_STATE_CHANGE_MIN_TIMEOUT", driveStateChangeMinTimeout)) {
      configuration_.setDriveStateChangeMinTimeout(driveStateChangeMinTimeout);
    }

    unsigned int minNumberOfSuccessfulTargetStateReadings;
    if (getValueFromFile(elmoEthercatSlaveNode, "MIN_NUMBER_OF_SUCCESSFUL_TARGET_STATE_READINGS",
                         minNumberOfSuccessfulTargetStateReadings)) {
      configuration_.setMinNumberOfSuccessfulTargetStateReadings(minNumberOfSuccessfulTargetStateReadings);
    }

    unsigned int driveStateChangeMaxTimeout;
    if (getValueFromFile(elmoEthercatSlaveNode, "DRIVE_STATE_CHANGE_MAX_TIMEOUT", driveStateChangeMaxTimeout)) {
      configuration_.setDriveStateChangeMaxTimeout(driveStateChangeMaxTimeout);
    }

    bool printErrorMessage;
    if (getValueFromFile(elmoEthercatSlaveNode, "PRINT_ERROR_MESSAGE", printErrorMessage)) {
      configuration_.setPrintErrorMessageElmoEthercatSlave(printErrorMessage);
    }
  }

  /// The configuration options for the elmo::ethercat::Reading class
  if (configNode["Reading"].IsDefined()) {
    YAML::Node readingNode = configNode["Reading"];

    bool forceAppendEqualError;
    if (getValueFromFile(readingNode, "FORCE_APPEND_EQUAL_ERROR", forceAppendEqualError)) {
      configuration_.setForceAppendEqualError(forceAppendEqualError);
    }

    bool forceAppendEqualFault;
    if (getValueFromFile(readingNode, "FORCE_APPEND_EQUAL_FAULT", forceAppendEqualFault)) {
      configuration_.setForceAppendEqualFault(forceAppendEqualFault);
    }

    unsigned int errorStorageCapacity;
    if (getValueFromFile(readingNode, "ERROR_STORAGE_CAPACITY", errorStorageCapacity)) {
      configuration_.setErrorStorageCapacity(errorStorageCapacity);
    }

    unsigned int faultStorageCapacity;
    if (getValueFromFile(readingNode, "FAULT_STORAGE_CAPACITY", faultStorageCapacity)) {
      configuration_.setFaultStorageCapacity(faultStorageCapacity);
    }
  }

  /// The configuration options for the Elmo servo drive ("hardware")
  if (configNode["Hardware"].IsDefined()) {
    YAML::Node hardwareNode = configNode["Hardware"];

    RxPdoTypeEnum rxPdo;
    if (getRxPdoFromFile(hardwareNode, "RX_PDO_TYPE", rxPdo)) {
      configuration_.setRxPdoTypeEnum(rxPdo);
    }

    TxPdoTypeEnum txPdo;
    if (getTxPdoFromFile(hardwareNode, "TX_PDO_TYPE", txPdo)) {
      configuration_.setTxPdoTypeEnum(txPdo);
    }

    ModeOfOperationEnum modeOfOperation_;
    if (getModeFromFile(hardwareNode, "MODE_OF_OPERATION", modeOfOperation_)) {
      configuration_.setModeOfOperation(modeOfOperation_);
    }

    int32_t positionEncoderResolution;
    if (getValueFromFile(hardwareNode, "POSITION_ENCODER_RESOLUTION", positionEncoderResolution)) {
      configuration_.setPositionEncoderResolution(positionEncoderResolution);
    }

    std::pair<float, float> gearRatio;
    if (getValueFromFile(hardwareNode, "GEAR_RATIO", gearRatio)) {
      configuration_.setGearRatio(gearRatio);
    }

    double motorConstant;
    if (getValueFromFile(hardwareNode, "MOTOR_CONSTANT", motorConstant)) {
      configuration_.setMotorConstant(motorConstant);
    }

    double maxCurrentA;
    if (getValueFromFile(hardwareNode, "MAX_CURRENT", maxCurrentA)) {
      configuration_.setMaxCurrentA(maxCurrentA);
    }

    double motorRatedCurrentA;
    if (getValueFromFile(hardwareNode, "MOTOR_RATED_CURRENT", motorRatedCurrentA)) {
      configuration_.setMotorRatedCurrentA(motorRatedCurrentA);
    }

    bool useMultipleModeOfOperations;
    if (getValueFromFile(hardwareNode, "USE_MULTIPLE_MODE_OF_OPERATIONS", useMultipleModeOfOperations)) {
      configuration_.setUseMultipleModeOfOperations(useMultipleModeOfOperations);
    }
  }
}

Configuration ConfigurationParser::getConfiguration() const {
  return configuration_;
}

}  // namespace ethercat
}  // namespace elmo
