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

#include "elmo_ethercat_sdk/ConfigurationParser.hpp"

namespace elmo {

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
    MELO_WARN_STREAM("[elmo_ethercat_sdk:ConfigurationParser::parseConfiguration]: field '" << varName
                                                                                            << "' is missing. Default value will be used.");
    return false;
  }
  try {
    T tmpVar = yamlNode[varName].as<T>();
    var = tmpVar;
    return true;
  } catch (...) {
    MELO_ERROR_STREAM("[elmo_ethercat_sdk:ConfigurationParser::getValueFromFile] Error while parsing value \""
                      << varName << "\", default values will be used");
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
    MELO_WARN_STREAM("[elmo_ethercat_sdk:ConfigurationParser::parseConfiguration]: field '" << varName
                                                                                            << "' is missing. Default value will be used.");
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
      MELO_ERROR_STREAM("[elmo_ethercat_sdk:ConfigurationParser::getRxPdoFromFile] Unsupported Rx PDO Type");
      return false;
    }
  } catch (...) {
    MELO_ERROR_STREAM("[elmo_ethercat_sdk:ConfigurationParser::getRxPdoFromFile] Error while parsing value \""
                      << varName << "\", default values will be used");
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
    MELO_WARN_STREAM("[elmo_ethercat_sdk:ConfigurationParser::parseConfiguration]: field '" << varName
                                                                                            << "' is missing. Default value will be used.");
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
      MELO_ERROR_STREAM("[elmo_ethercat_sdk:ConfigurationParser::getTxPdoFromFile] Unsupported Tx PDO Type");
      return false;
    }

  } catch (...) {
    MELO_ERROR_STREAM("[elmo_ethercat_sdk:ConfigurationParser::getTxPdoFromFile] Error while parsing value \""
                      << varName << "\", default values will be used");
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
    MELO_WARN_STREAM("[elmo_ethercat_sdk:ConfigurationParser::parseConfiguration]: field '" << varName
                                                                                            << "' is missing. Default value will be used.");
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
      MELO_ERROR_STREAM("[elmo_ethercat_sdk:ConfigurationParser::getModeFromFile] Unsupported Mode Of Operation");
      return false;
    }
  } catch (...) {
    MELO_ERROR_STREAM("[elmo_ethercat_sdk:ConfigurationParser::getModeFromFile] Error while parsing value \""
                      << varName << "\", default values will be used");
    return false;
  }
}

/*!
 * Function to read an Encoder Position enum from the yaml file.
 * @param[in] yamlNode	the node containing the requested value
 * @param [in] varName	The name of the variable
 * @param [out] encoderPosition Position of the encoder
 * @return	true on success
 */
bool getEncoderPositionFromFile(YAML::Node& yamlNode, const std::string& varName, Configuration::EncoderPosition& encoderPosition) {
  if (!yamlNode[varName].IsDefined()) {
    MELO_WARN_STREAM("[elmo_ethercat_sdk:ConfigurationParser::parseConfiguration]: field '" << varName
                                                                                            << "' is missing. Default value will be used.");
    return false;
  }
  try {
    std::string str = yamlNode[varName].as<std::string>();
    /// no switch statements with std::string
    if (str == "motor") {
      encoderPosition = Configuration::EncoderPosition::motor;
      return true;
    } else if (str == "joint") {
      encoderPosition = Configuration::EncoderPosition::joint;
      return true;
    } else {
      MELO_ERROR_STREAM("[elmo_ethercat_sdk:ConfigurationParser::getEncoderPositionFromFile] Unsupported encoder position");
      return false;
    }
  } catch (...) {
    MELO_ERROR_STREAM("[elmo_ethercat_sdk:ConfigurationParser::getEncoderPositionFromFile] Error while parsing value \""
                      << varName << "\", default values will be used");
    return false;
  }
}

ConfigurationParser::ConfigurationParser(const std::string& filename) {
  YAML::Node configNode;
  try {
    configNode = YAML::LoadFile(filename);
  } catch (...) {
    MELO_FATAL_STREAM("[elmo_ethercat_sdk:ConfigurationParser::ConfigurationParser] Loading YAML configuration file '" << filename
                                                                                                                       << "' failed.");
  }
  parseConfiguration(configNode);
}

ConfigurationParser::ConfigurationParser(YAML::Node configNode) {
  parseConfiguration(configNode);
}

void ConfigurationParser::parseConfiguration(YAML::Node configNode) {
  if (configNode["Elmo"].IsDefined()) {
    /// A new node for the ElmoEthercat class
    YAML::Node elmoNode = configNode["Elmo"];

    unsigned int configRunSdoVerifyTimeout;
    if (getValueFromFile(elmoNode, "config_run_sdo_verify_timeout", configRunSdoVerifyTimeout)) {
      configuration_.configRunSdoVerifyTimeout = configRunSdoVerifyTimeout;
    }

    bool printDebugMessages;
    if (getValueFromFile(elmoNode, "print_debug_messages", printDebugMessages)) {
      configuration_.printDebugMessages = printDebugMessages;
    }

    bool useRawCommands;
    if (getValueFromFile(elmoNode, "use_raw_commands", useRawCommands)) {
      configuration_.useRawCommands = useRawCommands;
    }

    unsigned int driveStateChangeMinTimeout;
    if (getValueFromFile(elmoNode, "drive_state_change_min_timeout", driveStateChangeMinTimeout)) {
      configuration_.driveStateChangeMinTimeout = driveStateChangeMinTimeout;
    }

    unsigned int minNumberOfSuccessfulTargetStateReadings;
    if (getValueFromFile(elmoNode, "min_number_of_successful_target_state_readings", minNumberOfSuccessfulTargetStateReadings)) {
      configuration_.minNumberOfSuccessfulTargetStateReadings = minNumberOfSuccessfulTargetStateReadings;
    }

    unsigned int driveStateChangeMaxTimeout;
    if (getValueFromFile(elmoNode, "drive_state_change_max_timeout", driveStateChangeMaxTimeout)) {
      configuration_.driveStateChangeMaxTimeout = driveStateChangeMaxTimeout;
    }
  }

  /// The configuration options for the elmo::ethercat::Reading class
  if (configNode["Reading"].IsDefined()) {
    YAML::Node readingNode = configNode["Reading"];

    bool forceAppendEqualError;
    if (getValueFromFile(readingNode, "force_append_equal_error", forceAppendEqualError)) {
      configuration_.forceAppendEqualError = forceAppendEqualError;
    }

    bool forceAppendEqualFault;
    if (getValueFromFile(readingNode, "force_append_equal_fault", forceAppendEqualFault)) {
      configuration_.forceAppendEqualFault = forceAppendEqualFault;
    }

    unsigned int errorStorageCapacity;
    if (getValueFromFile(readingNode, "error_storage_capacity", errorStorageCapacity)) {
      configuration_.errorStorageCapacity = errorStorageCapacity;
    }

    unsigned int faultStorageCapacity;
    if (getValueFromFile(readingNode, "fault_storage_capacity", faultStorageCapacity)) {
      configuration_.faultStorageCapacity = faultStorageCapacity;
    }
  }

  /// The configuration options for the Elmo servo drive ("hardware")
  if (configNode["Hardware"].IsDefined()) {
    YAML::Node hardwareNode = configNode["Hardware"];

    RxPdoTypeEnum rxPdo;
    if (getRxPdoFromFile(hardwareNode, "rx_pdo_type", rxPdo)) {
      configuration_.rxPdoTypeEnum = rxPdo;
    }

    TxPdoTypeEnum txPdo;
    if (getTxPdoFromFile(hardwareNode, "tx_pdo_type", txPdo)) {
      configuration_.txPdoTypeEnum = txPdo;
    }

    ModeOfOperationEnum modeOfOperation_;
    if (getModeFromFile(hardwareNode, "mode_of_operation", modeOfOperation_)) {
      configuration_.modeOfOperationEnum = modeOfOperation_;
    }

    int32_t positionEncoderResolution;
    if (getValueFromFile(hardwareNode, "position_encoder_resolution", positionEncoderResolution)) {
      configuration_.positionEncoderResolution = positionEncoderResolution;
    }

    std::pair<float, float> gearRatio;
    if (getValueFromFile(hardwareNode, "gear_ratio", gearRatio)) {
      configuration_.gearRatio = static_cast<double>(gearRatio.first) / static_cast<double>(gearRatio.second);
    }

    double motorConstant;
    if (getValueFromFile(hardwareNode, "motor_constant", motorConstant)) {
      configuration_.motorConstant = motorConstant;
    }

    double maxCurrentA;
    if (getValueFromFile(hardwareNode, "max_current", maxCurrentA)) {
      configuration_.maxCurrentA = maxCurrentA;
    }

    double motorRatedCurrentA;
    if (getValueFromFile(hardwareNode, "motor_rated_current", motorRatedCurrentA)) {
      configuration_.motorRatedCurrentA = motorRatedCurrentA;
    }

    bool useMultipleModeOfOperations;
    if (getValueFromFile(hardwareNode, "use_multiple_modes_of_operation", useMultipleModeOfOperations)) {
      configuration_.useMultipleModeOfOperations = useMultipleModeOfOperations;
    }
    int direction;
    if (getValueFromFile(hardwareNode, "direction", direction)) {
      configuration_.direction = direction;
    }
    Configuration::EncoderPosition encoderPosition;
    if (getEncoderPositionFromFile(hardwareNode, "encoder_position", encoderPosition)) {
      configuration_.encoderPosition = encoderPosition;
    }
  }
}

Configuration ConfigurationParser::getConfiguration() const {
  return configuration_;
}

}  // namespace elmo
