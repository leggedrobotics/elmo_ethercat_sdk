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
#include <elmo/Controlword.hpp>
#include <elmo/Elmo.hpp>
#include <elmo/Statusword.hpp>
#include <string>
#include <yaml-cpp/yaml.h>

#include <elmo_ethercat/Configuration.hpp>
#include <elmo_ethercat/ElmoEthercatSlave.hpp>

namespace elmo {

namespace ethercat {

/*!
 * Alias for the Command in the elmo::ethercat namespace
 * All objects needed by the end user are therefore available in this namespace
 */
using Command = elmo::Command;

class ElmoEthercat : public Elmo {
 public:
  // empty constructor
  ElmoEthercat() = default;
  ~ElmoEthercat() override = default;

  // overridden methods

  // startup
  void startupWithCommunication() override;
  // send staged command to slave (Pdo)
  void updateSendStagedCommand() override;
  // get a reading from the slave
  void updateProcessReading() override;

  /*!
   * outputs the Reading object for further use
   * @return	a copy of the current Reading object
   */
  Reading getReading() const;

  /*!
   * outputs the Reading object for further use
   * @param[out] reading	a copy of the current Reading object
   */
  void getReading(Reading& reading) const;

  /*!
   * @brief	Read a configuration file and configure everything
   * This method first reads the yaml file. Have a look at the
   * "ConfigurationTemplate.yaml" file in this repo's main folder for an example
   * file Note: using absolute paths may be advantageous depending on the way
   * the final program is being run. Afterwards configureAll() is called to
   * configure everything.
   * @param[in] fileName	The path to the configuration file
   * @return	true if the configuration of the elmo servo drive was successful
   * (look at ElmoEthercat::configureHardware for further explanations)
   */
  bool loadConfigFile(const std::string& fileName);
  /*!
   * @brief	Read a yaml node and configure everything
   * Note: using absolute paths may be advantageous depending on the way the
   * final program is being run.
   * @param[in] configNode	Yaml node containing the configuration data
   * @return	true if the configuration of the elmo servo drive was successful
   * (look at ElmoEthercat::configureHardware for further explanations)
   */
  bool loadConfigNode(YAML::Node configNode);

  /*!
   * @brief	Configure everything
   * This Object, the ElmoEthercatSlave object pointed to by the slavePtr_ and
   * the hardware are configured according to the Configuration object Note:
   * depending on the config options, the Configuration object which is passed
   * as a parameter will be changed (only the motorRatedCurrentA_ variable)
   * @param[in,out] configuration	The Configuration object containing the
   * configuration parameters
   * @return	true if the configuration of the elmo servo drive was successful
   * (look at ElmoEthercat::configureHardware for further explanations)
   */
  bool configureAll(Configuration& configuration);

  /*!
   * @brief	Configure this object
   * This method sets the configuration parameters contained within this classe.
   * Note: This should be called before "configureHardware".
   * @param[in] configuration	The Configuration object containing the
   * configuration parameters
   */
  void configureElmoEthercat(const Configuration& configuration);

  /*!
   * @brief	Configure the elmo servo drive
   * This method sends the necessary PDO mapping SDOs and sets the mode of
   * operation These are the only configuration parameters that are actually
   * sent over EtherCAT If the configuration of the servo drive fails, an Error
   * will be added to the Reading objects. This Error can be read by using
   * ElmoEthercat::getReading()
   * @param[in] configuration	The Configuration object including the requested
   * configuration parameters
   * @return	true if all the SDO writings as well as the SDO readings for
   * verification purposes were successful
   */
  bool configureHardware(const Configuration& configuration);

  /*!
   * @brief	return the active Configuration object
   * This method returns the configuration which was used to configure the
   * ElmoEthercatSlave object and the actual device (hardware)
   * @return	The active Configuration
   */
  Configuration getActiveConfiguration();

  /*!
   * read the current statusword from the slave via SDO
   * @param[out] statusword the current status of the slave
   * @return true on success
   */
  bool getStatuswordViaSdo(Statusword& statusword);

  /*!
   * send a new controlword to the slave via SDO
   * @param[in] controlword the Controlword to be sent
   * @return true on success
   */
  bool setControlwordViaSdo(Controlword& controlword);

  /*!
   * set the desired state of the drive
   * The correct order of state changes is executed automatically
   * "states" means the drive states, not the EtherCAT states
   * EtherCAT state changes should be handled by the code which creates an
   * manages the used EthercatBusBase object
   * @param state	the target state
   * @return true if the SDO writes were successful
   */
  bool setDriveStateViaSdo(const DriveState& driveState);

  /*!
   * Change the drive state according to the diagram in the DS402 datasheed from
   * elmo (page 37)
   * @param stateTransition	which transition shall be made
   * @return	true if SDO writes were successful
   */
  bool stateTransitionViaSdo(const StateTransition& stateTransition);

  /*!
   * @brief	Enables the PDO drive state machine
   * The slave will try to reach the requested drive state.
   * For more documentation have a look at
   * ElmoEthercatSlave::setDriveStateViaPdo.
   * @param[in] driveState	The requested target drive state
   * @param[in] waitForState	True if the method shall return only after a
   * successfull state change (or a predefined maximum timeout to prevent
   * hangup, defined in the config file)
   * @return	true after successfully reaching the target state or if no
   * waiting is requested
   */
  bool setDriveStateViaPdo(const DriveState& driveState, const bool waitForState = true);

  /*!
   * Sets the drive into operational mode such that PDO communication is
   * possible. Always call this method BEFORE starting the PDO read/write cycle
   * of the bus.
   * @param waitForOperationEnabled	true if this method should only return
   * once the operation enabled state has been reached by the slave (optional)
   * @return	true if the operation enabled state has been reached (or if the
   * state change is not checked), false if the operation enabled state has not
   * been reached during the maximal waiting period
   */
  bool getReady(bool waitForOperationEnabled = true);

  // set the corresponding slave pointer
  void setSlavePointer(const ElmoEthercatSlavePtr& slavePtr);

  /*!
   * @brief	Return the name of the slave
   * @return	The name of the slave
   */
  std::string getNameOfSlave() const;

  /*!
   * @brief	Return the address of the slave
   * @return	The address of the slave
   */
  uint16_t getAddressOfSlave() const;

  /*!
   * Return the success of the last drive state change over PDO.
   * @return true if the last state change was successfully finished.
   */
  bool getLastPdoStateChangeSuccess() const { return slavePtr_->getLastPdoStateChangeSuccess(); }

  // shutting down
  void shutdown() override;

  // Sdo handling
  template <typename Value>
  bool sendSdoWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess, Value value);

  template <typename Value>
  bool sendSdoRead(const uint16_t index, const uint8_t subindex, const bool completeAccess, Value& value);

  // Sdo write with verification
  template <typename Value>
  bool sdoVerifyWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess, Value value, float delay = 0);

 protected:
  /*!
   * Sends the necessary SDOs to map the Rx and Tx Pdos
   * This method gets called automatically when executing the
   * "configureHardware" method.
   * Do not use this method manually.
   * @param[in] rxPdoTypeEnum	the RxPdo Type that shall be mapped
   * @param[in] txPdoTypeEnum	the TxPdo Type that shall be mapped
   * @return	true on success
   */
  bool mapPdos(RxPdoTypeEnum rxPdoTypeEnum, TxPdoTypeEnum txPdoTypeEnum);

  ElmoEthercatSlavePtr slavePtr_;

  /*!
   * The elmo::ethercat::Reading object which is filled during PDO communication
   * Note: This is not an elmo::Reading but a child of it!
   */
  Reading reading_;

  /*!
   * Configuration paramters which can be set using Cpnfiguration objects and /
   * or yaml config files
   */
  unsigned int sdoVerifyTimeout_{20000};
  bool printErrorMessage_{true};
};

using ElmoEthercatPtr = std::shared_ptr<elmo::ethercat::ElmoEthercat>;

// template specialization sdo writing
template <>
bool ElmoEthercat::sendSdoWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess, const int8_t value);
template <>
bool ElmoEthercat::sendSdoWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess, const int16_t value);
template <>
bool ElmoEthercat::sendSdoWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess, const int32_t value);
template <>
bool ElmoEthercat::sendSdoWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess, const int64_t value);
template <>
bool ElmoEthercat::sendSdoWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess, const uint8_t value);
template <>
bool ElmoEthercat::sendSdoWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess, const uint16_t value);
template <>
bool ElmoEthercat::sendSdoWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess, const uint32_t value);
template <>
bool ElmoEthercat::sendSdoWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess, const uint64_t value);
template <>
bool ElmoEthercat::sendSdoWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess, const float value);
template <>
bool ElmoEthercat::sendSdoWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess, const double value);

// template specialization sdo reading
template <>
bool ElmoEthercat::sendSdoRead(const uint16_t index, const uint8_t subindex, const bool completeAccess, int8_t& value);
template <>
bool ElmoEthercat::sendSdoRead(const uint16_t index, const uint8_t subindex, const bool completeAccess, int16_t& value);
template <>
bool ElmoEthercat::sendSdoRead(const uint16_t index, const uint8_t subindex, const bool completeAccess, int32_t& value);
template <>
bool ElmoEthercat::sendSdoRead(const uint16_t index, const uint8_t subindex, const bool completeAccess, int64_t& value);
template <>
bool ElmoEthercat::sendSdoRead(const uint16_t index, const uint8_t subindex, const bool completeAccess, uint8_t& value);
template <>
bool ElmoEthercat::sendSdoRead(const uint16_t index, const uint8_t subindex, const bool completeAccess, uint16_t& value);
template <>
bool ElmoEthercat::sendSdoRead(const uint16_t index, const uint8_t subindex, const bool completeAccess, uint32_t& value);
template <>
bool ElmoEthercat::sendSdoRead(const uint16_t index, const uint8_t subindex, const bool completeAccess, uint64_t& value);
template <>
bool ElmoEthercat::sendSdoRead(const uint16_t index, const uint8_t subindex, const bool completeAccess, float& value);
template <>
bool ElmoEthercat::sendSdoRead(const uint16_t index, const uint8_t subindex, const bool completeAccess, double& value);
template <>
bool ElmoEthercat::sendSdoRead(const uint16_t index, const uint8_t subindex, const bool completeAccess, std::string& value);

// template specialization sdo verivy write
template <>
bool ElmoEthercat::sdoVerifyWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess, int8_t value, float delay);
template <>
bool ElmoEthercat::sdoVerifyWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess, int16_t value, float delay);
template <>
bool ElmoEthercat::sdoVerifyWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess, int32_t value, float delay);
template <>
bool ElmoEthercat::sdoVerifyWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess, int64_t value, float delay);
template <>
bool ElmoEthercat::sdoVerifyWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess, uint8_t value, float delay);
template <>
bool ElmoEthercat::sdoVerifyWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess, uint16_t value, float delay);
template <>
bool ElmoEthercat::sdoVerifyWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess, uint32_t value, float delay);
template <>
bool ElmoEthercat::sdoVerifyWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess, uint64_t value, float delay);
template <>
bool ElmoEthercat::sdoVerifyWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess, float value, float delay);
template <>
bool ElmoEthercat::sdoVerifyWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess, double value, float delay);

}  // namespace ethercat
}  // namespace elmo
