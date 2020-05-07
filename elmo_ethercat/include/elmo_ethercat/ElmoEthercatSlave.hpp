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

#include <atomic>
#include <chrono>
#include <elmo/Command.hpp>
#include <elmo/Controlword.hpp>
#include <memory>
#include <mutex>
#include <soem_interface/EthercatBusBase.hpp>
#include <soem_interface/EthercatSlaveBase.hpp>
#include <string>

#include <elmo_ethercat/Configuration.hpp>
#include <elmo_ethercat/PdoTypeEnum.hpp>
#include <elmo_ethercat/Reading.hpp>
#include <elmo_ethercat/RxPdo.hpp>
#include <elmo_ethercat/TxPdo.hpp>

namespace elmo {
namespace ethercat {

class ElmoEthercatSlave : public soem_interface::EthercatSlaveBase {
 public:
  // no empty constructor
  ElmoEthercatSlave() = delete;

  // the only accepted constructor:
  ElmoEthercatSlave(std::string name, soem_interface::EthercatBusBase* bus, uint32_t address,
                    const RxPdoTypeEnum rxPdoTypeEnum = RxPdoTypeEnum::RxPdoStandard,
                    const TxPdoTypeEnum txPdoTypeEnum = TxPdoTypeEnum::TxPdoStandard);

  /*!
   * startup procedure
   * Configures the PDO types
   * The PDO types can be changed later via the ELmo::configureHardware method
   * this method gets called by the bus
   * there is no need to call this manually
   *
   * @return	true on success
   */
  bool startup() override;

  /*!
   * Read from the device via PDO
   * This method is called via the bus.
   * Do not call this method manually, sending frequencies should be constant.
   */
  void updateRead() override;

  /*!
   * Write to the device via PDO
   * This method is called via the bus
   * Do not call this method manually, sending frequencies should be constant.
   * if goToOperationEnabled is true, the commands are overwritten such that the
   * drive undergoes the necessary state transitions to reach the operational
   * state.
   */
  void updateWrite() override;

  // shutdown the slave
  void shutdown() override;

  // configure the Pdo
  // check the kind of Pdo
  bool configureRxPdo(const RxPdoTypeEnum rxPdoTypeEnum);
  bool configureTxPdo(const TxPdoTypeEnum txPdoTypeEnum);

  /*!
   * Makes the slave go to the desired drive state
   * @param DriveState	the desired drive state which should be reached by the
   * hardware
   * @param waitForDriveStateChange	true if the method should only exit once
   * the desired state has been reached (default)
   * @return	true if the desired state has been reached (or if the state
   * change is not checked), false if during the desired state has not been
   * reached during the maximal waiting period
   */
  bool setDriveStateViaPdo(const DriveState& targetDriveState, const bool waitForDriveStateChange = true);

  /*!
   * This is the state machine for the state transitions during PDO
   * communication This only works if the PDOs contain Statuswords and
   * Controlwords respectively.
   * @return	The necessary controlword
   */
  Controlword getNextStateTransitionControlword(const DriveState& requestedDriveState, const DriveState& currentDriveState);

  /*!
   * @brief	sets the Command to be sent to the hardware.
   * This also initializes the unit conversin factors
   * @param[in] command	The Command to be sent
   */
  void setCommand(const Command& command);

  /*!
   * @brief	return a Reading
   * This also sets the unit conversion factors
   * @param[out] reading	The reading to be returned
   */
  void getReading(Reading& reading);

  // small get / set methods
  // get the name of the slave
  std::string getName() const override { return name_; }

  // get the current Pdo Types (required because of atomic status)
  RxPdoTypeEnum getCurrentRxPdoTypeEnum() const { return currentRxPdoTypeEnum_; }
  TxPdoTypeEnum getCurrentTxPdoTypeEnum() const { return currentTxPdoTypeEnum_; }

  // pure virtual methods, must be overriden
  // currently unnused
  PdoInfo getCurrentPdoInfo() const override { return pdoInfo_; }

  /*!
   * Return the stateChangeSuccessful_ variable.
   * @return true if the last PDO drive state change was successful.
   */
  bool getLastPdoStateChangeSuccess() const { return stateChangeSuccessful_; }

  /*!
   * automatically configures the pdo size variables in the PdoInfo object to be
   * the same size as the size required by the hardware. At the moment this
   * would not be necessary for the soem_interface library. This is just to be
   * on the safe side. This method gets called during the configuration of the
   * slave via the ElmoEthercat Object
   */
  void autoConfigurePdoSizes();

  uint16_t getTxPdoSize();
  uint16_t getRxPdoSize();
  void setTxPdoSize(uint16_t size);
  void setRxPdoSize(uint16_t size);

  /*!
   * Adds an Error to the Reading object
   * This is necessary to be able to track Errors during startup procedure which
   * were caused by an other object
   */
  void addErrorToReading(const ErrorType& errorType);

  /*!
   * @brief	Set this object's configuration parameters
   * The parameters are loaded from a Configuration object.
   * @param[in] configuration	the Configuration with the requested
   * configuration parameters
   */
  void configureElmoEthercatSlave(const Configuration& configuration);

  /*!
   * @brief	Return the active Configuration object
   * This is useful for debug and to check whether the parsing of the
   * configuration file resulted in the requested behaviour
   * @return	The active Configuration
   */
  Configuration getActiveConfiguration();

 private:
  // private methods
  /*!
   * @brief	Drive state handling during PDO communication
   * This method should never be used manually. It is called automatically
   * during a call to "updateWrite" if a state transition is necessary.
   */
  void engagePdoStateMachine();
  // Variables
  //
  // The name of the slave
  const std::string name_;  // to be set in constructor

  // PdoTypeEnum, as atomic for thread safety
  std::atomic<RxPdoTypeEnum> rxPdoTypeEnum_;
  std::atomic<TxPdoTypeEnum> txPdoTypeEnum_;
  std::atomic<RxPdoTypeEnum> currentRxPdoTypeEnum_;
  std::atomic<TxPdoTypeEnum> currentTxPdoTypeEnum_;

  // mutexes
  std::recursive_mutex mutex_;

  // Command object
  Command command_;

  // Reading
  Reading reading_;

  PdoInfo pdoInfo_;

  /*! Will be true after the first updateRead has occured
   * This is needed because some buses will write first and then read
   * only manipulate this inside of a lock guard (locking mutex_)!
   */
  bool hasRead_{false};

  /*!
   * This flag indicates whether a change of the drive state should be conducted
   * during PDO sending phase only manipulate this inside of a lock guard
   * (locking mutex_)!
   */
  bool conductStateChange_{false};

  /*!
   * This is the target drive state
   * when the "conductStateChange" variable is set to true, the PDO state
   * machine will try to reach the target state only manipulate this inside of a
   * lock guard (locking mutex_)!
   */
  DriveState targetDriveState_{DriveState::NA};

  /*!
   * This is the controlword which will be sent to the hardware
   * This is stored in a separate variable because if no state change is
   * conducted, the last controlword should be sent. the updateWrite method
   * cannot be made static because different instances of "ElmoEthercatSlave"
   * will probably be acting differently at the same time only manipulate this
   * inside of a lock guard (locking mutex_)!
   */
  Controlword controlword_;

  /*!
   * This is the Configuration object used for the unit conversions when setting
   * a new command. Loading this Configuration from a file rather than
   * hardcoding it is recommended.
   */
  Configuration configuration_;

  /*!
   * This is the time point of the last pdo drive state change
   * It is needed because changing the drive states too quickly results in a bad
   * behaviour
   */
  std::chrono::time_point<std::chrono::steady_clock> driveStateChangeTimePoint_;

  /*!
   * This is the number of times a desired state has been reached.
   * Only if this value is above a predefined threshold the state change is
   * called successful. The reason for this is an unstable behaviour of the
   * elmos, mostly during startup procedure only manipulate this inside of a
   * lock guard (locking mutex_)!
   */
  uint16_t numberOfSuccessfulTargetStateReadings_{0};

  /*!
   * indicates whether a state change was successful or not
   * This is used to prevent changing states before the previous state
   * transition finished which could create a mess in the state mahine This
   * variable must be set to false whenever a state change is initialized. It
   * should only be set to true insied of the "engagePdoStateMachine" method
   * only manipulate this inside of a lock guard (locking mutex_)!
   */
  bool stateChangeSuccessful_{false};

  /*!
   * Configuration parameters
   * These parameters can be set with a Configuration object
   */
  unsigned int driveStateChangeMinTimeout_{20000};
  unsigned int driveStateChangeMaxTimeout_{300000};
  unsigned int minNumberOfSuccessfulTargetStateReadings_{10};
  bool printErrorMessage_{true};
  ModeOfOperationEnum modeOfOperation_{ModeOfOperationEnum::NA};
  bool allowModeChange_{false};

};  // class ElmoEthercatSlave

// alias for the shared pointer to the ElmoEthercatSlave pointer
using ElmoEthercatSlavePtr = std::shared_ptr<ElmoEthercatSlave>;

}  // namespace ethercat
}  // namespace elmo
