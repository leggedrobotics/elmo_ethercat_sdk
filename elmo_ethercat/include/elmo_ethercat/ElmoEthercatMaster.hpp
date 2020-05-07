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

#include <unistd.h>
#include <any_worker/Worker.hpp>
#include <atomic>
#include <functional>
#include <soem_interface/EthercatBusBase.hpp>
#include <string>
#include <unordered_map>

#include <elmo_ethercat/Configuration.hpp>
#include <elmo_ethercat/ElmoEthercat.hpp>
#include <elmo_ethercat/ElmoEthercatSlave.hpp>

namespace elmo {
namespace ethercat {

//! Class implementing an EtherCAT master for multiple Elmo drives.
class ElmoEthercatMaster {
 public:
  using EthercatBusPtr_t = std::unique_ptr<soem_interface::EthercatBusBase>;
  using Elmodrives = std::vector<ElmoEthercatPtr>;

  ElmoEthercatMaster() = delete;

  /*!
   * Constructor.
   * @param standalone  Flag for running in standalone mode
   * @param installSignalHandler  Flag indicating whether a signal handler shall
   * be installed.
   * @param timeStep	The timestep of the bus communication. This parameter
   * has to be provided even if the update cycle is handled by the user's code
   * (standalone = false) to ensure a working distributed clock synchronization.
   */
  ElmoEthercatMaster(const bool standalone, const bool installSignalHandler, const double timeStep);

  /*!
   * Destructor
   * Deletes the bus_ pointer if it wasn't created elsewhere
   */
  ~ElmoEthercatMaster();

  /*!
   * Update the ElmoEthercatMaster, calling all update steps sequentially.
   * If standalone == true, this method is called automatically.
   * @return True if successful.
   */
  bool update();

  /*!
   * expose the bus pointer
   * This is necessary to create various Slave objects
   * @return	pointer to the EthercatBusBase object
   */
  soem_interface::EthercatBusBase* getBus();

  /*!
   * Load the setup from a file.
   * @param setupFile Absolute path to the setup file.
   * @return True if successful.
   */
  bool loadSetup(const std::string& setupFile);

  /*!
   * Check if an Elmodrive with a given name exists.
   * @param name Name of the Elmo drive.
   * @return True if existing.
   */
  bool elmodriveExists(const std::string& name) const;

  /*!
   * Get an Elmo drive by name. If unsure, first check if the Elmo drive exists
   * with elmodriveExists(..).
   * @param name Name of the Elmo drive.
   * @return Pointer to the Elmo drive.
   */
  ElmoEthercatPtr getElmodrive(const std::string& name) const;

  /*!
   * Get all Elmo drive.
   * @return List of all Elmo drives.
   */
  Elmodrives getElmodrives() const;

  /*!
   * Get the number of Elmo drives.
   * @return Number of Elmo drives.
   */
  unsigned int getNumberOfElmodrives() const;

  /*!
   * @brief	starts the bus
   * All slaves are then set to the ethercat state "operational".
   * If no configuration has been done yet, the slaves are set to the ethercat
   * state "pre operational" which enables PDO mapping The clock synchronization
   * is also started
   * @param[in] isConfigRun	True if the bus is started for a config run.
   * @return	true on success
   */
  bool startup(const bool isConfigRun = false);

  /*!
   * puts all connected slaves in the operational EtherCAT state
   * @param waitForState	true if the program should wait until
   * the operational state is reached
   * This assumes that the current state is ProOp
   */
  void setAllSlavesOperational(const bool waitForState = true);

  /*!
   * puts all connected slaves in the pre op EtherCAT state
   * @param waitForState	true if the program should wait until
   * the pre op state is reached
   * This assumes that the current state is INIT
   */
  void setAllSlavesPreOp(const bool waitForState = true);

  /*!
   * puts all connected slaves in the init EtherCAT state
   * @param waitForState	true if the program should wait until
   * the init state is reached
   */
  void setAllSlavesInit(const bool waitForState = true);

  /*!
   * @brief	sets a drive desired state for all slaves
   * @param[in] driveState	The desired drive state
   * @param[in] waitForState	True (default) if the method shall return only
   * after the desired state has been reached (or after a maximum duration).
   * False if the method shall return immediately.
   */
  void setDriveStateForAllSlaves(const elmo::DriveState& driveState, const bool waitForState = true);

  /*!
   * Check if all drives are in a given state.
   * @param driveState State to check.
   * @return True if all drives are in the given state.
   */
  bool allDrivesAreInTheState(const elmo::DriveState driveState) const;

  /*!
   * Check if the last PDO(!) state change was successful for all slaves.
   * @return true if all state changes were successful.
   */
  bool allDrivesChangedStateSuccessfully() const;

  /*!
   * Shuts the bus (and thereby all the slaves) down
   */
  void shutdown();

  /*!
   * Check if the ElmoEthercat Bus Boss is running.
   * @return True if running.
   */
  bool isRunning() const;

 protected:
  /*!
   * Worker updating the ElmoEthercat Bus Boss if standalone is true.
   * @param event Worker event.
   * @return True if successful.
   */
  bool updateWorkerCb(const any_worker::WorkerEvent& event);

  /*!
   * Add an Elmodrive to the master.
   * @param elmodrive Elmodrive to add.
   * @return True if successful.
   */
  bool addElmodrive(const ElmoEthercatPtr& elmodrivePtr);

  /*!
   * Configures the updateWorker_ and starts it.
   * This function only gets called if "standlone_" is true
   * @return true on success
   */
  bool startStandaloneUpdateWorker();

  /*!
   * Signal callback function.
   * @param signum Signal code.
   */
  void handleSignal(const int signum);

  /*!
   * Flag for running in standalone mode.
   * True: A worker is instantiated, which calls update() in every time step.
   * False: update() has to be periodically called from the outside.
   */
  const bool standalone_{true};
  /*!
   * Flag indicating whether a shutdown has been requested.
   * True: handleSignal() is bound to signal handler.
   * False: default system signal handling is active.
   */
  const bool installSignalHandler_{true};

  //! Update worker.
  std::shared_ptr<any_worker::Worker> updateWorker_;

  //! Time step at which the update() function is called.
  const double timeStep_{0.0};

  //! Bus pointer.
  EthercatBusPtr_t bus_;

  std::atomic<bool> hasShutdown_{false};
  //! Flag indicating whether the master is running. True between startup() and
  //! shutdown() calls.
  std::atomic<bool> isRunning_{false};
  //! List of ELMO drives.
  Elmodrives elmodrives_;
};

}  // namespace ethercat
}  // namespace elmo
