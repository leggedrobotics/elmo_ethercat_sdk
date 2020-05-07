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

/*!
 * @file cst_example.cpp
 * @brief Example of using the elmo_ethercat_sdk with torque
 * commands in a single controller/EtherCAT update loop.
 *
 * One or multiple drives can be used for this example.
 * All Drives will execute the same sinusoidal torque profile
 * Change **SINE\_PERIOD** and **SINE\_AMPLITUDE** according
 * to the possibilities of your test setup.
 *
 * @warning For safety reasons: **Double check your configuration files.**
 * This will prevent the connected motors from spinning in an
 * uncontrolled manner.
 */

#include <atomic>
#include <cmath>
#include <csignal>
#include <memory>
#include <string>

#include <elmo_ethercat/ElmoEthercat.hpp>
#include <elmo_ethercat/ElmoEthercatMaster.hpp>

#include <any_worker/Worker.hpp>

#define _USE_MATH_DEFINES

/// Period of the sinusoidal torque profile the drives will follow [s].
#define SINE_PERIOD 10.0

/// Torque amplitude [Nm].
#define SINE_AMPLITUDE 2.0

/*!
 * Interrupt signal flag.
 * This is set to true if an interrupt signal (e.g. Ctrl-c)
 * is sent to the process.
 */
std::atomic<bool> sigint{false};

/*!
 * The path to the config file.
 */
std::string setupFile;

namespace elmo {
namespace ethercat {

/*!
 * Time Step of the EtherCAT / control loop.
 */
const double timeStep = 0.0025;

/*!
 * std::vector of shared pointers to elmo::ethercat::ElmoEthercat.
 */
std::vector<ElmoEthercatPtr> elmodrives;

/*!
 * The EtherCAT Master (shared pointer to elmo::ethercat::ElmoEthercatMaster)
 */
std::shared_ptr<ElmoEthercatMaster> master;

/*!
 * Command object.
 */
Command command;

/*!
 * Counter variable.
 * Used to generate new commands and for printing.
 * Only relevant for this example
 */
unsigned int counter = 0;

/*!
 * Controller Update Function.
 * This function is executed at a constant rate by the Anyworker.
 * Other alternatives are available.
 * @note Simply using a `usleep` statement for timing is not precise enough.
 */
bool updateController(const any_worker::WorkerEvent& event) {
  static bool operationInitiallyEnabled = false;

  /*
   * STEP 7)
   * Setting the mode of operation.
   * This is not required if the correct mode of operation is set in the hardware
   * configuration file AND USE_MULTIPLE_MODE_OF_OPERATIONS is set to false.
   * Tip: Always add this line (with the required mode) for safety reasons.
   */
  command.setModeOfOperation(ModeOfOperationEnum::CyclicSynchronousTorqueMode);

  /*
   * STEP 8)
   * Bring the drives into operation enabled mode.
   * Several iterations of this loop will be needed for a successful
   * state change.
   * Notice: master->setDriveStateForAllSlaves is called with "false" as
   * second argument. This prevents this call from blocking.
   */
  if (!operationInitiallyEnabled) {
    master->setDriveStateForAllSlaves(DriveState::OperationEnabled, false);
    operationInitiallyEnabled = true;
  }

  /*
   * STEP 9)
   * Only start controlling the drives once the "Operation Enabled" state has
   * been reached.
   */
  if (master->allDrivesChangedStateSuccessfully()) {
    /*
     * STEP 10)
     * At every loop iteration check whether all drives are still in "Operation
     * Enabled" state. If they suddenly are in an other drive state then an
     * error has occurred. Automatically setting the drives back to "Operation
     * Enabled" state after the occurrence of an error is not recommended.
     */
    if (!master->allDrivesAreInTheState(DriveState::OperationEnabled)) {
      MELO_ERROR_STREAM(
          "CST example: Fault occurred, not al slaves are in "
          "operation enabled mode");
    } else {
      /*
       * STEP 11)
       * Extract the readings and use the results for your control actions.
       * In this example the readings will only be printed.
       */
      if (counter % 10 == 0) {
        for (const auto& elmodrive : elmodrives) {
          const double current = elmodrive->getReading().getActualCurrent();
          const double torque = elmodrive->getReading().getActualTorque();
          const double velocity = elmodrive->getReading().getActualVelocity();

          MELO_INFO_STREAM("ELMO drive reading of \"" << elmodrive->getNameOfSlave() << "\":");
          MELO_INFO_STREAM("  Current:     " << current << " A");
          MELO_INFO_STREAM("  Torque:      " << torque << " Nm");
          MELO_INFO_STREAM("  Velocity:    " << velocity << " rad/s");
        }
      }

      /*
       * STEP 12)
       * Get new torque commands and stage them
       * Note: All drives will be given the same torque command in this example.
       */
      double targetTorque = SINE_AMPLITUDE * sin(2 * M_PI / SINE_PERIOD * timeStep * counter);
      command.setTargetTorque(targetTorque);
      for (const auto& elmodrive : elmodrives) {
        elmodrive->stageCommand(command);
      }
    }
  }

  /*
   * STEP 13)
   * The EtherCAT communication must be updated over the master at every
   * iteration of this loop independently of the current state of the drives.
   */
  master->update();
  counter++;
  return true;
}  // updateController

/*!
 * CST setup example function (non-standalone mode).
 */
void cstExampleSetup() {
  /*
   * STEP 1)
   * Generate an EtherCAT Master object.
   * It is used to create all necessary objects according to a config file, to
   * start and to stop the EtherCAT communication.
   */
  const bool standalone = false;
  const bool installSignalHandler = false;  // Signals are handled manually
  master = std::make_shared<ElmoEthercatMaster>(standalone, installSignalHandler, timeStep);

  /*
   * STEP 2)
   * load the hardware setup.
   * Notice: The configuration is split into two parts: one global config file
   * (referenced by std::string setupFile) and a drive specific config file for
   * each connected drive.
   * Notice: This will open a connection on the network card and data
   * will be sent over the network.
   * The connection is closed again after the configuration.
   */
  if (!master->loadSetup(setupFile)) {
    MELO_ERROR_STREAM("ElmoEthercat Master could not load setup parameters.");
  }

  /*
   * STEP 3)
   * Get the pointers to the ElmoEthercat objects created by the
   * ElmoEthercatMaster in accordance to the config file.
   */
  elmodrives = master->getElmodrives();

  /*
   * STEP 4)
   * Configure the controller update loop.
   * This example uses the Anyworker for that purpose.
   */
  // Anyworker config object:
  any_worker::WorkerOptions updateWorkerOptions;
  // Bind the controller update loop
  updateWorkerOptions.callback_ = std::bind(&updateController, std::placeholders::_1);

  updateWorkerOptions.defaultPriority_ = 90;
  updateWorkerOptions.name_ = "elmo_ethercat_sdk: CST example worker";
  updateWorkerOptions.timeStep_ = timeStep;
  updateWorkerOptions.enforceRate_ = true;

  // Anyworker governing the controller loop:
  any_worker::Worker updateWorker(updateWorkerOptions);

  /*
   * STEP 5)
   * Start the master.
   */
  master->startup();

  /*
   * STEP 6)
   * Start the controller loop
   */
  if (!updateWorker.start()) {
    MELO_ERROR_STREAM("CST example: Anyworker could not be started!");
  }

  /*
   * STEP 14)
   * Wait for the interrupt signal
   * Shutdown the Anyworker and then the master upon reception of the signal.
   */
  while (!sigint) {
    usleep(10000);
  }
  MELO_INFO_STREAM("Shutting down the EtherCAT communication...");
  updateWorker.stop(true);
  master->shutdown();

}  // cstExampleSetup()

}  // namespace ethercat
}  // namespace elmo

/*!
 * Handle the interrupt signal.
 * This enables the user to stop the program while still letting the bus
 * shut down properly. This enables restarting the program without problems.
 */
void handleSigint(int signal) {
  sigint = true;
}

/*!
 * Main function.
 * The CST example is started after a 3 second countdown.
 */
int main(int argc, char* argv[]) {
  /*
   * A 3 second countdown before starting anything.
   * This is enough time to grab your e-stop button. DO IT, errors can occur.
   */
  std::cout << "Starting in 3 seconds.\nGrab your emergency stop button!" << std::endl;
  for (int i = 3; i > 0; i--) {
    std::cout << "Starting in: " << i << "s" << std::endl;
    usleep(1000000);
  }

  signal(SIGINT, handleSigint);

  if (argc > 1) {
    setupFile = argv[1];
  } else {
    std::cout << "Pass path to config file as command line argument:\n" << argv[0] << " path/to/configfile.yaml" << std::endl;
    return 1;
  }

  // Run example setup
  elmo::ethercat::cstExampleSetup();
}
