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
 * @file csv_example_standalone.cpp
 * @brief Example of using the elmo_ethercat_sdk with a standalone master and
 * velocity commands.
 *
 * One or multiple drives can be used for this example.
 * All drives will execute the same velocity profile (a sine wave with constant
 * amplitude and constant frequency).
 * Change **SINE_PERIOD** and **SINE_AMPLITUDE** according to the
 * possibilities of your test setup.
 *
 * @warning For safety reasons: **Double check your configuration files.**
 * This will prevent the connected motors from spinning in an
 * uncontrolled manner.
 */

#include <atomic>
#include <cmath>
#include <csignal>
#include <string>

#include <elmo_ethercat/ElmoEthercat.hpp>
#include <elmo_ethercat/ElmoEthercatMaster.hpp>

#define _USE_MATH_DEFINES

#define SINE_PERIOD 4.0             /// Period of the sine profile the drives will follow [s].
#define SINE_AMPLITUDE (12 * M_PI)  /// max motor speed [rad/s]

/*!
 * Interrupt signal flag.
 * This is set to true if an interrupt signal (e.g. Ctrl-c)
 * is sent to the process.
 */
std::atomic<bool> sigint{false};

/*!
 * The path to the config file
 */
std::string setupFile;

namespace elmo {
namespace ethercat {

/*!
 * CSV example function in standalone mode.
 * All connected motors will exhibit a sinusoidal motion.
 */
void csvExample() {
  /*
   * STEP 1)
   * Generate an EtherCAT Master object.
   * It creates all necessary objects according to a config file.
   * In standalone mode it also times and executes the actual EtherCAT
   * communication.
   */
  const bool standalone = true;
  const bool installSignalHandler = false;  // Signals are handled manually
  const double timeStep = 0.0025;
  ElmoEthercatMaster master(standalone, installSignalHandler, timeStep);

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
  if (!master.loadSetup(setupFile)) {
    MELO_ERROR_STREAM("ElmoEthercat Master could not load setup parameters.");
  }

  /*
   * STEP 3)
   * Create a command object.
   */
  Command command;

  /*
   * STEP 4)
   * Get the pointers to the ElmoEthercat objects created by the ElmoEthercatMaster in accordance to the config file.
   */
  std::vector<ElmoEthercatPtr> elmodrives = master.getElmodrives();

  /*
   * STEP 4.1) (optional)
   * If a specific command should be sent as soon as the communication starts
   * then this must be done now.
   * The default values of the Command object are all zero.
   */
  // Torque mode (only works if USE_MULTIPLE_MODE_OF_OPERATIONS is true in the config file.
  command.setModeOfOperation(ModeOfOperationEnum::CyclicSynchronousTorqueMode);
  command.setTargetTorque(0);

  for (const auto& elmodrive : elmodrives) {
    // stage the same command for all drives
    elmodrive->stageCommand(command);
  }

  /*
   * STEP 5)
   * Start the master now.
   * The master will start to send commands to the drives and receive readings from them
   * with a frequency of 1/timeStep (400Hz in this case).
   */
  if (!master.startup()) {
    MELO_ERROR_STREAM("ElmoEthercat Master could not be started.");
  }

  /*
   * STEP 6)
   * The drives can now be used.
   * No specific and/or constant frequency needs to be used to stage new commands because
   * the ElmoEthercatMaster operates in standalone mode and sends new commands over the
   * EtherCAT connection at a constant frequency. If no new command has been staged then
   * the last one staged is simply sent again.
   *
   * The order in which things are done is not that important in standalone mode.
   * The following should however be considered:
   * - create a new command first and then stage it.
   *
   * Note: DO NOT CALL ElmoEthercatMaster::update() yourself if the master is in standalone mode!
   */

  double newVelocity = 0.0;
  int i = 0;
  // drives will follow the sine wave velocity profile until Ctrl-c is pressed on the keyboard
  while (!sigint) {
    // Define new command.
    newVelocity = SINE_AMPLITUDE * sin(2 * M_PI / SINE_PERIOD * 0.01 * i);
    /*
     * Set mode of operation.
     * This needs to be done if USE_MULTIPLE_MODE_OF_OPERATIONS is set to true.
     */
    command.setModeOfOperation(ModeOfOperationEnum::CyclicSynchronousVelocityMode);
    command.setTargetVelocity(newVelocity);

    // Stage new command.
    for (const auto& elmodrive : elmodrives) {
      elmodrive->stageCommand(command);
    }

    /*
     * All drives need to be set into the "OperationEnabled" state in order for them to actually
     * start processing the commands.
     */
    static bool operationInitiallyEnabled = false;
    if (!operationInitiallyEnabled) {
      /*
       * "true" flag: wait here until drive changes were successful.
       * This only works in standalone mode!
       * The state changes will be conducted for all drives SEQUENTIALLY, not at the same time!
       * The drives will stay in the OperationEnabled state until some fault occurs.
       */
      master.setDriveStateForAllSlaves(elmo::DriveState::OperationEnabled, true);
      operationInitiallyEnabled = true;
    }

    // Used for printing
    static int counter = 0;
    const size_t printingPeriodMultiplier = 10;  // only print every 10th iteration

    // Alert user if the drive state is not OperationEnabled (i.e. a fault occurred)
    if (!master.allDrivesAreInTheState(elmo::DriveState::OperationEnabled)) {
      MELO_ERROR_STREAM("standalone csv example: DRIVE STATE ERROR!");

      // Process and print readings if all drive states are OperationEnabled
    } else {
      counter++;
      if (counter == printingPeriodMultiplier) {
        for (const auto& elmodrive : elmodrives) {
          // Extract the readings
          const double voltage = elmodrive->getReading().getBusVoltage();
          const double current = elmodrive->getReading().getActualCurrent();
          const double torque = elmodrive->getReading().getActualTorque();
          const double velocity = elmodrive->getReading().getActualVelocity();

          // Print the readings.
          MELO_INFO_STREAM("ELMO drive reading of '" << elmodrive->getNameOfSlave() << "':");
          MELO_INFO_STREAM("  Voltage:     " << voltage << " V");
          MELO_INFO_STREAM("  Current:     " << current << " A");
          MELO_INFO_STREAM("  Torque:      " << torque << " Nm");
          MELO_INFO_STREAM("  Velocity:    " << velocity << " rad/s");
        }
        counter = 0;
      }
    }

    // New commands are set approx. 100x per second
    usleep(10000);
    i++;
  }

  sigint = true;

  /*
   * STEP 7)
   * Shut the ElmoEthercatMaster down.
   * The resetting of the EtherCAT states is done automatically
   */
  MELO_INFO_STREAM("Shutting down the EtherCAT connection...");
  master.shutdown();
}

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
 * The CSV example is started after a 3 second countdown.
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

  // Run example
  elmo::ethercat::csvExample();
}
