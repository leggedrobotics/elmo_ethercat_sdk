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

#include <signal_handler/SignalHandler.hpp>

#include <elmo_ethercat/ElmoEthercatMaster.hpp>

#define STATE_CHANGE_TIMEOUT 10000   // microseconds
#define CONFIGURATION_TIMEOUT 10000  // microseconds

namespace elmo {
namespace ethercat {

ElmoEthercatMaster::ElmoEthercatMaster(const bool standalone, const bool installSignalHandler, const double timeStep)
    : standalone_(standalone), installSignalHandler_(installSignalHandler), timeStep_(timeStep) {
  if (installSignalHandler_) {
    signal_handler::SignalHandler::bindAll(&ElmoEthercatMaster::handleSignal, this);
  }
}

ElmoEthercatMaster::~ElmoEthercatMaster() {
  if (!hasShutdown_) {
    shutdown();
  }
  if (installSignalHandler_) {
    signal_handler::SignalHandler::unbindAll(&ElmoEthercatMaster::handleSignal, this);
  }
}

bool ElmoEthercatMaster::update() {
  for (auto& elmodrive : elmodrives_) {
    elmodrive->updateSendStagedCommand();
  }
  bus_->updateWrite();

  bus_->updateRead();
  for (auto& elmodrive : elmodrives_) {
    elmodrive->updateProcessReading();
  }
  return true;
}

soem_interface::EthercatBusBase* ElmoEthercatMaster::getBus() {
  return bus_.get();
}

bool ElmoEthercatMaster::loadSetup(const std::string& setupFile) {
  bool success = true;

  // Load the setup from the file.
  YAML::Node yamlNode;
  try {
    yamlNode = YAML::LoadFile(setupFile);
  } catch(...) {
    MELO_FATAL_STREAM("ElmoEthercatMaster: Could not parse YAML file '" << setupFile << "'");
  }

  if (yamlNode["elmodrives"].IsDefined()) {
    // Clear the elmodrives first.
    elmodrives_.clear();

    const YAML::Node elmoSetup = yamlNode["elmodrives"];

    // Setup bus.
    if (elmoSetup[0]["interface_name"].IsDefined()) {
      const std::string networkInterface = elmoSetup[0]["interface_name"].as<std::string>();
      bus_ = EthercatBusPtr_t(new soem_interface::EthercatBusBase(networkInterface));
    } else {
      MELO_ERROR_STREAM("EtherCAT bus could not be setup.");
      return false;
    }

    // Add all slaves to bus.
    for (size_t i = 0; i < elmoSetup.size(); i++) {
      // Get address of slave
      if (elmoSetup[i]["ethercat_address"].IsDefined()) {
        int address = elmoSetup[i]["ethercat_address"].as<int>();
        // Get name of slave
        if (elmoSetup[i]["name"].IsDefined()) {
          std::string nameOfDrive = elmoSetup[i]["name]"].as<std::string>();
          ElmoEthercatPtr elmodrivePtr = std::make_shared<ElmoEthercat>();
          elmo::ethercat::ElmoEthercatSlavePtr slavePtr = std::make_shared<ElmoEthercatSlave>(nameOfDrive, getBus(), address);
          bus_->addSlave(slavePtr);
          elmodrivePtr->setSlavePointer(slavePtr);
          addElmodrive(elmodrivePtr);

        } else {
          MELO_ERROR_STREAM("Elmo: " << i << "name not defined");
        }
      } else {
        MELO_ERROR_STREAM("Elmo: " << i << " had no entry: ethercat_address");
      }
    }

    // Startup master.
    success &= startup(true);

    // Configure slave drives.
    for (size_t i = 0; i < elmoSetup.size(); i++) {
      success &= elmodrives_[i]->setDriveStateViaSdo(DriveState::OperationEnabled);

      std::string configurationFile;
      if (elmoSetup[i]["configuration_file"].IsDefined()) {
        configurationFile = elmoSetup[i]["configuration_file"].as<std::string>();
        if (configurationFile.empty()) {
          MELO_FATAL_STREAM("The path to the configuration file is empty.");
          success = false;
        } else if (configurationFile.front() == '/') {
          // Path to the configuration file is absolute, we can use it as is.
        } else if (configurationFile.front() == '~') {
          // Path to the configuration file is absolute, we need to replace '~' with
          // the home directory.
          const char* homeDirectory = getenv("HOME");
          if (homeDirectory == nullptr) {
            MELO_FATAL_STREAM("Environment variable 'HOME' could not be evaluated.");
            success = false;
          }
          configurationFile.erase(configurationFile.begin());
          configurationFile = homeDirectory + configurationFile;
        } else {
          // Path to the configuration file is relative, we need to append it to the
          // path of the setup file.
          configurationFile = setupFile.substr(0, setupFile.find_last_of("/") + 1) + configurationFile;
        }
      }

      success &= elmodrives_[i]->loadConfigFile(configurationFile);
    }
    setAllSlavesInit();

    // Shutdown bus.
    bus_->shutdown();
  } else {
    MELO_INFO_STREAM("No ELMO drive setup found.");
    success = false;
  }
  return success;
}

bool ElmoEthercatMaster::elmodriveExists(const std::string& name) const {
  for (const auto& elmodrive : elmodrives_) {
    if (elmodrive->getName() == name) {
      return true;
    }
  }
  return false;
}

ElmoEthercatPtr ElmoEthercatMaster::getElmodrive(const std::string& name) const {
  for (auto& elmodrive : elmodrives_) {
    if (elmodrive->getName() == name) {
      return elmodrive;
    }
  }
  return ElmoEthercatPtr();
}

ElmoEthercatMaster::Elmodrives ElmoEthercatMaster::getElmodrives() const {
  return elmodrives_;
}

unsigned int ElmoEthercatMaster::getNumberOfElmodrives() const {
  return elmodrives_.size();
}

bool ElmoEthercatMaster::startup(const bool isConfigRun) {
  hasShutdown_ = false;
  // Start the bus without checking the PDO sizes
  bool success = bus_->startup(false);

  // Activate distributed clock synchronization for all Elmos
  for (auto& elmodrive : elmodrives_) {
    std::string elmo_hw_version(64, 'N');
    elmodrive->sendSdoRead<std::string>(0x1009, 0, false, elmo_hw_version);
    MELO_INFO_STREAM("Elmo hardware version: " << elmo_hw_version);

    std::string elmo_sw_version(33, 'N');
    elmodrive->sendSdoRead<std::string>(0x100A, 0, false, elmo_sw_version);
    MELO_INFO_STREAM("Elmo software version: " << elmo_sw_version);

    bus_->syncDistributedClock0(elmodrive->getAddressOfSlave(), true, timeStep_, timeStep_ / 2.f);
  }

  if (isConfigRun) {
    // pre operational EtherCAT state is required for PDO mapping
    setAllSlavesPreOp();
  } else {
    setAllSlavesOperational();
    if (standalone_) {
      success &= startStandaloneUpdateWorker();
    }
    isRunning_ = true;
  }

  return success;
}

void ElmoEthercatMaster::setAllSlavesOperational(const bool waitForState) {
  bus_->setState(EC_STATE_SAFE_OP);
  if (waitForState) {
    bus_->waitForState(EC_STATE_SAFE_OP, 0, 50, 0.05);
  }
  usleep(STATE_CHANGE_TIMEOUT);

  bus_->setState(EC_STATE_OPERATIONAL);
  if (waitForState) {
    bus_->waitForState(EC_STATE_OPERATIONAL, 0, 50, 0.05);
  }
  usleep(STATE_CHANGE_TIMEOUT);
}

void ElmoEthercatMaster::setAllSlavesPreOp(const bool waitForState) {
  // Assuming init state at the moment.
  bus_->setState(EC_STATE_PRE_OP);
  if (waitForState) {
    bus_->waitForState(EC_STATE_PRE_OP, 0, 50, 0.05);
  }
  usleep(STATE_CHANGE_TIMEOUT);
}

void ElmoEthercatMaster::setAllSlavesInit(const bool waitForState) {
  bus_->setState(EC_STATE_INIT);
  if (waitForState) {
    bus_->waitForState(EC_STATE_INIT, 0, 50, 0.05);
  }
  usleep(STATE_CHANGE_TIMEOUT);
}

void ElmoEthercatMaster::setDriveStateForAllSlaves(const elmo::DriveState& driveState, const bool waitForState) {
  for (const auto& elmodrive : elmodrives_) {
    if (!elmodrive->setDriveStateViaPdo(driveState, waitForState))
      MELO_ERROR_STREAM("Drive state could not be set:\t" << elmodrive->getNameOfSlave());
  }
}

bool ElmoEthercatMaster::allDrivesAreInTheState(const elmo::DriveState driveState) const {
  if (getNumberOfElmodrives() == 0) {
    return false;
  }

  for (const auto& elmodrive : elmodrives_) {
    if (elmodrive->getReading().getDriveState() != driveState) {
      return false;
    }
  }
  return true;
}

bool ElmoEthercatMaster::allDrivesChangedStateSuccessfully() const {
  if (getNumberOfElmodrives() == 0) {
    return false;
  }

  for (const auto& elmodrive : elmodrives_) {
    if (!elmodrive->getLastPdoStateChangeSuccess()) {
      return false;
    }
  }
  return true;
}

void ElmoEthercatMaster::shutdown() {
  if (standalone_) {
    updateWorker_->stop(true);
  }

  bus_->setState(EC_STATE_INIT);
  bus_->shutdown();
  hasShutdown_ = true;
  isRunning_ = false;
}

bool ElmoEthercatMaster::isRunning() const {
  // std::lock_guard<std::recursive_mutex> lock(isRunningMutex_);
  return isRunning_;
}

bool ElmoEthercatMaster::updateWorkerCb(const any_worker::WorkerEvent& event) {
  return update();
}

bool ElmoEthercatMaster::addElmodrive(const ElmoEthercatPtr& elmodrivePtr) {
  const std::string name = elmodrivePtr->getName();
  if (elmodriveExists(name)) {
    MELO_ERROR_STREAM("Cannot add ELMO drive with name '" << name << "', because it already exists.");
    return false;
  }
  elmodrives_.push_back(elmodrivePtr);
  return true;
}

bool ElmoEthercatMaster::startStandaloneUpdateWorker() {
  any_worker::WorkerOptions updateWorkerOptions;
  updateWorkerOptions.callback_ = std::bind(&ElmoEthercatMaster::updateWorkerCb, this, std::placeholders::_1);
  updateWorkerOptions.defaultPriority_ = 90;
  updateWorkerOptions.name_ = "ElmoEthercatMaster::updateWorker";
  updateWorkerOptions.timeStep_ = timeStep_;
  updateWorkerOptions.enforceRate_ = true;
  updateWorker_.reset(new any_worker::Worker(updateWorkerOptions));
  return updateWorker_->start();
}

void ElmoEthercatMaster::handleSignal(const int signum) {
  MELO_INFO_STREAM("Received signal (" << signum << "), requesting shutdown ...");
  shutdown();
  if (signum == SIGSEGV) {
    // standard handling of the segmentation fault signal (SIGSEGV)
    signal(signum, SIG_DFL);
    kill(getpid(), signum);
  }
}

}  // namespace ethercat
}  // namespace elmo
