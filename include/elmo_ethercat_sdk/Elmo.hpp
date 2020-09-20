#pragma once

#include "elmo_ethercat_sdk/Command.hpp"
#include"elmo_ethercat_sdk/DriveState.hpp"
#include "elmo_ethercat_sdk/Reading.hpp"
#include "elmo_ethercat_sdk/Controlword.hpp"

#include <ethercat_sdk_master/EthercatDrive.hpp>

#include <yaml-cpp/yaml.h>

#include <mutex>
#include <atomic>
#include <string>
#include <cstdint>
#include <chrono>

namespace elmo {
  class Elmo : public ecat_master::EthercatDrive{
    // pure virtual overwrites
    public:
      std::string getName() const override;
      bool startup() override;
      void shutdown() override;
      void updateWrite() override;
      void updateRead() override;
      PdoInfo getCurrentPdoInfo() const override { return pdoInfo_; }

    // virtual overwrites
    public:
      bool preopConfigurationRequired() override {return true;}
      bool clockSyncRequired() override {return true;}
      bool runPreopConfiguration() override;

    public:
      void stageCommand(const Command& command);
      Reading getReading() const;
      void getReading(Reading& reading) const;

      bool loadConfigFile(const std::string& fileName);
      bool loadConfigNode(YAML::Node configNode);
      bool loadConfiguration(const Configuration& configuration);
      Configuration getConfiguration() const;

    //SDO
    public:
      bool getStatuswordViaSdo(Statusword& statusword);
      bool setControlwordViaSdo(Controlword& controlword);
      bool setDriveStateViaSdo(const DriveState& driveState);
    protected:
      bool stateTransitionViaSdo(const StateTransition& stateTransition);

    // PDO
    public:
      bool setDriveStateViaPdo(const DriveState& driveState, const bool waitForState);
      bool lastPdoStateChangeSuccessful() const { return stateChangeSuccessful_; }

    protected:
      void engagePdoStateMachine();
      bool mapPdos(RxPdoTypeEnum rxPdoTypeEnum, TxPdoTypeEnum txPdoTypeEnum);
      Controlword getNextStateTransitionControlword(const DriveState& requestedDriveState,
                                                    const DriveState& currentDriveState);
      void autoConfigurePdoSizes();

      uint16_t getTxPdoSize();
      uint16_t getRxPdoSize();

    // Errors
    protected:
      void addErrorToReading(const ErrorType& errorType);



    protected:
      Command stagedCommand_;
      Reading reading_;
      std::string name_;
      Configuration configuration_;
      Controlword controlword_;
      PdoInfo pdoInfo_;
      bool hasRead_{false};
      bool conductStateChange_{false};
      DriveState targetDriveState_{DriveState::NA};
      std::chrono::time_point<std::chrono::steady_clock> driveStateChangeTimePoint_;
      uint16_t numberOfSuccessfulTargetStateReadings_{0};
      std::atomic<bool> stateChangeSuccessful_{false};


    // Configurable parameters
    protected:
      bool allowModeChange_{false};
      ModeOfOperationEnum modeOfOperation_{ModeOfOperationEnum::NA};
      // unsigned int sdoVerifyTimeout_{20000}; //microseconds
      // bool printDebugMessages_{true};
      // unsigned int driveStateChangeMinTimeout_{20000};
      // unsigned int driveStateChangeMaxTimeout_{300000};
      // unsigned int minNumberOfSuccessfulTargetStateReadings_{10};
      // ModeOfOperationEnum modeOfOperation_{ModeOfOperationEnum::NA};
      // bool allowModeChange_{false};

    protected:
      mutable std::recursive_mutex stagedCommandMutex_; //TODO required?
      mutable std::recursive_mutex readingMutex_; //TODO required?
      mutable std::recursive_mutex mutex_; // TODO: change name!!!!

  };
} // namespace elmo
