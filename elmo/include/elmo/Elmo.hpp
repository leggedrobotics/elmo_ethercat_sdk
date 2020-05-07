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
#include <mutex>
#include <string>

#include <elmo/Command.hpp>
#include <elmo/DriveState.hpp>
#include <elmo/Reading.hpp>

namespace elmo {

class Elmo {
 public:
  Elmo() = default;
  virtual ~Elmo() = default;

  // pure virtual functions
  // they shall be implemented in a bus specific way
  virtual void startupWithCommunication() = 0;
  virtual void updateSendStagedCommand() = 0;
  virtual void updateProcessReading() = 0;
  virtual void shutdown() = 0;

  // get the name of this object
  std::string getName() { return name_; }

  // staging a Command
  void stageCommand(const Command& command);

  Command getStagedCommand() const;

 protected:
  // the name of this object
  std::string name_;

  // the staged command
  Command stagedCommand_;

  // is a command staged?
  std::atomic<bool> commandIsStaged_{false};

  // mutex for the staged command
  mutable std::recursive_mutex stagedCommandMutex_;

  // mutex for reading Pdos
  mutable std::recursive_mutex readingMutex_;
};

}  // namespace elmo
