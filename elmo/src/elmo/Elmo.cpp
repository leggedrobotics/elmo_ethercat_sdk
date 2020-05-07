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

#include <elmo/Elmo.hpp>

namespace elmo {

void Elmo::stageCommand(const Command& command) {
  // lock for thread safety
  std::lock_guard<std::recursive_mutex> lock(stagedCommandMutex_);

  // overwrite the staged command object
  // TODO(duboisf) implement time handling here or in Command class
  stagedCommand_ = command;

  // we have a staged Command
  commandIsStaged_ = true;
}
Command Elmo::getStagedCommand() const {
  std::lock_guard<std::recursive_mutex> lock(stagedCommandMutex_);
  return stagedCommand_;
}

}  // namespace elmo
