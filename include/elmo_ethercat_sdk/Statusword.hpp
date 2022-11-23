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
#include <iomanip>
#include <iostream>
#include <string>

#include "elmo_ethercat_sdk/DriveState.hpp"

namespace elmo {

class Statusword {
 private:
  bool readyToSwitchOn_{false};      // bit 0
  bool switchedOn_{false};           // bit 1
  bool operationEnabled_{false};     // bit 2
  bool fault_{false};                // bit 3
  bool voltageEnabled_{false};       // bit 4
  bool quickStop_{false};            // bit 5
  bool switchOnDisabled_{false};     // bit 6
  bool warning_{false};              // bit 7
  bool targetReached_{false};        // bit 10
  bool internalLimitActive_{false};  // bit 11
  bool followingError_{false};       // bit 13, CSV mode

  // the raw statusword
  uint16_t rawStatusword_{0};

 public:
  friend std::ostream& operator<<(std::ostream& os, const Statusword& statusword);
  void setFromRawStatusword(uint16_t status);
  DriveState getDriveState() const;
  std::string getDriveStateString() const;
};

}  // namespace elmo
