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
#include <iostream>

namespace elmo {

struct Controlword {
  bool switchOn_{false};              // bit 0
  bool enableVoltage_{false};         // bit 1
  bool quickStop_{false};             // bit 2
  bool enableOperation_{false};       // bit 3
  bool newSetPoint_{false};           // bit 4 profiled position mode
  bool homingOperationStart_{false};  // bit 4 homing mode
  bool changeSetImmediately_{false};  // bit 5 profiled position mode
  bool relative_{false};              // bit 6 profiled position mode
  bool faultReset_{false};            // bit 7
  bool halt_{false};                  // bit 8

  /*!
   * get the control word as a 16 bit unsigned integer
   * THIS DOES NOT RESPECT THE MODE SPECIFIC OPTIONS!
   * The usually used cyclic modes do not need mode specific options.
   * @return	the raw controlword
   */
  uint16_t getRawControlword();

  /*!
   * State transition 2
   * SWITCH ON DISABLED -> READY TO SWITCH ON
   * This corresponds to a "shutdown" Controlword
   */
  void setStateTransition2();

  /*!
   * State transition 3
   * READY TO SWITCH ON -> SWITCHED ON
   * This corresponds to a "switch on" Controlword
   */
  void setStateTransition3();

  /*!
   * State transition 4
   * SWITCHED ON -> ENABLE OPERATION
   */
  void setStateTransition4();

  /*!
   * State transition 5
   * OPERATION ENABLED -> SWITCHED ON
   * This corresponds to a "disable operation" Controlword
   */
  void setStateTransition5();

  /*!
   * State transition 6
   * SWITCHED ON -> READY TO SWITCH ON
   */
  void setStateTransition6();

  /*!
   * State transition 7
   * READY TO SWITCH ON -> SWITCH ON DISABLED
   */
  void setStateTransition7();

  /*!
   * State transition 8
   * OPERATION ENABLED -> READY TO SWITCH ON
   */
  void setStateTransition8();

  /*!
   * State transition 9
   * OPERATION ENABLED -> SWITCH ON DISABLED
   * This resets the elmo to the same state as on hardware startup
   * 0x0000
   */
  void setStateTransition9();

  /*!
   * State transition 10
   * SWITCHED ON -> SWITCH ON DISABLED
   * This Statusword is 0x0000
   */
  void setStateTransition10();

  /*!
   * State transition 11
   * OPERATION ENABLED -> QUICK STOP ACTIVE
   */
  void setStateTransition11();

  /*!
   * State transition 12
   * QUICK STOP ACTIVE -> SWITCH ON DISABLED
   */
  void setStateTransition12();

  /*!
   * State transition 15
   * FAULT -> SWITCH ON DISABLED
   */
  void setStateTransition15();

  /*!
   * Sets all bools of this struct to false
   */
  void setAllFalse();

  /*!
   * goes to the init state
   * Alias for state transition 2
   */
  void setInit();

  friend std::ostream& operator<<(std::ostream& os, const Controlword& controlword);
};

}  // namespace elmo
