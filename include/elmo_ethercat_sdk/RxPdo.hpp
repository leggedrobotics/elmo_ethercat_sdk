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
 * @file	RxPdo.hpp
 * @brief	This file contains the PDOs which are sent to the hardware
 * (RxPdo) Note: each struct MUST contain the controlWord_ variable!
 */
#pragma once

#include <cstdint>

namespace elmo {

/*!
 * Standard Rx PDO type.
 * Includes a padding_ byte for firmware version 01.01.15.00 (october 2018)
 */
struct RxPdoStandard {
  // 0x1605
  // page 21 EtherCAT Application Manual
  int32_t targetPosition_;
  int32_t targetVelocity_;
  int16_t targetTorque_;
  uint16_t maxTorque_;
  uint16_t controlWord_;
  int8_t modeOfOperation_;

  int8_t padding_;

  // 0x1618
  int16_t torqueOffset_;

} __attribute__((packed));

/*!
 * CST Rx PDO type.
 * Includes a padding_ byte for firmware version 01.01.15.00 (october 2018)
 */
struct RxPdoCST {
  // 0x1602
  int16_t targetTorque_;
  uint16_t controlWord_;
  // 0x160B
  int8_t modeOfOperation_;
  int8_t padding_;
} __attribute__((packed));

}  // namespace elmo
