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
 * @brief	This file contains the different Tx Pdo structs. Each struct must
 * contain a statusword_, or else the state changes won't work! Each struct can
 * contain either the actual torque or the actual current but not both.
 */
#pragma once

#include <cstdint>

namespace elmo {

/*!
 * Standard Tx Pdo type
 */
struct TxPdoStandard  // Size:
{
  // 0x1a03
  int32_t actualPosition_;
  uint32_t digitalInputs_;
  int32_t actualVelocity_;
  uint16_t statusword_;

  // 0x1a1d
  int16_t analogInput_;

  // 0x1a1f
  int16_t actualCurrent_;

  // 0x1a18
  uint32_t busVoltage_;

} __attribute__((packed));

/*!
 * CST Tx PDO type
 * Includes padding_ byte for firmware version 01.01.15.00 (october 2018)
 */
struct TxPdoCST {
  // 0x1A02
  int32_t actualPosition_;
  int16_t actualTorque_;
  uint16_t statusword_;
  int8_t modeOfOperationDisplay_;
  int8_t padding_;

  // 0x1A11
  int32_t actualVelocity_;
} __attribute__((packed));

}  // namespace elmo
