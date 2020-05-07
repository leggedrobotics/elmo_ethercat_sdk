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

#include <cstdint>
#include <iostream>

#include <elmo/Command.hpp>
#include <elmo/Controlword.hpp>
#include <elmo/Reading.hpp>

int main() {
  uint16_t rawStatusword_ = 0b0100110001110011;
  elmo::Reading reading;
  reading.setStatusword(rawStatusword_);
  reading.setDigitalInputs(0b00110011001100110011001100110011);
  std::cout << reading << std::endl;

  elmo::Command command;

  command.setTargetVelocity(143);
  std::cout << std::endl;
  std::cout << command << std::endl;
}
