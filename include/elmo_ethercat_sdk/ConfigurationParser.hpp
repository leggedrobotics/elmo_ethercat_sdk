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

#include <yaml-cpp/yaml.h>
#include <message_logger/message_logger.hpp>
#include <string>

#include "elmo_ethercat_sdk/Configuration.hpp"

namespace elmo {

/*!
 * @brief	Read configuration data from a yaml file
 */
class ConfigurationParser {
 public:
  /*!
   * no default constructor
   */
  ConfigurationParser() = delete;

  /*!
   * @brief	Constructor
   * Using an absolute path may be preferable, depending on the way the final
   * program is being run.
   * @param filename	The path to the configuration file
   */
  explicit ConfigurationParser(const std::string& filename);

  /*!
   * @brief	Constructor
   * This enables the use of a yaml node instead of a yaml file for the
   * configuration. This is useful for the creation of nested config files
   * @param configNode	The yaml node containing the configuration data
   */
  explicit ConfigurationParser(YAML::Node configNode);

  /*!
   * @brief	return the configuration
   * The Configuration object is filled according to the specified configuration
   * file. Any missing parameters will be automatically filled with well tested
   * default values.
   * @return	A configured elmo::ethercat::Configuration object
   */
  Configuration getConfiguration() const;

 private:
  /*!
   * @brief	Parse the configuration data
   * @param configNode	yaml node containing the config data
   */
  void parseConfiguration(YAML::Node configNode);
  /*!
   * The Configuration object
   */
  Configuration configuration_;
};

}  // namespace elmo
