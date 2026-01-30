/*
 * Copyright (C) 2025 ROS-Industrial Consortium Asia Pacific
 * Advanced Remanufacturing and Technology Centre
 * A*STAR Research Entities (Co. Registration No. 199702110H)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef VDA5050_MASTER__STANDARD_NAMES_HPP_
#define VDA5050_MASTER__STANDARD_NAMES_HPP_

#include <string>

namespace vda5050_master {

/// \brief VDA5050 protocol version
const std::string Version = "v2";  // NOLINT

/// \brief VDA5050 interface name
const std::string InterfaceName = "rmf2";  // NOLINT

/// \brief Topic name for connection messages
const std::string ConnectionTopic = "connection";  // NOLINT

/// \brief Topic name for factsheet messages
const std::string FactsheetTopic = "factsheet";  // NOLINT

/// \brief Topic name for order messages
const std::string OrderTopic = "order";  // NOLINT

/// \brief Topic name for state messages
const std::string StateTopic = "state";  // NOLINT

/// \brief Topic name for instant actions messages
const std::string InstantActionsTopic = "instant_actions";  // NOLINT

/// \brief Topic name for visualization messages
const std::string VisualizationTopic = "visualization";  // NOLINT

/// \brief QoS level for connection messages
const int ConnectionQos = 1;

/// \brief QoS level for factsheet messages
const int FactsheetQos = 0;

/// \brief QoS level for order messages
const int OrderQos = 0;

/// \brief QoS level for state messages
const int StateQos = 0;

/// \brief QoS level for visualization messages
const int VisualizationQos = 0;

/// \brief QoS level for instant actions messages
const int InstantActionsQos = 0;

/// \brief Heartbeat interval for connection messages in seconds
const int ConnectionHeartbeatInterval = 15;

/// \brief Heartbeat interval for state messages in seconds
const int StateHeartbeatInterval = 30;

}  // namespace vda5050_master

#endif  // VDA5050_MASTER__STANDARD_NAMES_HPP_
