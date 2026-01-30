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

#ifndef VDA5050_MASTER__VDA5050_MASTER__MASTER_HPP_
#define VDA5050_MASTER__VDA5050_MASTER__MASTER_HPP_

#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <utility>

#include "vda5050_core/mqtt_client/mqtt_client_interface.hpp"
#include "vda5050_master/agv/agv.hpp"
#include "vda5050_master/vda5050_interfaces.hpp"

namespace vda5050_master {

/// \brief VDA5050 Master for multi-AGV fleet management
///
/// This class manages VDA5050 communication for multiple AGVs
/// using a single shared MQTT client that creates protocol adapters for each AGV.
///
/// Features:
/// - Single shared MQTT client that creates protocol adapters for each AGV
/// - AGV onboarding/offboarding
/// - Protocol adapters for subscribing and publishing to AGVs
/// - Querying of onboarded AGVs
/// - Thread-safe access to AGV instances
/// - Method to publish orders and instant actions to specific AGVs

class VDA5050Master
{
public:
  /// \brief Construct a VDA5050 master with shared MQTT client
  ///
  /// \param mqtt_client Shared MQTT client for subscriptions
  VDA5050Master(std::shared_ptr<vda5050_core::mqtt_client::MqttClientInterface>
                  mqtt_client);

  /// \brief Virtual destructor - disconnects MQTT client
  virtual ~VDA5050Master();

  // Non-copyable, non-movable
  VDA5050Master(const VDA5050Master&) = delete;
  VDA5050Master& operator=(const VDA5050Master&) = delete;
  VDA5050Master(VDA5050Master&&) = delete;
  VDA5050Master& operator=(VDA5050Master&&) = delete;

  // ============================================================================
  // Connection Management
  // ============================================================================

  /// \brief Connect the MQTT client
  void connect();

  /// \brief Disconnect the MQTT client
  void disconnect();

  /// \brief Check if MQTT client is connected
  bool is_connected() const;

  // ============================================================================
  // AGV Onboarding/Offboarding
  // ============================================================================

  /// \brief Onboard an AGV to allow message routing
  ///
  /// Creates an AGV instance that creates the ProtocolHandlers that will
  /// listen to the various topics required.
  ///
  /// \param manufacturer Manufacturer name
  /// \param serial_number Serial number
  /// \param max_queue_size Maximum number of outgoing messages to queue
  /// \param drop_oldest If true, drop oldest message when queue full; if false, reject new message
  void onboard_agv(
    const std::string& manufacturer, const std::string& serial_number,
    size_t max_queue_size = 10, bool drop_oldest = true);

  /// \brief Offboard an AGV to stop pub/sub
  ///
  /// \param manufacturer Manufacturer name
  /// \param serial_number Serial number
  void offboard_agv(
    const std::string& manufacturer, const std::string& serial_number);

  /// \brief Check if an AGV is onboarded
  ///
  /// \param manufacturer Manufacturer name
  /// \param serial_number Serial number
  ///
  /// \return true if AGV is onboarded
  bool is_agv_onboarded(
    const std::string& manufacturer, const std::string& serial_number) const;

  // ============================================================================
  // AGV Access
  // ============================================================================

  /// \brief Get a shared pointer to an onboarded AGV
  ///
  /// \param manufacturer Manufacturer name
  /// \param serial_number Serial number
  ///
  /// \return Shared pointer to AGV, or nullptr if not onboarded
  std::shared_ptr<AGV> get_agv(
    const std::string& manufacturer, const std::string& serial_number) const;

  // ============================================================================
  // Outgoing Messages
  // ============================================================================

  /// \brief Publish an order to a specific AGV
  ///
  /// \param manufacturer Manufacturer name
  /// \param serial_number Serial number
  /// \param order The order message
  ///
  /// \return true if queued successfully, false if queue is full
  ///
  /// \throws std::runtime_error if AGV is not onboarded
  bool publish_order(
    const std::string& manufacturer, const std::string& serial_number,
    const vda5050_types::Order& order);

  /// \brief Publish instant actions to a specific AGV
  ///
  /// \param manufacturer Manufacturer name
  /// \param serial_number Serial number
  /// \param actions The instant actions message
  ///
  /// \return true if queued successfully, false if queue is full
  ///
  /// \throws std::runtime_error if AGV is not onboarded
  bool publish_instant_actions(
    const std::string& manufacturer, const std::string& serial_number,
    const vda5050_types::InstantActions& actions);

private:
  // ============================================================================
  // Internal AGV lookup
  // ============================================================================

  /// \brief Look up an AGV by its combined ID
  ///
  /// \param agv_id The AGV ID in format "manufacturer/serial_number"
  ///
  /// \return Shared pointer to AGV, or nullptr if not found
  std::shared_ptr<AGV> get_agv_by_id(const std::string& agv_id) const;

  // ============================================================================
  // Member Variables
  // ============================================================================

  /// \brief Shared MQTT client for protocol adapters
  std::shared_ptr<vda5050_core::mqtt_client::MqttClientInterface> mqtt_client_;

  /// \brief Mutex protecting access to onboarded AGVs
  mutable std::mutex agv_mutex_;

  /// \brief Map of onboarded AGVs
  std::unordered_map<std::string, std::shared_ptr<AGV>> agvs_;
};

}  // namespace vda5050_master

#endif  // VDA5050_MASTER__VDA5050_MASTER__MASTER_HPP_
