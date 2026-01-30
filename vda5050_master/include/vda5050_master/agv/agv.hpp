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

#ifndef VDA5050_MASTER__AGV__AGV_HPP_
#define VDA5050_MASTER__AGV__AGV_HPP_

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <queue>
#include <string>
#include <thread>

#include "vda5050_execution/protocol_adapter.hpp"
#include "vda5050_master/communication/heartbeat.hpp"
#include "vda5050_master/standard_names.hpp"
#include "vda5050_master/vda5050_interfaces.hpp"

namespace vda5050_master {

// Forward declaration
class VDA5050Master;

/// \brief AGV operational state based on state heartbeat
enum class AGVState
{
  STATE_UNKNOWN,  // Initial state or state heartbeat timed out
  AVAILABLE,      // State heartbeat is being received, AGV operational
  UNAVAILABLE,    // AGV reported unavailable or connection lost
  ERROR           // AGV reported error state
};

/// \brief Represents an individual AGV managed by VDA5050Master
///
/// This class is primarily a data container that holds:
/// - Identity information (manufacturer, serial number)
/// - Cached VDA5050 messages (connection, state, factsheet, visualization)
/// - Connection and operational state
/// - Outgoing message queue for orders and instant actions
///
/// Additionally, this class handles ingoing/outgoing messages via the
/// VDA5050Execution ProtocolAdapter.
///
class AGV
{
public:
  // Type aliases
  using Clock = std::chrono::system_clock;
  using TimePoint = std::chrono::time_point<Clock>;

  /// \brief Construct an AGV instance
  ///
  /// \param protocol_adapter Protocol adapter for pub/sub mechanism creation
  /// \param manufacturer Manufacturer name
  /// \param serial_number Serial number
  /// \param max_queue_size Maximum number of outgoing messages to queue
  /// \param drop_oldest If true, drop oldest message when queue full; if false, reject new
  /// \param state_heartbeat_interval State heartbeat timeout in seconds
  AGV(
    std::shared_ptr<vda5050_execution::ProtocolAdapter> protocol_adapter,
    const std::string& manufacturer, const std::string& serial_number,
    size_t max_queue_size = 10, bool drop_oldest = true,
    int state_heartbeat_interval = StateHeartbeatInterval);

  /// \brief Destructor - stops the queue processing thread
  ~AGV();

  // Non-copyable, non-movable (due to thread member)
  AGV(const AGV&) = delete;
  AGV& operator=(const AGV&) = delete;
  AGV(AGV&&) = delete;
  AGV& operator=(AGV&&) = delete;

  // ============================================================================
  // Identity
  // ============================================================================

  /// \brief Get the manufacturer name
  const std::string& get_manufacturer() const
  {
    return manufacturer_;
  }

  /// \brief Get the serial number
  const std::string& get_serial_number() const
  {
    return serial_number_;
  }

  /// \brief Get the AGV ID (manufacturer/serial_number)
  const std::string& get_agv_id() const
  {
    return agv_id_;
  }

  // ============================================================================
  // Connection and Operational State
  // ============================================================================

  /// \brief Check if the AGV is connected (based on VDA5050 connection message)
  ///
  /// \return true if connection_status is ONLINE, false otherwise
  bool is_connected() const;

  /// \brief Get the AGV connection status (based on VDA5050 connection message)
  ///
  /// \return ONLINE, OFFLINE, or CONNECTIONBROKEN
  vda5050_types::ConnectionState get_connection_status() const;

  /// \brief Get the AGV operational state (based on state heartbeat)
  ///
  /// \return STATE_UNKNOWN, AVAILABLE, UNAVAILABLE, or ERROR
  AGVState get_operational_state() const;

  /// \brief Stop the AGV, releasing all runtime resources (heartbeat, queue processor)
  ///
  /// Stops the queue processor and heartbeat, resets connection and operational state,
  /// and clears all message queues. Cached messages are preserved.
  /// The AGV can be restarted later with start().
  void stop();

  /// \brief Restart the AGV, fully resetting state to accept new connections
  ///
  /// Calls stop(), then clears cached messages and timestamps.
  /// The heartbeat and queue processor will be started automatically when an
  /// ONLINE connection message is received.
  void restart();

  /// \brief Pause the AGV, suspending runtime resources without clearing queues
  ///
  /// Stops the queue processor and heartbeat, sets connection status to OFFLINE
  /// and operational state to UNAVAILABLE. Queued messages and cached data are
  /// preserved and will be processed when resumed.
  void pause();

  /// \brief Resume a paused AGV, restarting the queue processor and heartbeat
  ///
  /// Restarts the queue processor and heartbeat so the AGV can resume
  /// publishing queued messages and monitoring state heartbeats.
  void resume();

  // ============================================================================
  // Cached Messages (read-only access)
  // ============================================================================

  /// \brief Get the last received connection message
  ///
  /// \return Optional containing the message if received, nullopt otherwise
  std::optional<vda5050_types::Connection> get_last_connection() const;

  /// \brief Get the last received state message
  ///
  /// \return Optional containing the message if received, nullopt otherwise
  std::optional<vda5050_types::State> get_last_state() const;

  /// \brief Get the last received factsheet message
  ///
  /// \return Optional containing the message if received, nullopt otherwise
  std::optional<vda5050_types::Factsheet> get_last_factsheet() const;

  /// \brief Get the last received visualization message
  ///
  /// \return Optional containing the message if received, nullopt otherwise
  std::optional<vda5050_types::Visualization> get_last_visualization() const;

  // ============================================================================
  // Timestamps
  // ============================================================================

  /// \brief Get the time when the AGV was created
  ///
  /// \return Time at which the AGV instance was created
  TimePoint get_created_time() const
  {
    return created_time_;
  }

  /// \brief Get the time of the last received connection message
  ///
  /// \return Optional containing the timestamp if received, nullopt otherwise
  std::optional<TimePoint> get_last_connection_time() const;

  /// \brief Get the time of the last received state message
  ///
  /// \return Optional containing the timestamp if received, nullopt otherwise
  std::optional<TimePoint> get_last_state_time() const;

  /// \brief Get the time of the last received factsheet message
  ///
  /// \return Optional containing the timestamp if received, nullopt otherwise
  std::optional<TimePoint> get_last_factsheet_time() const;

  /// \brief Get the time of the last received visualization message
  ///
  /// \return Optional containing the timestamp if received, nullopt otherwise
  std::optional<TimePoint> get_last_visualization_time() const;

  // ============================================================================
  // Outgoing Messages
  // ============================================================================

  /// \brief Queue an order to be sent to this AGV
  ///
  /// \param order The order message
  ///
  /// \return true if queued successfully, false if queue is full (drop_oldest=false)
  bool send_order(const vda5050_types::Order& order);

  /// \brief Queue instant actions to be sent to this AGV
  ///
  /// \param actions The instant actions message
  ///
  /// \return true if queued successfully, false if queue is full (drop_oldest=false)
  bool send_instant_actions(const vda5050_types::InstantActions& actions);

  /// \brief Get the number of pending orders in the queue
  ///
  /// \return Number of orders waiting to be sent
  size_t get_pending_order_count() const;

  /// \brief Get the number of pending instant actions in the queue
  ///
  /// \return Number of instant actions waiting to be sent
  size_t get_pending_instant_actions_count() const;

  // ============================================================================
  // Message Handlers (called by VDA5050Master to route incoming messages)
  // ============================================================================

  /// \brief Handle an incoming connection message
  ///
  /// \param msg The parsed connection message
  void handle_connection(const vda5050_types::Connection& msg);

  /// \brief Handle an incoming state message
  ///
  /// \param msg The parsed state message
  void handle_state(const vda5050_types::State& msg);

  /// \brief Handle an incoming factsheet message
  ///
  /// \param msg The parsed factsheet message
  void handle_factsheet(const vda5050_types::Factsheet& msg);

  /// \brief Handle an incoming visualization message
  ///
  /// \param msg The parsed visualization message
  void handle_visualization(const vda5050_types::Visualization& msg);

private:
  // ============================================================================
  // Subscription Management
  // ============================================================================

  /// \brief Setup MQTT subscriptions for all VDA5050 message types via ProtocolAdapter
  void setup_subscriptions();

  // ============================================================================
  // Message Handlers (from MQTT callbacks)
  // ============================================================================

  /// \brief Create a subscription for a specific message type
  ///
  /// \tparam MsgType The VDA5050 message type to subscribe to
  /// \param agv_handler Pointer to member function that handles the message
  /// \param qos Quality of service for the subscription
  template <typename MsgType>
  void create_subscription(void (AGV::*agv_handler)(const MsgType&), int qos);

  // ============================================================================
  // Internal State Management
  // ============================================================================

  /// \brief Update the connection status
  ///
  /// \param status New connection status
  void set_connection_status(vda5050_types::ConnectionState status);

  /// \brief Update the operational state
  ///
  /// \param state New operational state
  void set_operational_state(AGVState state);

  /// \brief Callback invoked when state heartbeat times out
  void on_state_heartbeat_timeout();

  /// \brief Initialize and start the heartbeat listener
  void setup_heartbeat();

  /// \brief Stop and cleanup the heartbeat listener
  void cleanup_heartbeat();

  // ============================================================================
  // Queue Processing
  // ============================================================================

  /// \brief Start the background thread for processing message queues
  void start_queue_processor();

  /// \brief Stop the background queue processing thread
  void stop_queue_processor();

  /// \brief Main loop for the queue processing thread
  void process_queues();

  /// \brief Publish an order message via the protocol adapter
  ///
  /// \param order The order to publish
  void publish_order(const vda5050_types::Order& order);

  /// \brief Publish instant actions via the protocol adapter
  ///
  /// \param actions The instant actions to publish
  void publish_instant_actions(const vda5050_types::InstantActions& actions);

  // ============================================================================
  // Member Variables
  // ============================================================================

  /// \brief AGV Manufacturer name
  std::string manufacturer_;

  /// \brief AGV Serial Number
  std::string serial_number_;

  /// \brief AGV ID (manufacturer/serial_number)
  std::string agv_id_;

  /// \brief Protocol Adapter that sets up mqtt related communications
  std::shared_ptr<vda5050_execution::ProtocolAdapter> protocol_adapter_;

  /// \brief Mutex to protect heartbeat listener
  mutable std::mutex heartbeat_mutex_;

  /// \brief Heartbeat to monitor state messages
  std::unique_ptr<communication::HeartbeatListener> state_heartbeat_;

  /// \brief State heartbeat interval in seconds
  int state_heartbeat_interval_;

  /// \brief Mutex to protect connection and operational state
  mutable std::mutex state_mutex_;

  /// \brief AGV connection status (changed by incoming Connection messages, default OFFLINE)
  vda5050_types::ConnectionState connection_status_{
    vda5050_types::ConnectionState::OFFLINE};

  /// \brief AGV State (changed by incoming State messages, default STATE_UNKNOWN)
  AGVState operational_state_{AGVState::STATE_UNKNOWN};

  /// \brief Time when the AGV instance was created
  TimePoint created_time_;

  /// \brief Cached messages and timestamps (protected by data_mutex_)
  mutable std::mutex data_mutex_;

  /// \brief Last received connection message, if any
  std::optional<vda5050_types::Connection> last_connection_;

  /// \brief Time of last received connection message, if any
  std::optional<TimePoint> last_connection_time_;

  /// \brief Last received state message, if any
  std::optional<vda5050_types::State> last_state_;

  /// \brief Time of last received state message, if any
  std::optional<TimePoint> last_state_time_;

  /// \brief Last received factsheet message, if any
  std::optional<vda5050_types::Factsheet> last_factsheet_;

  /// \brief Time of last received factsheet message, if any
  std::optional<TimePoint> last_factsheet_time_;

  /// \brief Last received visualization message, if any
  std::optional<vda5050_types::Visualization> last_visualization_;

  /// \brief ime of last received visualization message, if any
  std::optional<TimePoint> last_visualization_time_;

  /// \brief Outgoing message queues (protected by queue_mutex_)
  size_t max_queue_size_;

  /// \brief Whether to drop oldest messages when queue is full
  bool drop_oldest_;

  /// \brief Mutex to protect outgoing message queues
  mutable std::mutex queue_mutex_;

  /// \brief Condition variable to notify queue processor thread to stop
  std::condition_variable queue_cv_;

  /// \brief Queue of orders to be sent
  std::queue<vda5050_types::Order> order_queue_;

  /// \brief Queue of instant actions to be sent
  std::queue<vda5050_types::InstantActions> instant_actions_queue_;

  /// \brief Queue processing thread
  std::mutex thread_mutex_;

  /// \brief Whether to stop processing the queues
  bool stop_processing_{false};

  /// \brief Whether the queue processor thread is running
  bool queue_processor_running_{false};

  /// \brief Thread for processing outgoing message queues
  std::thread queue_thread_;
};

}  // namespace vda5050_master

#endif  // VDA5050_MASTER__AGV__AGV_HPP_
