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

#ifndef VDA5050_MASTER__COMMUNICATION__HEARTBEAT_HPP_
#define VDA5050_MASTER__COMMUNICATION__HEARTBEAT_HPP_

#include <chrono>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <string>
#include <thread>

namespace vda5050_master {
namespace communication {

/**
 * @brief Heartbeat listener lifecycle states
 *
 * State transitions:
 *   STOPPED -> RUNNING (via start_connection_heartbeat())
 *   RUNNING -> STOPPING (via stop_connection_heartbeat())
 *   STOPPING -> STOPPED (when cleanup completes)
 */
enum class HeartbeatState
{
  STOPPED,  // Not running, safe to destroy or restart
  RUNNING,  // Actively monitoring heartbeats
  STOPPING  // Stop in progress, cleanup ongoing
};

class HeartbeatListener
{
public:
  /**
   * @brief Construct a HeartbeatListener
   * @param id Identifier for logging
   * @param heartbeat_interval Timeout interval in seconds (must be positive)
   * @param disconnection_callback Callback invoked on timeout (cannot be null)
   * @throws std::invalid_argument if heartbeat_interval <= 0 or callback is null
   */
  HeartbeatListener(
    const std::string& id, const int heartbeat_interval,
    std::function<void()> disconnection_callback);

  ~HeartbeatListener();

  // Non-copyable, non-movable (due to thread and mutex members)
  HeartbeatListener(const HeartbeatListener&) = delete;
  HeartbeatListener& operator=(const HeartbeatListener&) = delete;
  HeartbeatListener(HeartbeatListener&&) = delete;
  HeartbeatListener& operator=(HeartbeatListener&&) = delete;

  /**
   * @brief Start the heartbeat monitoring thread
   */
  void start_connection_heartbeat();

  /**
   * @brief Stop the heartbeat monitoring thread
   */
  void stop_connection_heartbeat();

  /**
   * @brief Signal that a heartbeat was received, resetting the timeout
   */
  void received_connection();

  /**
   * @brief Get the timestamp of the last received heartbeat
   * @return Time point of last heartbeat
   */
  std::chrono::steady_clock::time_point get_last_connection_report() const;

  /**
   * @brief Get the current heartbeat listener state
   * @return HeartbeatState enum value
   */
  HeartbeatState get_state() const;

  /**
   * @brief Get the current time (virtual for testing)
   * @return Current steady_clock time point
   */
  virtual std::chrono::steady_clock::time_point get_current_time();

  /**
   * @brief Get the check interval (virtual for testing)
   * @return Heartbeat interval in seconds
   */
  virtual int get_check_interval();

protected:
  std::condition_variable message_received_;

private:
  bool is_stop_requested();
  bool is_timeout();
  void listen();

  std::string id_;
  std::thread connection_thread_;
  std::thread callback_thread_;
  int heartbeat_interval_;

  // Lifecycle state protected by state_mutex_
  HeartbeatState state_;
  std::chrono::steady_clock::time_point last_connection_report_;

  // Mutex for state variable
  mutable std::mutex state_mutex_;

  // Mutex to protect access to last_connection_report_
  mutable std::mutex last_connection_report_mutex_;

  // Mutex for condition variable wait
  std::mutex check_lock_;

  std::function<void()> disconnection_callback_;
};

}  // namespace communication
}  // namespace vda5050_master

#endif  // VDA5050_MASTER__COMMUNICATION__HEARTBEAT_HPP_
