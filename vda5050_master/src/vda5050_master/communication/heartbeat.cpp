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

#include "vda5050_master/communication/heartbeat.hpp"

#include "vda5050_core/logger/logger.hpp"

namespace vda5050_master {
namespace communication {

HeartbeatListener::HeartbeatListener(
  const std::string& id, const int heartbeat_interval,
  std::function<void()> disconnection_callback)
: id_(id),
  heartbeat_interval_(heartbeat_interval),
  state_(HeartbeatState::STOPPED),
  last_connection_report_(std::chrono::steady_clock::now()),
  disconnection_callback_(disconnection_callback)
{
  if (heartbeat_interval_ <= 0)
  {
    throw std::invalid_argument(
      "HeartbeatListener: heartbeat_interval must be positive");
  }
  if (!disconnection_callback_)
  {
    throw std::invalid_argument(
      "HeartbeatListener: disconnection_callback cannot be null");
  }
}

HeartbeatListener::~HeartbeatListener()
{
  stop_connection_heartbeat();
  VDA5050_INFO("[{}] Deconstructing HeartbeatListener", id_);
}

void HeartbeatListener::start_connection_heartbeat()
{
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (state_ != HeartbeatState::STOPPED)
    {
      VDA5050_WARN(
        "[{}] Cannot start heartbeat listener: not in STOPPED state", id_);
      return;
    }
    VDA5050_INFO("Starting Connection heartbeat listener");
    state_ = HeartbeatState::RUNNING;
  }
  // Reset the last connection report time when starting
  {
    std::lock_guard<std::mutex> lock(last_connection_report_mutex_);
    last_connection_report_ = get_current_time();
  }
  connection_thread_ = std::thread(&HeartbeatListener::listen, this);
}

void HeartbeatListener::received_connection()
{
  // Check state with proper synchronization
  if (get_state() != HeartbeatState::RUNNING)
  {
    VDA5050_DEBUG("Connection heartbeat not running, ignored...");
    return;
  }
  std::lock_guard<std::mutex> lock(last_connection_report_mutex_);
  last_connection_report_ = get_current_time();
  VDA5050_INFO("[{}] Received connection heartbeat", id_);
  {
    std::lock_guard<std::mutex> lock(check_lock_);
    message_received_.notify_all();
  }
}

std::chrono::steady_clock::time_point
HeartbeatListener::get_last_connection_report() const
{
  std::lock_guard<std::mutex> lock(last_connection_report_mutex_);
  return last_connection_report_;
}

HeartbeatState HeartbeatListener::get_state() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return state_;
}

void HeartbeatListener::stop_connection_heartbeat()
{
  VDA5050_INFO("Stopping Connection heartbeat listener");

  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    state_ = HeartbeatState::STOPPING;
  }

  {
    std::lock_guard<std::mutex> lock(check_lock_);
    message_received_.notify_all();
  }

  if (connection_thread_.joinable())
  {
    connection_thread_.join();
  }
  else
  {
    VDA5050_INFO("Connection thread not joinable");
  }

  if (callback_thread_.joinable())
  {
    callback_thread_.join();
  }

  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    state_ = HeartbeatState::STOPPED;
  }

  VDA5050_INFO("Stopped Connection heartbeat listener");
}

std::chrono::steady_clock::time_point HeartbeatListener::get_current_time()
{
  return std::chrono::steady_clock::now();
}

int HeartbeatListener::get_check_interval()
{
  return heartbeat_interval_;
}

bool HeartbeatListener::is_stop_requested()
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return state_ == HeartbeatState::STOPPING;
}

bool HeartbeatListener::is_timeout()
{
  std::chrono::steady_clock::time_point current_time = get_current_time();
  int time_since_last_connection_report;
  {
    std::lock_guard<std::mutex> lock(last_connection_report_mutex_);
    time_since_last_connection_report =
      std::chrono::duration_cast<std::chrono::seconds>(
        current_time - last_connection_report_)
        .count();
  }

  const int interval = get_check_interval();
  if (std::abs(time_since_last_connection_report) > interval)
  {
    VDA5050_WARN(
      "[{}] Connection heartbeat timeout after {} seconds (max: {}s)", id_,
      time_since_last_connection_report, interval);
    return true;
  }
  return false;
}

void HeartbeatListener::listen()
{
  while (!is_stop_requested())
  {
    {
      std::unique_lock<std::mutex> lock(check_lock_);
      message_received_.wait_for(lock, std::chrono::seconds(wait_seconds));
    }

    // Check if shutdown was requested while waiting
    if (is_stop_requested())
    {
      VDA5050_DEBUG("[{}] Shutdown requested, exiting listen loop", id_);
      return;
    }

    if (is_timeout())
    {
      // Copy callback by value to ensure safe invocation
      auto callback_copy = disconnection_callback_;
      callback_thread_ = std::thread([callback_copy]() { callback_copy(); });
      if (callback_thread_.joinable())
      {
        callback_thread_.join();
      }
      VDA5050_INFO("[{}] Heartbeat monitoring stopped after timeout", id_);
      return;
    }
  }
}

}  // namespace communication
}  // namespace vda5050_master
