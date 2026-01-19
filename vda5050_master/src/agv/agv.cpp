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

#include "vda5050_master/agv/agv.hpp"

#include <utility>

#include "nlohmann/json.hpp"
#include "vda5050_core/logger/logger.hpp"
#include "vda5050_core/mqtt_client/mqtt_client_interface.hpp"
#include "vda5050_json_utils/serialization.hpp"
#include "vda5050_master/standard_names.hpp"

// ============================================================================
// Constructor / Destructor
// ============================================================================

AGV::AGV(
  const std::string& manufacturer, const std::string& serial_number,
  const std::string& broker_address, size_t max_queue_size, bool drop_oldest,
  int state_heartbeat_interval)
: manufacturer_(manufacturer),
  serial_number_(serial_number),
  agv_id_(manufacturer + "/" + serial_number),
  broker_address_(broker_address),
  state_heartbeat_interval_(state_heartbeat_interval),
  created_time_(Clock::now()),
  max_queue_size_(max_queue_size),
  drop_oldest_(drop_oldest)
{
  VDA5050_INFO("[AGV] Created AGV instance: {}", agv_id_);
}

AGV::~AGV()
{
  VDA5050_INFO("[AGV] Destroying AGV instance: {}", agv_id_);

  // Stop queue processor
  stop_queue_processor();

  // Cleanup heartbeat
  cleanup_heartbeat();

  VDA5050_INFO("[AGV] AGV instance destroyed: {}", agv_id_);
}

// ============================================================================
// Connection and Operational State
// ============================================================================

bool AGV::is_connected() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return connection_status_ == vda5050_types::ConnectionState::ONLINE;
}

vda5050_types::ConnectionState AGV::get_connection_status() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return connection_status_;
}

AGVState AGV::get_operational_state() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return operational_state_;
}

void AGV::set_connection_status(vda5050_types::ConnectionState status)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  auto old_status = connection_status_;
  connection_status_ = status;

  // When connection is lost, AGV becomes unavailable
  if (
    status == vda5050_types::ConnectionState::OFFLINE ||
    status == vda5050_types::ConnectionState::CONNECTIONBROKEN)
  {
    if (operational_state_ != AGVState::UNAVAILABLE)
    {
      operational_state_ = AGVState::UNAVAILABLE;
      VDA5050_INFO(
        "[AGV] Operational state changed to UNAVAILABLE for {} (connection {})",
        agv_id_,
        status == vda5050_types::ConnectionState::OFFLINE ? "OFFLINE"
                                                          : "CONNECTIONBROKEN");
    }
  }

  // Log connection status change
  if (old_status != status)
  {
    const char* status_str = "UNKNOWN";
    switch (status)
    {
      case vda5050_types::ConnectionState::ONLINE:
        status_str = "ONLINE";
        break;
      case vda5050_types::ConnectionState::OFFLINE:
        status_str = "OFFLINE";
        break;
      case vda5050_types::ConnectionState::CONNECTIONBROKEN:
        status_str = "CONNECTIONBROKEN";
        break;
    }
    VDA5050_INFO(
      "[AGV] Connection status changed to {} for {}", status_str, agv_id_);
  }
}

void AGV::set_operational_state(AGVState state)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  operational_state_ = state;

  const char* state_str = "UNKNOWN";
  switch (state)
  {
    case AGVState::STATE_UNKNOWN:
      state_str = "STATE_UNKNOWN";
      break;
    case AGVState::AVAILABLE:
      state_str = "AVAILABLE";
      break;
    case AGVState::UNAVAILABLE:
      state_str = "UNAVAILABLE";
      break;
    case AGVState::ERROR:
      state_str = "ERROR";
      break;
  }
  VDA5050_INFO(
    "[AGV] Operational state changed to {} for {}", state_str, agv_id_);
}

void AGV::on_state_heartbeat_timeout()
{
  set_operational_state(AGVState::STATE_UNKNOWN);
  VDA5050_WARN("[AGV] State heartbeat timeout for {}", agv_id_);
}

// ============================================================================
// Heartbeat Management
// ============================================================================

void AGV::setup_heartbeat()
{
  if (state_heartbeat_)
  {
    return;  // Already set up
  }

  VDA5050_INFO("[AGV] Setting up heartbeat for {}", agv_id_);

  state_heartbeat_ =
    std::make_unique<vda5050_master::communication::HeartbeatListener>(
      agv_id_ + "_state_heartbeat", state_heartbeat_interval_,
      [this]() { on_state_heartbeat_timeout(); });
  state_heartbeat_->start_connection_heartbeat();
}

void AGV::cleanup_heartbeat()
{
  if (!state_heartbeat_)
  {
    return;  // Nothing to clean up
  }

  VDA5050_INFO("[AGV] Cleaning up heartbeat for {}", agv_id_);

  state_heartbeat_->stop_connection_heartbeat();
  state_heartbeat_.reset();
}

// ============================================================================
// Message Handlers
// ============================================================================

void AGV::handle_connection(const vda5050_types::Connection& msg)
{
  // Update cached message
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    last_connection_ = msg;
    last_connection_time_ = Clock::now();
  }

  // Update connection status
  set_connection_status(msg.connection_state);

  // Manage heartbeat based on connection state
  if (msg.connection_state == vda5050_types::ConnectionState::ONLINE)
  {
    // Start heartbeat and queue processor when ONLINE
    setup_heartbeat();
    start_queue_processor();
  }
  else
  {
    // Stop heartbeat and queue processor when OFFLINE/CONNECTIONBROKEN
    cleanup_heartbeat();
    stop_queue_processor();
  }
}

void AGV::handle_state(const vda5050_types::State& msg)
{
  // Update cached message and order progress
  {
    std::lock_guard<std::mutex> lock(data_mutex_);

    // Capture old state for event detection
    auto old_state = last_state_;

    // Update cached state
    last_state_ = msg;
    last_state_time_ = Clock::now();

    // Update order progress and detect events
    update_order_progress(old_state, msg);
  }

  // Notify heartbeat listener
  if (state_heartbeat_)
  {
    state_heartbeat_->received_connection();
  }

  // Update operational state to AVAILABLE
  set_operational_state(AGVState::AVAILABLE);
}

void AGV::handle_factsheet(const vda5050_types::Factsheet& msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  last_factsheet_ = msg;
  last_factsheet_time_ = Clock::now();
}

void AGV::handle_visualization(const vda5050_types::Visualization& msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  last_visualization_ = msg;
  last_visualization_time_ = Clock::now();
}

// ============================================================================
// Cached Messages - Get
// ============================================================================

std::optional<vda5050_types::Connection> AGV::get_last_connection() const
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  return last_connection_;
}

std::optional<vda5050_types::State> AGV::get_last_state() const
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  return last_state_;
}

std::optional<vda5050_types::Factsheet> AGV::get_last_factsheet() const
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  return last_factsheet_;
}

std::optional<vda5050_types::Visualization> AGV::get_last_visualization() const
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  return last_visualization_;
}

// ============================================================================
// Timestamps
// ============================================================================

std::optional<AGV::TimePoint> AGV::get_last_connection_time() const
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  return last_connection_time_;
}

std::optional<AGV::TimePoint> AGV::get_last_state_time() const
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  return last_state_time_;
}

std::optional<AGV::TimePoint> AGV::get_last_factsheet_time() const
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  return last_factsheet_time_;
}

std::optional<AGV::TimePoint> AGV::get_last_visualization_time() const
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  return last_visualization_time_;
}

// ============================================================================
// Order Tracking
// ============================================================================

std::optional<vda5050_types::Order> AGV::get_current_order() const
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  return current_order_;
}

void AGV::set_current_order(const vda5050_types::Order& order)
{
  std::lock_guard<std::mutex> lock(data_mutex_);

  // Check if this is an order update to an existing tracked progress
  bool is_order_update =
    current_progress_.has_value() &&
    current_progress_->order_id == order.order_id &&
    order.order_update_id > current_progress_->order_update_id;

  // Store the exact order message as sent (no modifications)
  current_order_ = order;

  // Add to order history
  order_history_[order.order_id].push_back(order);

  if (is_order_update)
  {
    // Order update: append new nodes/edges to full path in progress
    // The first node in order.nodes is the stitching node (last base node of old order)
    // Skip it when appending since it already exists in full_path
    VDA5050_INFO(
      "[AGV] Order update for {}: order_id={}, update_id={} -> {}", agv_id_,
      order.order_id, current_progress_->order_update_id,
      order.order_update_id);

    // Append all nodes after the stitching node (skip order.nodes[0])
    for (size_t i = 1; i < order.nodes.size(); ++i)
    {
      current_progress_->full_path_nodes.push_back(order.nodes[i]);
    }

    // Append all new edges from the update
    for (const auto& edge : order.edges)
    {
      current_progress_->full_path_edges.push_back(edge);
    }

    // Update progress metadata
    current_progress_->order_update_id = order.order_update_id;
    current_progress_->total_nodes = current_progress_->full_path_nodes.size();
    current_progress_->total_edges = current_progress_->full_path_edges.size();
    current_progress_->confirmed = false;  // Reset confirmation for new update
    current_progress_->completed = false;  // Reset completed for new update

    VDA5050_INFO(
      "[AGV] Full path now has {} total nodes, {} total edges",
      current_progress_->total_nodes, current_progress_->total_edges);
  }
  else
  {
    // Fresh order: create new progress with this order's nodes/edges as initial full path
    VDA5050_INFO(
      "[AGV] New order for {}: order_id={}, update_id={}, nodes={}, edges={}",
      agv_id_, order.order_id, order.order_update_id, order.nodes.size(),
      order.edges.size());

    OrderProgress progress;
    progress.order_id = order.order_id;
    progress.order_update_id = order.order_update_id;
    progress.confirmed = false;

    // Initialize full path with this order's nodes/edges
    progress.full_path_nodes = order.nodes;
    progress.full_path_edges = order.edges;

    progress.completed_nodes = 0;
    progress.total_nodes = order.nodes.size();
    progress.current_node_id = "";
    progress.current_node_sequence_id = 0;
    progress.completed_edges = 0;
    progress.total_edges = order.edges.size();
    progress.current_edge_id = std::nullopt;
    progress.current_edge_sequence_id = std::nullopt;
    progress.driving = false;
    progress.completed = false;

    current_progress_ = progress;
  }
}

std::optional<AGV::OrderProgress> AGV::get_order_progress() const
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  return current_progress_;
}

std::vector<vda5050_types::Order> AGV::get_order_history(
  const std::string& order_id) const
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  auto it = order_history_.find(order_id);
  if (it != order_history_.end())
  {
    return it->second;
  }
  return {};
}

const std::map<std::string, std::vector<vda5050_types::Order>>&
AGV::get_all_order_history() const
{
  // Note: Caller should not hold reference across mutex boundaries
  // This returns a const reference for read-only access
  return order_history_;
}

void AGV::update_order_progress(
  const std::optional<vda5050_types::State>& old_state,
  const vda5050_types::State& new_state)
{
  // Note: Caller must hold data_mutex_

  // If no progress being tracked, nothing to update
  if (!current_progress_.has_value())
  {
    return;
  }

  auto& progress = current_progress_.value();

  // Verify state matches tracked order
  if (new_state.order_id != progress.order_id)
  {
    return;
  }

  // Check if AGV has confirmed acceptance (state order_update_id >= our order_update_id)
  bool was_confirmed = progress.confirmed;
  progress.confirmed = (new_state.order_update_id >= progress.order_update_id);

  // Update progress based on state (using full_path for totals)
  progress.total_nodes = progress.full_path_nodes.size();
  progress.completed_nodes =
    progress.total_nodes - new_state.node_states.size();
  progress.current_node_id = new_state.last_node_id;
  progress.current_node_sequence_id = new_state.last_node_sequence_id;

  progress.total_edges = progress.full_path_edges.size();
  progress.completed_edges =
    progress.total_edges - new_state.edge_states.size();
  progress.driving = new_state.driving;

  // Current edge (if driving)
  if (new_state.driving && !new_state.edge_states.empty())
  {
    progress.current_edge_id = new_state.edge_states.front().edge_id;
    progress.current_edge_sequence_id =
      new_state.edge_states.front().sequence_id;
  }
  else
  {
    progress.current_edge_id = std::nullopt;
    progress.current_edge_sequence_id = std::nullopt;
  }

  // Check if order is completed (no base nodes remaining, no edges remaining)
  bool all_base_done = true;
  for (const auto& ns : new_state.node_states)
  {
    if (ns.released)
    {
      all_base_done = false;
      break;
    }
  }
  progress.completed = all_base_done && new_state.edge_states.empty();

  // Detect order confirmation (AGV accepted our order)
  if (progress.confirmed && !was_confirmed)
  {
    VDA5050_INFO(
      "[AGV] {} confirmed order acceptance: order_id={}, update_id={}", agv_id_,
      progress.order_id, progress.order_update_id);
  }

  // Event detection: compare with old state
  if (old_state.has_value() && old_state->order_id == progress.order_id)
  {
    const auto& old = old_state.value();

    // Detect node reached
    if (new_state.last_node_sequence_id != old.last_node_sequence_id)
    {
      VDA5050_INFO(
        "[AGV] {} reached node: id={}, seq={}", agv_id_, new_state.last_node_id,
        new_state.last_node_sequence_id);
    }

    // Detect nodes completed (nodes removed from node_states)
    if (new_state.node_states.size() < old.node_states.size())
    {
      size_t nodes_completed =
        old.node_states.size() - new_state.node_states.size();
      VDA5050_INFO(
        "[AGV] {} completed {} node(s), progress: {}/{}", agv_id_,
        nodes_completed, progress.completed_nodes, progress.total_nodes);
    }

    // Detect edges completed (edges removed from edge_states)
    if (new_state.edge_states.size() < old.edge_states.size())
    {
      size_t edges_completed =
        old.edge_states.size() - new_state.edge_states.size();
      VDA5050_INFO(
        "[AGV] {} completed {} edge(s), progress: {}/{}", agv_id_,
        edges_completed, progress.completed_edges, progress.total_edges);
    }

    // Detect driving state change
    if (
      new_state.driving && !old.driving && progress.current_edge_id.has_value())
    {
      VDA5050_INFO(
        "[AGV] {} started traversing edge: id={}, seq={}", agv_id_,
        progress.current_edge_id.value(),
        progress.current_edge_sequence_id.value());
    }
    else if (!new_state.driving && old.driving)
    {
      VDA5050_INFO(
        "[AGV] {} stopped driving, now at node: {}", agv_id_,
        new_state.last_node_id);
    }

    // Detect order completion (reached last base node, no more base nodes/edges)
    if (
      progress.completed &&
      (!old.node_states.empty() || !old.edge_states.empty()))
    {
      VDA5050_INFO(
        "[AGV] {} reached last base node, order ready for completion or "
        "update: order_id={}, update_id={}",
        agv_id_, progress.order_id, progress.order_update_id);

      // Notify queue processor that AGV may be ready for a new order
      queue_cv_.notify_one();
    }
  }
}

bool AGV::can_accept_new_order(const vda5050_types::Order& new_order) const
{
  std::lock_guard<std::mutex> lock(data_mutex_);

  // If no order being tracked, AGV can accept any order
  if (!current_progress_.has_value())
  {
    return true;
  }

  const auto& progress = current_progress_.value();

  // If new order is an update to current order (same order_id), allow it
  if (new_order.order_id == progress.order_id)
  {
    return true;
  }

  // Otherwise, check if current order is complete (all base nodes done)
  if (!last_state_.has_value())
  {
    VDA5050_WARN(
      "[AGV] {} cannot accept new order '{}': no state received yet, "
      "still executing order '{}'",
      agv_id_, new_order.order_id, progress.order_id);
    return false;
  }

  const auto& state = last_state_.value();

  // If state doesn't match tracked order, something is off
  if (state.order_id != progress.order_id)
  {
    VDA5050_WARN(
      "[AGV] {} cannot accept new order '{}': state order_id '{}' doesn't "
      "match tracked order '{}'",
      agv_id_, new_order.order_id, state.order_id, progress.order_id);
    return false;
  }

  // Check if all base nodes are done and no edges remaining
  for (const auto& ns : state.node_states)
  {
    if (ns.released)
    {
      VDA5050_WARN(
        "[AGV] {} cannot accept new order '{}': still executing order '{}', "
        "base node '{}' (seq={}) remaining",
        agv_id_, new_order.order_id, progress.order_id, ns.node_id,
        ns.sequence_id);
      return false;
    }
  }

  if (!state.edge_states.empty())
  {
    VDA5050_WARN(
      "[AGV] {} cannot accept new order '{}': still executing order '{}', "
      "{} edge(s) remaining",
      agv_id_, new_order.order_id, progress.order_id, state.edge_states.size());
    return false;
  }

  // No base nodes remaining and no edges - order is complete
  return true;
}

// ============================================================================
// Outgoing Messages - Queue
// ============================================================================

bool AGV::send_order(const vda5050_types::Order& order)
{
  std::lock_guard<std::mutex> lock(queue_mutex_);

  if (order_queue_.size() >= max_queue_size_)
  {
    if (!drop_oldest_)
    {
      VDA5050_WARN(
        "[AGV] Dropping new order: queue full ({}/{}) for {}",
        order_queue_.size(), max_queue_size_, agv_id_);
      return false;
    }
    // Drop oldest order to make room
    VDA5050_WARN(
      "[AGV] Dropping oldest order: queue full ({}/{}) for {}",
      order_queue_.size(), max_queue_size_, agv_id_);
    order_queue_.pop();
  }

  order_queue_.push(order);
  queue_cv_.notify_one();

  VDA5050_INFO("[AGV] Queued order for AGV: {}", agv_id_);
  return true;
}

bool AGV::send_instant_actions(const vda5050_types::InstantActions& actions)
{
  std::lock_guard<std::mutex> lock(queue_mutex_);

  if (instant_actions_queue_.size() >= max_queue_size_)
  {
    if (!drop_oldest_)
    {
      VDA5050_WARN(
        "[AGV] Dropping new instant actions: queue full ({}/{}) for {}",
        instant_actions_queue_.size(), max_queue_size_, agv_id_);
      return false;
    }
    // Drop oldest instant actions to make room
    VDA5050_WARN(
      "[AGV] Dropping oldest instant actions: queue full ({}/{}) for {}",
      instant_actions_queue_.size(), max_queue_size_, agv_id_);
    instant_actions_queue_.pop();
  }

  instant_actions_queue_.push(actions);
  queue_cv_.notify_one();

  VDA5050_INFO("[AGV] Queued instant actions for AGV: {}", agv_id_);
  return true;
}

// ============================================================================
// Queue Processing
// ============================================================================

void AGV::start_queue_processor()
{
  if (queue_processor_running_.load())
  {
    return;  // Already running
  }

  VDA5050_INFO("[AGV] Starting queue processor for {}", agv_id_);

  stop_processing_ = false;
  queue_processor_running_ = true;
  queue_thread_ = std::thread(&AGV::process_queues, this);
}

void AGV::stop_queue_processor()
{
  if (!queue_processor_running_.load())
  {
    return;  // Not running
  }

  VDA5050_INFO("[AGV] Stopping queue processor for {}", agv_id_);

  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    stop_processing_ = true;
  }
  queue_cv_.notify_one();

  if (queue_thread_.joinable())
  {
    queue_thread_.join();
  }

  queue_processor_running_ = false;
  VDA5050_INFO("[AGV] Queue processor stopped for {}", agv_id_);
}

void AGV::process_queues()
{
  VDA5050_INFO("[AGV] Queue processing thread started for {}", agv_id_);

  while (true)
  {
    std::optional<vda5050_types::Order> order;
    std::optional<vda5050_types::InstantActions> actions;

    {
      std::unique_lock<std::mutex> lock(queue_mutex_);

      // Wait for a message or stop signal
      queue_cv_.wait(lock, [this] {
        return stop_processing_ || !order_queue_.empty() ||
               !instant_actions_queue_.empty();
      });

      // Check if we should stop
      if (
        stop_processing_ && order_queue_.empty() &&
        instant_actions_queue_.empty())
      {
        break;
      }

      // Process instant actions first (higher priority)
      if (!instant_actions_queue_.empty())
      {
        actions = std::move(instant_actions_queue_.front());
        instant_actions_queue_.pop();
      }
      else if (!order_queue_.empty())
      {
        // Copy the order to check if AGV can accept it (need copy since we release lock)
        vda5050_types::Order pending_order = order_queue_.front();

        // Release queue lock to check acceptance (avoids lock ordering issues)
        lock.unlock();

        if (can_accept_new_order(pending_order))
        {
          // Re-acquire lock and pop the order
          lock.lock();
          if (!order_queue_.empty())
          {
            order = std::move(order_queue_.front());
            order_queue_.pop();
          }
        }
        else
        {
          // AGV not ready - wait for state update to signal completion
          // Don't pop from queue, just continue waiting
          continue;
        }
      }
    }

    // Publish the message (outside the lock)
    if (actions)
    {
      publish_instant_actions(*actions);
    }
    else if (order)
    {
      publish_order(*order);
    }
  }

  VDA5050_INFO("[AGV] Queue processing thread stopped for {}", agv_id_);
}

// ============================================================================
// Publishing via Transient MQTT Client
// ============================================================================

void AGV::publish_order(const vda5050_types::Order& order)
{
  if (broker_address_.empty())
  {
    VDA5050_WARN(
      "[AGV] Cannot publish order: no broker address for {}", agv_id_);
    return;
  }

  try
  {
    // Create transient client
    std::string client_id =
      agv_id_ + "_order_" +
      std::to_string(Clock::now().time_since_epoch().count());
    auto client = vda5050_core::mqtt_client::create_default_client(
      broker_address_, client_id);

    // Connect, publish, disconnect
    client->connect();

    nlohmann::json j;
    vda5050_types::to_json(j, order);
    client->publish(
      build_topic(vda5050_master::OrderTopic), j.dump(),
      vda5050_master::OrderQos);

    client->disconnect();

    // Start tracking the order (progress will be updated when AGV confirms via state)
    set_current_order(order);

    VDA5050_INFO("[AGV] Published order to AGV: {}", agv_id_);
  }
  catch (const std::exception& e)
  {
    VDA5050_WARN("[AGV] Failed to publish order for {}: {}", agv_id_, e.what());
  }
}

void AGV::publish_instant_actions(const vda5050_types::InstantActions& actions)
{
  if (broker_address_.empty())
  {
    VDA5050_WARN(
      "[AGV] Cannot publish instant actions: no broker address for {}",
      agv_id_);
    return;
  }

  try
  {
    // Create transient client
    std::string client_id =
      agv_id_ + "_actions_" +
      std::to_string(Clock::now().time_since_epoch().count());
    auto client = vda5050_core::mqtt_client::create_default_client(
      broker_address_, client_id);

    // Connect, publish, disconnect
    client->connect();

    nlohmann::json j;
    vda5050_types::to_json(j, actions);
    client->publish(
      build_topic(vda5050_master::InstantActionsTopic), j.dump(),
      vda5050_master::InstantActionsQos);

    client->disconnect();

    VDA5050_INFO("[AGV] Published instant actions to AGV: {}", agv_id_);
  }
  catch (const std::exception& e)
  {
    VDA5050_WARN(
      "[AGV] Failed to publish instant actions for {}: {}", agv_id_, e.what());
  }
}

// ============================================================================
// Helper Methods
// ============================================================================

std::string AGV::build_topic(const std::string& topic_name) const
{
  return vda5050_master::InterfaceName + "/" + vda5050_master::Version + "/" +
         manufacturer_ + "/" + serial_number_ + "/" + topic_name;
}
