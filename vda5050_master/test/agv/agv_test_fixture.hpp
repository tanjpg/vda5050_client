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

#ifndef AGV__AGV_TEST_FIXTURE_HPP_
#define AGV__AGV_TEST_FIXTURE_HPP_

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "vda5050_master/agv/agv.hpp"

namespace vda5050_master::test {

/**
 * @brief Common test fixture for AGV unit tests
 *
 * Provides helper methods for creating AGV instances and test messages.
 */
class AGVTestFixture : public ::testing::Test
{
protected:
  void SetUp() override
  {
    manufacturer_ = "TestManufacturer";
    serial_number_ = "SN001";
    agv_id_ = manufacturer_ + "/" + serial_number_;
  }

  void TearDown() override
  {
    agv_.reset();
  }

  std::unique_ptr<AGV>& create_agv()
  {
    // Use empty broker address for unit tests
    agv_ = std::make_unique<AGV>(manufacturer_, serial_number_, "");
    return agv_;
  }

  std::unique_ptr<AGV>& create_agv_with_heartbeat_interval(
    int state_heartbeat_interval)
  {
    // Use empty broker address for unit tests
    agv_ = std::make_unique<AGV>(
      manufacturer_, serial_number_, "", AGV::DEFAULT_MAX_QUEUE_SIZE, true,
      state_heartbeat_interval);
    return agv_;
  }

  vda5050_types::Connection create_connection_msg(const std::string& state)
  {
    vda5050_types::Connection msg;
    msg.header.header_id = 1;
    msg.header.timestamp = std::chrono::system_clock::now();
    msg.header.version = "2.0.0";
    msg.header.manufacturer = manufacturer_;
    msg.header.serial_number = serial_number_;

    if (state == "ONLINE")
    {
      msg.connection_state = vda5050_types::ConnectionState::ONLINE;
    }
    else if (state == "OFFLINE")
    {
      msg.connection_state = vda5050_types::ConnectionState::OFFLINE;
    }
    else
    {
      msg.connection_state = vda5050_types::ConnectionState::CONNECTIONBROKEN;
    }

    return msg;
  }

  vda5050_types::State create_state_msg()
  {
    vda5050_types::State msg;
    msg.header.header_id = 1;
    msg.header.timestamp = std::chrono::system_clock::now();
    msg.header.version = "2.0.0";
    msg.header.manufacturer = manufacturer_;
    msg.header.serial_number = serial_number_;
    msg.order_id = "test_order";
    msg.order_update_id = 0;
    msg.driving = false;
    msg.paused = false;
    msg.new_base_request = false;
    msg.distance_since_last_node = 0.0;
    return msg;
  }

  vda5050_types::Order create_test_order(const std::string& order_id)
  {
    vda5050_types::Order order;
    order.order_id = order_id;
    order.order_update_id = 0;
    return order;
  }

  vda5050_types::InstantActions create_test_instant_actions(uint32_t id)
  {
    vda5050_types::InstantActions actions;
    actions.header.header_id = id;
    return actions;
  }

  /**
   * @brief Validate order structure according to VDA5050 rules
   *
   * This is a copy of the structural validation logic from
   * VDA5050Master::validate_order() in master.cpp, without the AGV-specific
   * checks (onboarding, order update stitching). Keep in sync with master.
   *
   * Validates:
   * - Minimum 1 node required
   * - Node count must equal edge count + 1
   * - Node sequence_ids must be even and incremental
   * - Edge sequence_ids must be odd and incremental
   * - Combined sequence_ids must have no gaps
   * - Must have at least one released node
   * - Edges must reference existing nodes
   * - An edge can only be released if both start and end nodes are released
   * - After an unreleased edge, no released nodes or edges can follow
   *
   * @param order The order to validate
   * @param is_new_order True if this is a fresh order (sequence starts at 0)
   * @return True if order is valid
   */
  static bool validate_order_structure(
    const vda5050_types::Order& order, bool is_new_order = true)
  {
    // Minimum 1 node required
    if (order.nodes.empty())
    {
      return false;
    }

    // Node count must equal edge count + 1
    if (order.nodes.size() != order.edges.size() + 1)
    {
      return false;
    }

    // Node sequence_ids must be even and incremental
    if (is_new_order && order.nodes.front().sequence_id != 0)
    {
      return false;
    }
    for (size_t i = 0; i < order.nodes.size(); ++i)
    {
      if (order.nodes[i].sequence_id % 2 != 0)
      {
        return false;
      }
      if (
        i > 0 &&
        order.nodes[i].sequence_id != order.nodes[i - 1].sequence_id + 2)
      {
        return false;
      }
    }

    // Edge sequence_ids must be odd and incremental
    if (!order.edges.empty())
    {
      if (is_new_order && order.edges.front().sequence_id != 1)
      {
        return false;
      }
      for (size_t i = 0; i < order.edges.size(); ++i)
      {
        if (order.edges[i].sequence_id % 2 == 0)
        {
          return false;
        }
        if (
          i > 0 &&
          order.edges[i].sequence_id != order.edges[i - 1].sequence_id + 2)
        {
          return false;
        }
      }
    }

    // Combined sequence_ids must have no gaps
    std::vector<uint32_t> all_seqs;
    all_seqs.reserve(order.nodes.size() + order.edges.size());
    for (const auto& node : order.nodes)
    {
      all_seqs.push_back(node.sequence_id);
    }
    for (const auto& edge : order.edges)
    {
      all_seqs.push_back(edge.sequence_id);
    }
    std::sort(all_seqs.begin(), all_seqs.end());
    for (size_t i = 1; i < all_seqs.size(); ++i)
    {
      if (all_seqs[i] != all_seqs[i - 1] + 1)
      {
        return false;
      }
    }

    // Build node lookup for edge validation
    std::unordered_map<std::string, const vda5050_types::Node*> node_map;
    for (const auto& node : order.nodes)
    {
      node_map[node.node_id] = &node;
    }

    // Must have at least one released node
    bool has_released_node = false;
    for (const auto& node : order.nodes)
    {
      if (node.released)
      {
        has_released_node = true;
        break;
      }
    }
    if (!has_released_node)
    {
      return false;
    }

    for (const auto& edge : order.edges)
    {
      // Edges must reference existing nodes
      auto start_it = node_map.find(edge.start_node_id);
      auto end_it = node_map.find(edge.end_node_id);
      if (start_it == node_map.end() || end_it == node_map.end())
      {
        return false;
      }

      // An edge can only be released if both start and end nodes are released
      if (edge.released)
      {
        if (!start_it->second->released || !end_it->second->released)
        {
          return false;
        }
      }
    }

    // After an unreleased edge, no released nodes or edges can follow
    bool found_unreleased_edge = false;
    for (size_t i = 0; i < order.edges.size(); ++i)
    {
      if (!order.edges[i].released)
      {
        found_unreleased_edge = true;
      }
      else if (found_unreleased_edge)
      {
        return false;
      }

      // Check the node that follows this edge (node at index i+1)
      if (found_unreleased_edge && order.nodes[i + 1].released)
      {
        return false;
      }
    }

    return true;
  }

  /**
   * @brief Assert that an order is valid, failing the test if not
   */
  static void assert_valid_order(
    const vda5050_types::Order& order, bool is_new_order = true)
  {
    ASSERT_TRUE(validate_order_structure(order, is_new_order))
      << "Order '" << order.order_id << "' failed validation";
  }

  std::unique_ptr<AGV> agv_;
  std::string manufacturer_;
  std::string serial_number_;
  std::string agv_id_;
};

}  // namespace vda5050_master::test

#endif  // AGV__AGV_TEST_FIXTURE_HPP_
