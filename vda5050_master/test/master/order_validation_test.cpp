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

#include <gtest/gtest.h>

#include <memory>
#include <string>

#include "vda5050_core/mqtt_client/mqtt_client_interface.hpp"
#include "vda5050_master/vda5050_master/master.hpp"

namespace vda5050_master::test {

// Minimal stub MQTT client - VDA5050Master constructor requires one,
// but order validation doesn't use MQTT functionality
class StubMqttClient : public vda5050_core::mqtt_client::MqttClientInterface
{
public:
  void connect() override {}
  void disconnect() override {}
  bool connected() override
  {
    return false;
  }
  void publish(const std::string&, const std::string&, int, bool) override {}
  void subscribe(const std::string&, MessageHandler, int) override {}
  void unsubscribe(const std::string&) override {}
  void set_will(const std::string&, const std::string&, int) override {}
};

// =============================================================================
// Test Fixture
// =============================================================================

class OrderValidationTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    auto client = std::make_shared<StubMqttClient>();
    master_ = std::make_unique<VDA5050Master>(client, "tcp://localhost:1883");
  }

  void onboard_test_agv()
  {
    master_->onboard_agv(manufacturer_, serial_number_);
  }

  // Create a valid minimal order (1 released node, 0 edges)
  vda5050_types::Order create_valid_order()
  {
    vda5050_types::Order order;
    order.header.manufacturer = manufacturer_;
    order.header.serial_number = serial_number_;
    order.order_id = "order_001";
    order.order_update_id = 0;

    vda5050_types::Node node;
    node.node_id = "node_0";
    node.sequence_id = 0;
    node.released = true;
    order.nodes.push_back(node);

    return order;
  }

  // Create a valid order with N nodes and N-1 edges
  vda5050_types::Order create_order_with_nodes(
    size_t num_nodes, size_t num_released_nodes)
  {
    vda5050_types::Order order;
    order.header.manufacturer = manufacturer_;
    order.header.serial_number = serial_number_;
    order.order_id = "order_001";
    order.order_update_id = 0;

    for (size_t i = 0; i < num_nodes; ++i)
    {
      vda5050_types::Node node;
      node.node_id = "node_" + std::to_string(i);
      node.sequence_id = static_cast<uint32_t>(i * 2);
      node.released = (i < num_released_nodes);
      order.nodes.push_back(node);
    }

    for (size_t i = 0; i < num_nodes - 1; ++i)
    {
      vda5050_types::Edge edge;
      edge.edge_id = "edge_" + std::to_string(i);
      edge.sequence_id = static_cast<uint32_t>(i * 2 + 1);
      edge.start_node_id = "node_" + std::to_string(i);
      edge.end_node_id = "node_" + std::to_string(i + 1);
      edge.released = (i < num_released_nodes - 1);
      order.edges.push_back(edge);
    }

    return order;
  }

  std::unique_ptr<VDA5050Master> master_;
  std::string manufacturer_ = "TestManufacturer";
  std::string serial_number_ = "SN001";
};

// =============================================================================
// AGV must be onboarded
// =============================================================================

TEST_F(OrderValidationTest, RejectsOrderForNonOnboardedAGV)
{
  // No AGV onboarded
  auto order = create_valid_order();

  EXPECT_FALSE(master_->validate_order(order));
}

TEST_F(OrderValidationTest, AcceptsOrderForOnboardedAGV)
{
  onboard_test_agv();
  auto order = create_valid_order();

  EXPECT_TRUE(master_->validate_order(order));
}

// =============================================================================
// Minimum 1 node required
// =============================================================================

TEST_F(OrderValidationTest, RejectsOrderWithNoNodes)
{
  onboard_test_agv();
  auto order = create_valid_order();
  order.nodes.clear();

  EXPECT_FALSE(master_->validate_order(order));
}

TEST_F(OrderValidationTest, AcceptsOrderWithSingleNode)
{
  onboard_test_agv();
  auto order = create_valid_order();

  EXPECT_TRUE(master_->validate_order(order));
}

// =============================================================================
// Node count must equal edge count + 1
// =============================================================================

TEST_F(OrderValidationTest, RejectsOrderWithTooManyEdges)
{
  onboard_test_agv();
  auto order = create_order_with_nodes(3, 3);

  vda5050_types::Edge extra_edge;
  extra_edge.edge_id = "extra_edge";
  extra_edge.sequence_id = 5;
  extra_edge.start_node_id = "node_0";
  extra_edge.end_node_id = "node_1";
  extra_edge.released = true;
  order.edges.push_back(extra_edge);

  EXPECT_FALSE(master_->validate_order(order));
}

TEST_F(OrderValidationTest, RejectsOrderWithTooFewEdges)
{
  onboard_test_agv();
  auto order = create_order_with_nodes(3, 3);
  order.edges.pop_back();

  EXPECT_FALSE(master_->validate_order(order));
}

TEST_F(OrderValidationTest, AcceptsOrderWithCorrectNodeEdgeRatio)
{
  onboard_test_agv();
  auto order = create_order_with_nodes(3, 3);

  EXPECT_TRUE(master_->validate_order(order));
}

// =============================================================================
// Node sequence_ids must be even and incremental
// =============================================================================

TEST_F(OrderValidationTest, RejectsOrderWithOddNodeSequenceId)
{
  onboard_test_agv();
  auto order = create_valid_order();
  order.nodes[0].sequence_id = 1;

  EXPECT_FALSE(master_->validate_order(order));
}

TEST_F(OrderValidationTest, RejectsNewOrderWithNodeSequenceNotStartingAtZero)
{
  onboard_test_agv();
  auto order = create_valid_order();
  order.nodes[0].sequence_id = 2;

  EXPECT_FALSE(master_->validate_order(order));
}

TEST_F(OrderValidationTest, RejectsOrderWithNonIncrementalNodeSequence)
{
  onboard_test_agv();
  auto order = create_order_with_nodes(3, 3);
  order.nodes[2].sequence_id = 6;  // Should be 4

  EXPECT_FALSE(master_->validate_order(order));
}

// =============================================================================
// Edge sequence_ids must be odd and incremental
// =============================================================================

TEST_F(OrderValidationTest, RejectsOrderWithEvenEdgeSequenceId)
{
  onboard_test_agv();
  auto order = create_order_with_nodes(2, 2);
  order.edges[0].sequence_id = 2;

  EXPECT_FALSE(master_->validate_order(order));
}

TEST_F(OrderValidationTest, RejectsNewOrderWithEdgeSequenceNotStartingAtOne)
{
  onboard_test_agv();
  auto order = create_order_with_nodes(2, 2);
  order.edges[0].sequence_id = 3;

  EXPECT_FALSE(master_->validate_order(order));
}

TEST_F(OrderValidationTest, RejectsOrderWithNonIncrementalEdgeSequence)
{
  onboard_test_agv();
  auto order = create_order_with_nodes(3, 3);
  order.edges[1].sequence_id = 5;  // Should be 3

  EXPECT_FALSE(master_->validate_order(order));
}

// =============================================================================
// Combined sequence_ids must have no gaps
// =============================================================================

TEST_F(OrderValidationTest, RejectsOrderWithGapInSequenceIds)
{
  onboard_test_agv();
  auto order = create_order_with_nodes(3, 3);
  // Create gap: 0, 1, 4, 5 instead of 0, 1, 2, 3, 4
  order.nodes[1].sequence_id = 4;
  order.edges[1].sequence_id = 5;

  EXPECT_FALSE(master_->validate_order(order));
}

// =============================================================================
// Must have at least one released node
// =============================================================================

TEST_F(OrderValidationTest, RejectsOrderWithNoReleasedNodes)
{
  onboard_test_agv();
  auto order = create_valid_order();
  order.nodes[0].released = false;

  EXPECT_FALSE(master_->validate_order(order));
}

TEST_F(OrderValidationTest, AcceptsOrderWithAtLeastOneReleasedNode)
{
  onboard_test_agv();
  auto order = create_order_with_nodes(3, 1);  // Only first node released

  EXPECT_TRUE(master_->validate_order(order));
}

// =============================================================================
// Edges must reference existing nodes
// =============================================================================

TEST_F(OrderValidationTest, RejectsEdgeWithNonExistentStartNode)
{
  onboard_test_agv();
  auto order = create_order_with_nodes(2, 2);
  order.edges[0].start_node_id = "non_existent_node";

  EXPECT_FALSE(master_->validate_order(order));
}

TEST_F(OrderValidationTest, RejectsEdgeWithNonExistentEndNode)
{
  onboard_test_agv();
  auto order = create_order_with_nodes(2, 2);
  order.edges[0].end_node_id = "non_existent_node";

  EXPECT_FALSE(master_->validate_order(order));
}

// =============================================================================
// An edge can only be released if both start and end nodes are released
// =============================================================================

TEST_F(OrderValidationTest, RejectsReleasedEdgeWithUnreleasedStartNode)
{
  onboard_test_agv();
  auto order = create_order_with_nodes(2, 2);
  order.nodes[0].released = false;
  order.edges[0].released = true;

  EXPECT_FALSE(master_->validate_order(order));
}

TEST_F(OrderValidationTest, RejectsReleasedEdgeWithUnreleasedEndNode)
{
  onboard_test_agv();
  auto order = create_order_with_nodes(2, 2);
  order.nodes[1].released = false;
  order.edges[0].released = true;

  EXPECT_FALSE(master_->validate_order(order));
}

TEST_F(OrderValidationTest, AcceptsReleasedEdgeWithBothNodesReleased)
{
  onboard_test_agv();
  auto order = create_order_with_nodes(2, 2);

  EXPECT_TRUE(master_->validate_order(order));
}

TEST_F(OrderValidationTest, RejectsUnreleasedEdgeFollowedByReleasedNode)
{
  onboard_test_agv();
  auto order = create_order_with_nodes(2, 2);
  order.edges[0].released = false;
  // node_0(released) -> edge_0(unreleased) -> node_1(released)
  // This violates: after an unreleased edge, no released nodes can follow

  EXPECT_FALSE(master_->validate_order(order));
}

TEST_F(OrderValidationTest, AcceptsUnreleasedEdgeFollowedByUnreleasedNode)
{
  onboard_test_agv();
  auto order = create_order_with_nodes(2, 2);
  order.edges[0].released = false;
  order.nodes[1].released = false;
  // node_0(released) -> edge_0(unreleased) -> node_1(unreleased)
  // This is valid: unreleased edge followed by unreleased node

  EXPECT_TRUE(master_->validate_order(order));
}

// =============================================================================
// After an unreleased edge, no released nodes or edges can follow in sequence
// =============================================================================

TEST_F(OrderValidationTest, RejectsReleasedEdgeAfterUnreleasedEdge)
{
  onboard_test_agv();
  auto order = create_order_with_nodes(3, 3);
  order.edges[0].released = false;
  order.edges[1].released = true;  // Released edge after unreleased

  EXPECT_FALSE(master_->validate_order(order));
}

TEST_F(OrderValidationTest, RejectsReleasedNodeAfterUnreleasedEdge)
{
  onboard_test_agv();
  auto order = create_order_with_nodes(3, 3);
  order.edges[0].released = false;
  order.nodes[1].released = true;  // Released node after unreleased edge

  EXPECT_FALSE(master_->validate_order(order));
}

TEST_F(OrderValidationTest, AcceptsAllUnreleasedAfterFirstUnreleasedEdge)
{
  onboard_test_agv();
  auto order = create_order_with_nodes(3, 1);
  // node_0 released, edge_0 unreleased, node_1 unreleased,
  // edge_1 unreleased, node_2 unreleased

  EXPECT_TRUE(master_->validate_order(order));
}

// =============================================================================
// Valid complex orders
// =============================================================================

TEST_F(OrderValidationTest, AcceptsValidOrderWithAllReleased)
{
  onboard_test_agv();
  auto order = create_order_with_nodes(5, 5);

  EXPECT_TRUE(master_->validate_order(order));
}

TEST_F(OrderValidationTest, AcceptsValidOrderWithReleasedThenUnreleased)
{
  onboard_test_agv();
  auto order = create_order_with_nodes(5, 3);
  // 3 released nodes, 2 released edges, then unreleased

  EXPECT_TRUE(master_->validate_order(order));
}

}  // namespace vda5050_master::test
