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

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "agv_test_fixture.hpp"

namespace vda5050_master::test {

class OrderProgressTest : public AGVTestFixture
{
protected:
  // Helper: create a node with given id and sequence
  vda5050_types::Node make_node(
    const std::string& id, uint32_t seq, bool released = true)
  {
    vda5050_types::Node n;
    n.node_id = id;
    n.sequence_id = seq;
    n.released = released;
    return n;
  }

  // Helper: create an edge with given id and sequence
  vda5050_types::Edge make_edge(
    const std::string& id, uint32_t seq, const std::string& start,
    const std::string& end, bool released = true)
  {
    vda5050_types::Edge e;
    e.edge_id = id;
    e.sequence_id = seq;
    e.start_node_id = start;
    e.end_node_id = end;
    e.released = released;
    return e;
  }

  // Helper: create a node state
  vda5050_types::NodeState make_node_state(
    const std::string& id, uint32_t seq, bool released = true)
  {
    vda5050_types::NodeState ns;
    ns.node_id = id;
    ns.sequence_id = seq;
    ns.released = released;
    return ns;
  }

  // Helper: create an edge state
  vda5050_types::EdgeState make_edge_state(
    const std::string& id, uint32_t seq, bool released = true)
  {
    vda5050_types::EdgeState es;
    es.edge_id = id;
    es.sequence_id = seq;
    es.released = released;
    return es;
  }

  // Helper: create order with nodes N0->N2->N4 and edges E1->E3
  vda5050_types::Order make_simple_order(
    const std::string& order_id, uint32_t update_id = 0)
  {
    vda5050_types::Order order;
    order.order_id = order_id;
    order.order_update_id = update_id;
    order.nodes = {make_node("N0", 0), make_node("N2", 2), make_node("N4", 4)};
    order.edges = {
      make_edge("E1", 1, "N0", "N2"), make_edge("E3", 3, "N2", "N4")};
    return order;
  }

  // Helper: create state matching an order at a specific position
  vda5050_types::State make_state_at_node(
    const std::string& order_id, uint32_t update_id, const std::string& node_id,
    uint32_t node_seq,
    const std::vector<vda5050_types::NodeState>& remaining_nodes,
    const std::vector<vda5050_types::EdgeState>& remaining_edges,
    bool driving = false)
  {
    vda5050_types::State state = create_state_msg();
    state.order_id = order_id;
    state.order_update_id = update_id;
    state.last_node_id = node_id;
    state.last_node_sequence_id = node_seq;
    state.node_states = remaining_nodes;
    state.edge_states = remaining_edges;
    state.driving = driving;
    return state;
  }
};

// =============================================================================
// Initial Order Setup Tests
// =============================================================================

TEST_F(OrderProgressTest, SetCurrentOrderCreatesProgress)
{
  auto& agv = create_agv();
  auto order = make_simple_order("order1");
  assert_valid_order(order);

  agv->set_current_order(order);

  auto progress = agv->get_order_progress();
  ASSERT_TRUE(progress.has_value());
  EXPECT_EQ(progress->order_id, "order1");
  EXPECT_EQ(progress->order_update_id, 0u);
  EXPECT_FALSE(progress->confirmed);
  EXPECT_FALSE(progress->completed);
  EXPECT_EQ(progress->total_nodes, 3u);
  EXPECT_EQ(progress->total_edges, 2u);
  EXPECT_EQ(progress->completed_nodes, 0u);
  EXPECT_EQ(progress->completed_edges, 0u);
  EXPECT_EQ(progress->full_path_nodes.size(), 3u);
  EXPECT_EQ(progress->full_path_edges.size(), 2u);
}

TEST_F(OrderProgressTest, GetCurrentOrderReturnsExactOrder)
{
  auto& agv = create_agv();
  auto order = make_simple_order("order1");
  assert_valid_order(order);

  agv->set_current_order(order);

  auto stored = agv->get_current_order();
  ASSERT_TRUE(stored.has_value());
  EXPECT_EQ(stored->order_id, order.order_id);
  EXPECT_EQ(stored->nodes.size(), order.nodes.size());
  EXPECT_EQ(stored->edges.size(), order.edges.size());
}

TEST_F(OrderProgressTest, NoProgressInitially)
{
  auto& agv = create_agv();
  EXPECT_FALSE(agv->get_order_progress().has_value());
  EXPECT_FALSE(agv->get_current_order().has_value());
}

// =============================================================================
// Order Update (Stitching) Tests
// =============================================================================

TEST_F(OrderProgressTest, OrderUpdateAppendsToFullPath)
{
  auto& agv = create_agv();

  // Initial order: N0->E1->N2->E3->N4
  auto order1 = make_simple_order("order1", 0);
  assert_valid_order(order1);
  agv->set_current_order(order1);

  // Simulate AGV confirming and reaching N4 via state
  auto state = make_state_at_node("order1", 0, "N4", 4, {}, {});
  agv->handle_state(state);

  auto progress = agv->get_order_progress();
  ASSERT_TRUE(progress.has_value());
  EXPECT_EQ(progress->order_id, "order1");
  EXPECT_EQ(progress->order_update_id, 0u);
  EXPECT_TRUE(progress->confirmed);
  EXPECT_TRUE(progress->completed);
  EXPECT_EQ(progress->total_nodes, 3u);
  EXPECT_EQ(progress->total_edges, 2u);
  EXPECT_EQ(progress->completed_nodes, 3u);
  EXPECT_EQ(progress->completed_edges, 2u);
  EXPECT_EQ(progress->full_path_nodes.size(), 3u);
  EXPECT_EQ(progress->full_path_edges.size(), 2u);

  // Order update: stitches at N4, adds N6->E7->N8
  // First node (N4) is the stitching node
  vda5050_types::Order order2;
  order2.order_id = "order1";
  order2.order_update_id = 1;
  order2.nodes = {make_node("N4", 4), make_node("N6", 6), make_node("N8", 8)};
  order2.edges = {
    make_edge("E5", 5, "N4", "N6"), make_edge("E7", 7, "N6", "N8")};
  assert_valid_order(order2, false);  // order update, not new order

  agv->set_current_order(order2);

  progress = agv->get_order_progress();
  ASSERT_TRUE(progress.has_value());
  EXPECT_EQ(progress->order_update_id, 1u);
  EXPECT_FALSE(progress->confirmed);  // Reset on update
  EXPECT_FALSE(progress->completed);  // False on update
  // Full path totals: N0,N2,N4 + N6,N8 = 5 nodes, E1,E3 + E5,E7 = 4 edges
  EXPECT_EQ(progress->total_nodes, 5u);
  EXPECT_EQ(progress->total_edges, 4u);
  // Completed counts retained from state1 (original order was complete)
  EXPECT_EQ(progress->completed_nodes, 3u);
  EXPECT_EQ(progress->completed_edges, 2u);

  // AGV confirms update: at N4, only N6,N8 remaining
  auto state2 = make_state_at_node(
    "order1", 1, "N4", 4, {make_node_state("N6", 6), make_node_state("N8", 8)},
    {make_edge_state("E5", 5), make_edge_state("E7", 7)});
  agv->handle_state(state2);

  EXPECT_TRUE(agv->get_order_progress()->confirmed);
}

TEST_F(OrderProgressTest, CurrentOrderStoresLatestUpdateOnly)
{
  auto& agv = create_agv();

  auto order1 = make_simple_order("order1", 0);
  assert_valid_order(order1);
  agv->set_current_order(order1);

  // Order update
  vda5050_types::Order order2;
  order2.order_id = "order1";
  order2.order_update_id = 1;
  order2.nodes = {make_node("N4", 4), make_node("N6", 6)};
  order2.edges = {make_edge("E5", 5, "N4", "N6")};
  assert_valid_order(order2, false);  // order update

  agv->set_current_order(order2);

  // current_order_ should be the exact update message (2 nodes, 1 edge)
  auto stored = agv->get_current_order();
  ASSERT_TRUE(stored.has_value());
  EXPECT_EQ(stored->order_update_id, 1u);
  EXPECT_EQ(stored->nodes.size(), 2u);
  EXPECT_EQ(stored->edges.size(), 1u);

  auto progress = agv->get_order_progress();
  ASSERT_TRUE(progress.has_value());
  EXPECT_EQ(progress->order_update_id, 1u);
  EXPECT_FALSE(progress->confirmed);
  EXPECT_FALSE(progress->completed);
  // Full path totals: N0,N2,N4 + N6 = 4 nodes, E1,E3 + E5 = 3 edges
  EXPECT_EQ(progress->total_nodes, 4u);
  EXPECT_EQ(progress->total_edges, 3u);
  EXPECT_EQ(progress->completed_nodes, 0);
  EXPECT_EQ(progress->completed_edges, 0);
}

// =============================================================================
// Progress Update via State Tests
// =============================================================================

TEST_F(OrderProgressTest, StateUpdatesProgressAtStartNode)
{
  auto& agv = create_agv();
  auto order = make_simple_order("order1");
  assert_valid_order(order);
  agv->set_current_order(order);

  // AGV at N0, all nodes/edges remaining
  auto state = make_state_at_node(
    "order1", 0, "N0", 0,
    {make_node_state("N0", 0), make_node_state("N2", 2),
     make_node_state("N4", 4)},
    {make_edge_state("E1", 1), make_edge_state("E3", 3)});

  agv->handle_state(state);

  auto progress = agv->get_order_progress();
  ASSERT_TRUE(progress.has_value());
  EXPECT_EQ(progress->current_node_id, "N0");
  EXPECT_EQ(progress->current_node_sequence_id, 0u);
  EXPECT_EQ(progress->completed_nodes, 0u);
  EXPECT_EQ(progress->completed_edges, 0u);
  EXPECT_FALSE(progress->driving);
  EXPECT_FALSE(progress->completed);
}

TEST_F(OrderProgressTest, StateUpdatesProgressMidOrder)
{
  auto& agv = create_agv();
  auto order = make_simple_order("order1");
  assert_valid_order(order);
  agv->set_current_order(order);

  // Before state: no completed nodes/edges
  auto progress = agv->get_order_progress();
  ASSERT_TRUE(progress.has_value());
  EXPECT_EQ(progress->completed_nodes, 0u);
  EXPECT_EQ(progress->completed_edges, 0u);

  // AGV reached N2, completed N0 and E1
  auto state = make_state_at_node(
    "order1", 0, "N2", 2, {make_node_state("N2", 2), make_node_state("N4", 4)},
    {make_edge_state("E3", 3)});
  agv->handle_state(state);

  progress = agv->get_order_progress();
  EXPECT_EQ(progress->current_node_id, "N2");
  EXPECT_EQ(progress->completed_nodes, 1u);  // N0 completed
  EXPECT_EQ(progress->completed_edges, 1u);  // E1 completed
  EXPECT_FALSE(progress->completed);
}

TEST_F(OrderProgressTest, StateUpdatesProgressAtEndNode)
{
  auto& agv = create_agv();
  auto order = make_simple_order("order1");
  assert_valid_order(order);
  agv->set_current_order(order);

  // AGV at final node N4, only horizon nodes remaining (none released)
  auto state = make_state_at_node("order1", 0, "N4", 4, {}, {});

  agv->handle_state(state);

  auto progress = agv->get_order_progress();
  ASSERT_TRUE(progress.has_value());
  EXPECT_EQ(progress->current_node_id, "N4");
  EXPECT_EQ(progress->completed_nodes, 3u);
  EXPECT_EQ(progress->completed_edges, 2u);
  EXPECT_TRUE(progress->completed);
}

TEST_F(OrderProgressTest, DifferentOrderIdStateIgnored)
{
  auto& agv = create_agv();
  auto order = make_simple_order("order1");
  assert_valid_order(order);
  agv->set_current_order(order);

  // State from different order
  auto state = make_state_at_node("different_order", 0, "X0", 0, {}, {});
  agv->handle_state(state);

  // Progress should remain at initial values
  auto progress = agv->get_order_progress();
  ASSERT_TRUE(progress.has_value());
  EXPECT_FALSE(progress->confirmed);
  EXPECT_EQ(progress->completed_nodes, 0u);
}

// =============================================================================
// Order Confirmation Tests
// =============================================================================

TEST_F(OrderProgressTest, OrderConfirmedWhenStateMatchesUpdateId)
{
  auto& agv = create_agv();
  auto order = make_simple_order("order1", 0);
  assert_valid_order(order);
  agv->set_current_order(order);

  EXPECT_FALSE(agv->get_order_progress()->confirmed);

  // AGV confirms order (state.order_update_id >= order.order_update_id)
  auto state = make_state_at_node(
    "order1", 0, "N0", 0,
    {make_node_state("N0", 0), make_node_state("N2", 2),
     make_node_state("N4", 4)},
    {make_edge_state("E1", 1), make_edge_state("E3", 3)});
  agv->handle_state(state);

  EXPECT_TRUE(agv->get_order_progress()->confirmed);
}

TEST_F(OrderProgressTest, OrderNotConfirmedWhenStateLagsUpdateId)
{
  auto& agv = create_agv();

  // Initial order
  auto order1 = make_simple_order("order1", 0);
  assert_valid_order(order1);
  agv->set_current_order(order1);

  // AGV confirms initial order
  auto state1 = make_state_at_node("order1", 0, "N4", 4, {}, {});
  agv->handle_state(state1);
  EXPECT_TRUE(agv->get_order_progress()->confirmed);

  // Send order update
  vda5050_types::Order order2;
  order2.order_id = "order1";
  order2.order_update_id = 1;
  order2.nodes = {make_node("N4", 4), make_node("N6", 6)};
  order2.edges = {make_edge("E5", 5, "N4", "N6")};
  assert_valid_order(order2, false);  // order update
  agv->set_current_order(order2);

  // Not confirmed yet (reset on update)
  EXPECT_FALSE(agv->get_order_progress()->confirmed);

  // State still reports old update_id
  auto state2 = make_state_at_node("order1", 0, "N4", 4, {}, {});
  agv->handle_state(state2);

  // Still not confirmed
  EXPECT_FALSE(agv->get_order_progress()->confirmed);

  // AGV confirms new update
  auto state3 = make_state_at_node(
    "order1", 1, "N4", 4, {make_node_state("N4", 4), make_node_state("N6", 6)},
    {make_edge_state("E5", 5)});
  agv->handle_state(state3);

  EXPECT_TRUE(agv->get_order_progress()->confirmed);
}

// =============================================================================
// can_accept_new_order Tests
// =============================================================================

TEST_F(OrderProgressTest, CanAcceptOrderWhenNoCurrentOrder)
{
  auto& agv = create_agv();
  auto order = make_simple_order("order1");
  assert_valid_order(order);

  EXPECT_TRUE(agv->can_accept_new_order(order));
}

TEST_F(OrderProgressTest, CanAcceptOrderUpdateSameOrderId)
{
  auto& agv = create_agv();
  auto order1 = make_simple_order("order1", 0);
  assert_valid_order(order1);
  agv->set_current_order(order1);

  // Order update with same order_id (minimal order update for validation check)
  vda5050_types::Order order2;
  order2.order_id = "order1";
  order2.order_update_id = 1;
  order2.nodes = {make_node("N4", 4), make_node("N6", 6)};
  order2.edges = {make_edge("E5", 5, "N4", "N6")};
  assert_valid_order(order2, false);  // order update

  EXPECT_TRUE(agv->can_accept_new_order(order2));
}

TEST_F(OrderProgressTest, CannotAcceptNewOrderWhenBusy)
{
  auto& agv = create_agv();
  auto order1 = make_simple_order("order1");
  assert_valid_order(order1);
  agv->set_current_order(order1);

  // AGV mid-order (base nodes remaining)
  auto state = make_state_at_node(
    "order1", 0, "N2", 2, {make_node_state("N2", 2), make_node_state("N4", 4)},
    {make_edge_state("E3", 3)});
  agv->handle_state(state);

  // Different order should be rejected
  auto order2 = make_simple_order("order2");
  assert_valid_order(order2);
  EXPECT_FALSE(agv->can_accept_new_order(order2));
}

TEST_F(OrderProgressTest, CannotAcceptNewOrderNoStateYet)
{
  auto& agv = create_agv();
  auto order1 = make_simple_order("order1");
  assert_valid_order(order1);
  agv->set_current_order(order1);

  // No state received yet
  auto order2 = make_simple_order("order2");
  assert_valid_order(order2);
  EXPECT_FALSE(agv->can_accept_new_order(order2));
}

TEST_F(OrderProgressTest, CanAcceptNewOrderWhenComplete)
{
  auto& agv = create_agv();
  auto order1 = make_simple_order("order1");
  assert_valid_order(order1);
  agv->set_current_order(order1);

  // Order complete: at last node, no base nodes remaining, no edges
  auto state = make_state_at_node("order1", 0, "N4", 4, {}, {});
  agv->handle_state(state);

  // Different order should be accepted
  auto order2 = make_simple_order("order2");
  assert_valid_order(order2);
  EXPECT_TRUE(agv->can_accept_new_order(order2));
}

TEST_F(OrderProgressTest, CannotAcceptNewOrderWithBaseNodesRemaining)
{
  auto& agv = create_agv();
  auto order1 = make_simple_order("order1");
  assert_valid_order(order1);
  agv->set_current_order(order1);

  // Base node and edge still remaining (consistent state)
  auto state = make_state_at_node(
    "order1", 0, "N2", 2, {make_node_state("N4", 4, true)},  // released = base
    {make_edge_state("E3", 3, true)});  // edge to N4 also remaining
  agv->handle_state(state);

  auto order2 = make_simple_order("order2");
  assert_valid_order(order2);
  EXPECT_FALSE(agv->can_accept_new_order(order2));
}

// =============================================================================
// Edge Traversal Tests
// =============================================================================

TEST_F(OrderProgressTest, CurrentEdgeSetWhenDriving)
{
  auto& agv = create_agv();
  auto order = make_simple_order("order1");
  assert_valid_order(order);
  agv->set_current_order(order);

  // AGV driving on edge E1
  auto state = make_state_at_node(
    "order1", 0, "N0", 0,
    {make_node_state("N0", 0), make_node_state("N2", 2),
     make_node_state("N4", 4)},
    {make_edge_state("E1", 1), make_edge_state("E3", 3)},
    true);  // driving=true
  agv->handle_state(state);

  auto progress = agv->get_order_progress();
  ASSERT_TRUE(progress.has_value());
  EXPECT_TRUE(progress->driving);
  EXPECT_TRUE(progress->current_edge_id.has_value());
  EXPECT_EQ(progress->current_edge_id.value(), "E1");
  EXPECT_EQ(progress->current_edge_sequence_id.value(), 1u);
}

TEST_F(OrderProgressTest, CurrentEdgeClearedWhenNotDriving)
{
  auto& agv = create_agv();
  auto order = make_simple_order("order1");
  assert_valid_order(order);
  agv->set_current_order(order);

  // First: driving
  auto state1 = make_state_at_node(
    "order1", 0, "N0", 0,
    {make_node_state("N0", 0), make_node_state("N2", 2),
     make_node_state("N4", 4)},
    {make_edge_state("E1", 1), make_edge_state("E3", 3)}, true);
  agv->handle_state(state1);

  EXPECT_TRUE(agv->get_order_progress()->current_edge_id.has_value());

  // Then: stopped at node
  auto state2 = make_state_at_node(
    "order1", 0, "N2", 2, {make_node_state("N2", 2), make_node_state("N4", 4)},
    {make_edge_state("E3", 3)}, false);  // driving=false
  agv->handle_state(state2);

  auto progress = agv->get_order_progress();
  EXPECT_FALSE(progress->driving);
  EXPECT_FALSE(progress->current_edge_id.has_value());
}

// =============================================================================
// Fresh Order Replaces Previous Tests
// =============================================================================

TEST_F(OrderProgressTest, FreshOrderReplacesExistingProgress)
{
  auto& agv = create_agv();

  // First order: N0->E1->N2->E3->N4
  auto order1 = make_simple_order("order1");
  assert_valid_order(order1);
  agv->set_current_order(order1);

  // AGV confirms order at start position (N0 with all nodes/edges remaining)
  auto state1 = make_state_at_node(
    "order1", 0, "N0", 0, {make_node_state("N2", 2), make_node_state("N4", 4)},
    {make_edge_state("E1", 1), make_edge_state("E3", 3)});
  agv->handle_state(state1);

  // AGV completes order (at N4 with no remaining work)
  auto state2 = make_state_at_node("order1", 0, "N4", 4, {}, {});
  agv->handle_state(state2);

  // New order with different ID (order1 complete)
  vda5050_types::Order order2;
  order2.order_id = "order2";
  order2.order_update_id = 0;
  order2.nodes = {make_node("A0", 0), make_node("A2", 2)};
  order2.edges = {make_edge("B1", 1, "A0", "A2")};
  assert_valid_order(order2);

  agv->set_current_order(order2);

  auto progress = agv->get_order_progress();
  ASSERT_TRUE(progress.has_value());
  EXPECT_EQ(progress->order_id, "order2");
  EXPECT_EQ(progress->full_path_nodes.size(), 2u);
  EXPECT_EQ(progress->full_path_edges.size(), 1u);
  EXPECT_FALSE(progress->confirmed);
}

}  // namespace vda5050_master::test
