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

#include <gmock/gmock.h>

#include <chrono>
#include <thread>

#include "vda5050_core/logger/logger.hpp"
#include "vda5050_master/standard_names.hpp"

using vda5050_master::communication::HeartbeatState;

/**
 * @brief Mock HeartbeatListener for testing
 *
 * This mock overrides get_check_interval() to use a 1 second interval
 * instead of the default 15 seconds, making tests run faster.
 */
class MockHeartbeatListener
: public vda5050_master::communication::HeartbeatListener
{
public:
  MockHeartbeatListener(
    const std::string& id, const int heartbeat_interval,
    std::function<void()> disconnection_callback)
  : HeartbeatListener(id, heartbeat_interval, disconnection_callback)
  {
  }

  // Override check interval to speed up tests
  // Instead of waiting 15 seconds, wait only 1 second
  int get_check_interval() override
  {
    return 1;  // Check every 1 second in tests
  }

  ~MockHeartbeatListener()
  {
    VDA5050_INFO("MockHeartbeatListener destroyed");
  }
};

TEST(HeartbeatListenerTest, HeartbeatListenerInit)
{
  auto hb_listener = MockHeartbeatListener(
    "test_listener", vda5050_master::ConnectionHeartbeatInterval, [&]() {
      // Timeout callback
      VDA5050_INFO("Timeout callback");
    });

  ASSERT_EQ(hb_listener.get_state(), HeartbeatState::STOPPED);
  hb_listener.start_connection_heartbeat();
  ASSERT_EQ(hb_listener.get_state(), HeartbeatState::RUNNING);
  ASSERT_NO_THROW(hb_listener.stop_connection_heartbeat());
  ASSERT_EQ(hb_listener.get_state(), HeartbeatState::STOPPED);
}

TEST(HeartbeatListenerTest, HeartbeatReceivedNoTimeout)
{
  std::atomic<bool> timeout_called{false};
  auto hb_listener = MockHeartbeatListener(
    "test_listener", vda5050_master::ConnectionHeartbeatInterval,
    [&timeout_called]() { timeout_called.store(true); });

  hb_listener.start_connection_heartbeat();
  ASSERT_EQ(hb_listener.get_state(), HeartbeatState::RUNNING);

  // Send heartbeat before timeout (within 1 second)
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  hb_listener.received_connection();

  // Verify last_connection_report was updated to approximately now
  auto now = std::chrono::steady_clock::now();
  auto last_report = hb_listener.get_last_connection_report();
  auto diff =
    std::chrono::duration_cast<std::chrono::milliseconds>(now - last_report)
      .count();
  ASSERT_LT(std::abs(diff), 100)
    << "last_connection_report should be close to now";

  // Stop before timeout triggers
  ASSERT_NO_THROW(hb_listener.stop_connection_heartbeat());
  ASSERT_FALSE(timeout_called.load()) << "Timeout should not have been called";
}

TEST(HeartbeatListenerTest, HeartbeatNotReceivedTimeout)
{
  std::atomic<bool> heartbeat_failed{false};
  auto hb_listener = MockHeartbeatListener(
    "test_listener", vda5050_master::ConnectionHeartbeatInterval,
    [&heartbeat_failed]() {
      VDA5050_INFO("Timeout callback");
      heartbeat_failed.store(true);
    });

  hb_listener.start_connection_heartbeat();

  // Wait for timeout (get_check_interval() returns 1 second in mock)
  // Timeout triggers at >= 1 second elapsed
  std::this_thread::sleep_for(std::chrono::milliseconds(1500));

  ASSERT_TRUE(heartbeat_failed.load()) << "Timeout callback should have fired";
  ASSERT_EQ(hb_listener.get_state(), HeartbeatState::STOPPED)
    << "State should be STOPPED after timeout";
}

TEST(HeartbeatListenerTest, HeartbeatReceivedResetsTimeout)
{
  std::atomic<bool> heartbeat_failed{false};
  auto hb_listener = MockHeartbeatListener(
    "test_listener", vda5050_master::ConnectionHeartbeatInterval,
    [&heartbeat_failed]() {
      VDA5050_INFO("Timeout callback");
      heartbeat_failed.store(true);
    });

  hb_listener.start_connection_heartbeat();

  // Wait 500ms (less than 1 second timeout interval)
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  ASSERT_FALSE(heartbeat_failed.load()) << "Should not timeout yet";

  // Send heartbeat - this should reset the timeout
  hb_listener.received_connection();

  // Wait 800ms - only 0.8s since heartbeat, should not timeout
  std::this_thread::sleep_for(std::chrono::milliseconds(800));
  ASSERT_FALSE(heartbeat_failed.load())
    << "Should not timeout - heartbeat reset the timer";

  // Wait another 500ms (total 1.3s since heartbeat) - should timeout now
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  ASSERT_TRUE(heartbeat_failed.load()) << "Should timeout now";
  ASSERT_EQ(hb_listener.get_state(), HeartbeatState::STOPPED);
}

TEST(HeartbeatListenerTest, GracefulShutdownDoesNotBlock)
{
  std::atomic<bool> callback_called{false};

  auto hb_listener = std::make_unique<MockHeartbeatListener>(
    "test_listener", vda5050_master::ConnectionHeartbeatInterval,
    [&callback_called]() { callback_called.store(true); });

  hb_listener->start_connection_heartbeat();
  ASSERT_EQ(hb_listener->get_state(), HeartbeatState::RUNNING);

  // This should complete quickly, not block forever
  auto start = std::chrono::steady_clock::now();
  hb_listener->stop_connection_heartbeat();
  auto elapsed = std::chrono::steady_clock::now() - start;

  // Should complete within 2 seconds (1 second check interval + margin)
  ASSERT_LT(
    std::chrono::duration_cast<std::chrono::seconds>(elapsed).count(), 2)
    << "stop_connection_heartbeat() blocked too long";

  ASSERT_EQ(hb_listener->get_state(), HeartbeatState::STOPPED);
  ASSERT_FALSE(callback_called.load())
    << "Callback should NOT be called during graceful shutdown";
}

TEST(HeartbeatListenerTest, StateIsStoppingWhileCallbackExecutes)
{
  std::atomic<bool> callback_started{false};
  std::atomic<bool> callback_finished{false};
  std::atomic<bool> was_stopping_during_callback{false};

  // We need a raw pointer to check get_state() from within callback
  MockHeartbeatListener* listener_ptr = nullptr;

  auto hb_listener = std::make_unique<MockHeartbeatListener>(
    "test_listener", vda5050_master::ConnectionHeartbeatInterval,
    [&callback_started, &callback_finished, &was_stopping_during_callback,
     &listener_ptr]() {
      callback_started.store(true);

      // Check get_state() during callback execution - should be STOPPING
      if (listener_ptr)
      {
        was_stopping_during_callback.store(
          listener_ptr->get_state() == HeartbeatState::STOPPING);
      }

      // Simulate work
      std::this_thread::sleep_for(std::chrono::milliseconds(100));

      callback_finished.store(true);
    });

  listener_ptr = hb_listener.get();
  hb_listener->start_connection_heartbeat();

  // Wait for timeout (get_check_interval() returns 1 second in mock)
  while (!callback_finished.load())
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  // Now stop - should be quick since callback already finished
  hb_listener->stop_connection_heartbeat();

  ASSERT_TRUE(callback_started.load()) << "Callback should have started";
  ASSERT_TRUE(callback_finished.load()) << "Callback should have finished";
  ASSERT_TRUE(was_stopping_during_callback.load())
    << "State should be STOPPING while callback executes";
  ASSERT_EQ(hb_listener->get_state(), HeartbeatState::STOPPED)
    << "State should be STOPPED after callback completes";
}

TEST(HeartbeatListenerTest, MultipleStopCallsSafe)
{
  auto hb_listener = std::make_unique<MockHeartbeatListener>(
    "test_listener", vda5050_master::ConnectionHeartbeatInterval,
    []() { /* No-op */ });

  hb_listener->start_connection_heartbeat();
  ASSERT_EQ(hb_listener->get_state(), HeartbeatState::RUNNING);

  ASSERT_NO_THROW(hb_listener->stop_connection_heartbeat());
  ASSERT_EQ(hb_listener->get_state(), HeartbeatState::STOPPED);

  ASSERT_NO_THROW(hb_listener->stop_connection_heartbeat());
  ASSERT_EQ(hb_listener->get_state(), HeartbeatState::STOPPED);

  ASSERT_NO_THROW(hb_listener.reset());
}

TEST(HeartbeatListenerTest, DoubleStartPrevented)
{
  auto hb_listener = MockHeartbeatListener(
    "test_listener", vda5050_master::ConnectionHeartbeatInterval,
    []() { /* No-op */ });

  hb_listener.start_connection_heartbeat();
  ASSERT_EQ(hb_listener.get_state(), HeartbeatState::RUNNING);

  // Second start should be ignored (no-op, not throw)
  ASSERT_NO_THROW(hb_listener.start_connection_heartbeat());
  ASSERT_EQ(hb_listener.get_state(), HeartbeatState::RUNNING);

  hb_listener.stop_connection_heartbeat();
  ASSERT_EQ(hb_listener.get_state(), HeartbeatState::STOPPED);
}

TEST(HeartbeatListenerTest, StopWhenNeverStarted)
{
  auto hb_listener = MockHeartbeatListener(
    "test_listener", vda5050_master::ConnectionHeartbeatInterval,
    []() { /* No-op */ });

  ASSERT_EQ(hb_listener.get_state(), HeartbeatState::STOPPED);

  // Stop when never started should be safe
  ASSERT_NO_THROW(hb_listener.stop_connection_heartbeat());
  ASSERT_EQ(hb_listener.get_state(), HeartbeatState::STOPPED);
}

TEST(HeartbeatListenerTest, ReceivedConnectionIgnoredWhenNotRunning)
{
  auto hb_listener = MockHeartbeatListener(
    "test_listener", vda5050_master::ConnectionHeartbeatInterval,
    []() { /* No-op */ });

  // Get initial last_connection_report
  auto initial_report = hb_listener.get_last_connection_report();

  // Small delay to ensure time difference would be detectable
  std::this_thread::sleep_for(std::chrono::milliseconds(10));

  // Call received_connection when STOPPED - should be ignored
  ASSERT_NO_THROW(hb_listener.received_connection());

  // last_connection_report should NOT be updated
  auto after_report = hb_listener.get_last_connection_report();
  ASSERT_EQ(initial_report, after_report)
    << "received_connection() should be ignored when not RUNNING";
}

TEST(HeartbeatListenerTest, InvalidHeartbeatIntervalThrows)
{
  // Zero interval should throw
  ASSERT_THROW(
    MockHeartbeatListener("test", 0, []() {}), std::invalid_argument);

  // Negative interval should throw
  ASSERT_THROW(
    MockHeartbeatListener("test", -1, []() {}), std::invalid_argument);
}

TEST(HeartbeatListenerTest, NullCallbackThrows)
{
  ASSERT_THROW(
    MockHeartbeatListener("test", 1, nullptr), std::invalid_argument);
}

TEST(HeartbeatListenerTest, ReceivedConnectionDuringStoppingSafe)
{
  std::atomic<bool> stop_completed{false};
  std::atomic<bool> sent_during_stopping{false};

  auto hb_listener = std::make_unique<MockHeartbeatListener>(
    "test_listener", vda5050_master::ConnectionHeartbeatInterval,
    []() { /* No-op */ });

  hb_listener->start_connection_heartbeat();
  ASSERT_EQ(hb_listener->get_state(), HeartbeatState::RUNNING);

  // Stop in a separate thread
  std::thread stop_thread([&]() {
    hb_listener->stop_connection_heartbeat();
    stop_completed.store(true);
  });

  // Repeatedly call received_connection during shutdown
  while (!stop_completed.load())
  {
    if (hb_listener->get_state() == HeartbeatState::STOPPING)
    {
      ASSERT_NO_THROW(hb_listener->received_connection());
      sent_during_stopping.store(true);
    }
    std::this_thread::sleep_for(std::chrono::microseconds(50));
  }

  stop_thread.join();

  ASSERT_TRUE(sent_during_stopping.load())
    << "Should have sent heartbeat while in STOPPING state";
  ASSERT_EQ(hb_listener->get_state(), HeartbeatState::STOPPED);
}

TEST(HeartbeatListenerTest, TimeoutGoesThoughStoppingState)
{
  std::atomic<bool> observed_stopping{false};
  std::atomic<bool> callback_finished{false};

  MockHeartbeatListener* listener_ptr = nullptr;

  auto hb_listener = std::make_unique<MockHeartbeatListener>(
    "test_listener", vda5050_master::ConnectionHeartbeatInterval,
    [&observed_stopping, &callback_finished, &listener_ptr]() {
      // Check state during callback - should be STOPPING
      if (listener_ptr && listener_ptr->get_state() == HeartbeatState::STOPPING)
      {
        observed_stopping.store(true);
      }
      callback_finished.store(true);
    });

  listener_ptr = hb_listener.get();
  hb_listener->start_connection_heartbeat();

  // Wait for timeout callback to complete
  while (!callback_finished.load())
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  ASSERT_TRUE(observed_stopping.load())
    << "State should be STOPPING during timeout callback";
  ASSERT_EQ(hb_listener->get_state(), HeartbeatState::STOPPED)
    << "State should be STOPPED after timeout callback completes";
}

TEST(HeartbeatListenerTest, StopAfterTimeoutIsSafe)
{
  std::atomic<bool> callback_called{false};

  auto hb_listener = std::make_unique<MockHeartbeatListener>(
    "test_listener", vda5050_master::ConnectionHeartbeatInterval,
    [&callback_called]() { callback_called.store(true); });

  hb_listener->start_connection_heartbeat();

  // Wait for timeout
  while (!callback_called.load())
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  // Wait a bit for state to transition to STOPPED
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  ASSERT_EQ(hb_listener->get_state(), HeartbeatState::STOPPED);

  // Calling stop after timeout should be safe (no double-join issues)
  ASSERT_NO_THROW(hb_listener->stop_connection_heartbeat());
  ASSERT_EQ(hb_listener->get_state(), HeartbeatState::STOPPED);
}

TEST(HeartbeatListenerTest, StopDuringTimeoutCallbackIsSafe)
{
  std::atomic<bool> callback_started{false};
  std::atomic<bool> callback_finished{false};
  std::atomic<bool> stop_completed{false};

  auto hb_listener = std::make_unique<MockHeartbeatListener>(
    "test_listener", vda5050_master::ConnectionHeartbeatInterval,
    [&callback_started, &callback_finished]() {
      callback_started.store(true);
      // Simulate slow callback
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
      callback_finished.store(true);
    });

  hb_listener->start_connection_heartbeat();

  // Wait for callback to start
  while (!callback_started.load())
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  // Try to stop while callback is executing
  std::thread stop_thread([&]() {
    hb_listener->stop_connection_heartbeat();
    stop_completed.store(true);
  });

  stop_thread.join();

  ASSERT_TRUE(callback_finished.load()) << "Callback should have completed";
  ASSERT_TRUE(stop_completed.load()) << "Stop should have completed";
  ASSERT_EQ(hb_listener->get_state(), HeartbeatState::STOPPED);
}
