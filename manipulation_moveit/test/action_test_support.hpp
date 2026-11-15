// Responsibility: Provides deterministic ROS executor and signal fixtures for action-port tests.

#ifndef ACTION_TEST_SUPPORT_HPP_
#define ACTION_TEST_SUPPORT_HPP_

#include <chrono>
#include <condition_variable>
#include <initializer_list>
#include <mutex>
#include <thread>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"

namespace manipulation::moveit::test
{

class Signal final
{
public:
  void notify()
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    ready_ = true;
    cv_.notify_all();
  }

  [[nodiscard]] bool wait_for(const std::chrono::steady_clock::duration timeout)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return cv_.wait_for(lock, timeout, [this]() {return ready_;});
  }

  void wait()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    cv_.wait(lock, [this]() {return ready_;});
  }

private:
  std::mutex mutex_;
  std::condition_variable cv_;
  bool ready_{false};
};

class ExecutorThread final
{
public:
  ExecutorThread(std::initializer_list<rclcpp::Node::SharedPtr> nodes)
  {
    for (const auto & node : nodes) {
      executor_.add_node(node);
    }
    thread_ = std::thread([this]() {executor_.spin();});
  }

  ~ExecutorThread()
  {
    executor_.cancel();
    if (thread_.joinable()) {
      thread_.join();
    }
  }

private:
  rclcpp::executors::MultiThreadedExecutor executor_;
  std::thread thread_;
};

class RclcppFixture : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    int argc = 0;
    char ** argv = nullptr;
    rclcpp::init(argc, argv);
  }

  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }
};

}  // namespace manipulation::moveit::test

#endif  // ACTION_TEST_SUPPORT_HPP_
