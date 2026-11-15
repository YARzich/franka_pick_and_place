// Responsibility: Verifies bounded cuRobo planning admission and RAII permit release under races.

#include <gtest/gtest.h>

#include <atomic>
#include <cstddef>
#include <stdexcept>
#include <thread>
#include <vector>

#include "manipulation_curobo/planning_admission.hpp"

namespace manipulation_curobo::curobo
{
namespace
{

TEST(PlanningAdmissionGate, RejectsInvalidCapacity)
{
  EXPECT_THROW((void)PlanningAdmissionGate{0U}, std::invalid_argument);
}

TEST(PlanningAdmissionGate, ReleasesPermitAtScopeExit)
{
  PlanningAdmissionGate gate{1U};
  {
    auto permit = gate.tryAcquire();
    ASSERT_TRUE(permit.has_value());
    EXPECT_EQ(gate.activeCount(), 1U);
    EXPECT_FALSE(gate.tryAcquire().has_value());
  }

  EXPECT_EQ(gate.activeCount(), 0U);
  EXPECT_TRUE(gate.tryAcquire().has_value());
}

TEST(PlanningAdmissionGate, ReleasesPermitDuringExceptionUnwinding)
{
  PlanningAdmissionGate gate{1U};
  EXPECT_THROW(
      {
        auto permit = gate.tryAcquire();
        if (!permit) {
          throw std::logic_error("test setup could not acquire planning permit");
        }
        throw std::runtime_error("test failure path");
    },
    std::runtime_error);

  EXPECT_EQ(gate.activeCount(), 0U);
  EXPECT_TRUE(gate.tryAcquire().has_value());
}

TEST(PlanningAdmissionGate, ConcurrentCallersFailFastWhileSlotIsOwned)
{
  constexpr std::size_t kThreadCount = 32U;
  PlanningAdmissionGate gate{1U};
  auto owner = gate.tryAcquire();
  ASSERT_TRUE(owner.has_value());

  std::atomic_bool start{false};
  std::atomic_size_t busy_count{0U};
  std::vector<std::thread> threads;
  threads.reserve(kThreadCount);
  for (std::size_t index = 0U; index < kThreadCount; ++index) {
    threads.emplace_back([&gate, &start, &busy_count]() {
        while (!start.load(std::memory_order_acquire)) {
          std::this_thread::yield();
        }
        if (!gate.tryAcquire().has_value()) {
          busy_count.fetch_add(1U, std::memory_order_relaxed);
        }
    });
  }

  start.store(true, std::memory_order_release);
  for (auto & thread : threads) {
    thread.join();
  }

  EXPECT_EQ(busy_count.load(), kThreadCount);
  EXPECT_EQ(gate.activeCount(), 1U);
}

TEST(PlanningAdmissionGate, HonorsConfiguredCapacity)
{
  PlanningAdmissionGate gate{2U};
  auto first = gate.tryAcquire();
  auto second = gate.tryAcquire();

  ASSERT_TRUE(first.has_value());
  ASSERT_TRUE(second.has_value());
  EXPECT_FALSE(gate.tryAcquire().has_value());
  EXPECT_EQ(gate.activeCount(), 2U);
}

}  // namespace
}  // namespace manipulation_curobo::curobo
