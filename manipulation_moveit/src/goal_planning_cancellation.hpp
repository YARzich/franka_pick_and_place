// Responsibility: Own exact-goal cancellation across delayed planning-context creation.

#ifndef GOAL_PLANNING_CANCELLATION_HPP_
#define GOAL_PLANNING_CANCELLATION_HPP_

#include <array>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <mutex>

namespace manipulation::moveit::detail
{

class GoalPlanningCancellation final
{
public:
  using GoalId = std::array<std::uint8_t, 16>;
  explicit GoalPlanningCancellation(GoalId goal_id);
  bool request(const GoalId & goal_id);
  void request_shutdown();
  [[nodiscard]] bool requested() const;
  // The planning worker retains ownership until run and its cancellation watcher return.
  bool run(const std::function<void()> & plan, const std::function<void()> & terminate);
  // Linearizes terminal selection against late callbacks for this exact goal.
  bool commit_terminal();

private:
  const GoalId goal_id_;
  mutable std::mutex mutex_;
  std::condition_variable condition_;
  bool requested_{false};
  bool terminal_committed_{false};
};

}  // namespace manipulation::moveit::detail
#endif  // GOAL_PLANNING_CANCELLATION_HPP_
