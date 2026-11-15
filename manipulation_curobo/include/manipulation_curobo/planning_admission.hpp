// Responsibility: Enforces the configured concurrency limit for external cuRobo planning jobs.

#ifndef MANIPULATION_CUROBO__PLANNING_ADMISSION_HPP_
#define MANIPULATION_CUROBO__PLANNING_ADMISSION_HPP_

#include <cstddef>
#include <memory>
#include <optional>

namespace manipulation_curobo::curobo
{

/**
 * Bound concurrent access to an expensive planner backend.
 *
 * tryAcquire() only holds an internal mutex while updating a counter. A
 * successful caller owns a move-only Permit whose destructor returns the slot,
 * so backend work and cancellation never run while the gate mutex is held.
 */
class PlanningAdmissionGate final
{
private:
  struct State;

public:
  /** Move-only ownership of one admitted planning slot. */
  class Permit final
  {
public:
    ~Permit() noexcept;

    Permit(const Permit &) = delete;
    Permit & operator=(const Permit &) = delete;
    Permit(Permit && other) noexcept;
    Permit & operator=(Permit && other) noexcept;

private:
    friend class PlanningAdmissionGate;

    explicit Permit(std::shared_ptr<State> state) noexcept;
    void release() noexcept;

    std::shared_ptr<State> state_;
  };

  /** Create a gate with at least one concurrent planning slot. */
  explicit PlanningAdmissionGate(std::size_t capacity);

  PlanningAdmissionGate(const PlanningAdmissionGate &) = delete;
  PlanningAdmissionGate & operator=(const PlanningAdmissionGate &) = delete;
  PlanningAdmissionGate(PlanningAdmissionGate &&) = delete;
  PlanningAdmissionGate & operator=(PlanningAdmissionGate &&) = delete;

  /**
   * Acquire a slot without waiting.
   *
   * @return A permit when capacity is available, otherwise std::nullopt.
   */
  [[nodiscard]] std::optional<Permit> tryAcquire() const;

  /** Return the immutable configured capacity. */
  [[nodiscard]] std::size_t capacity() const noexcept;

  /** Return the current number of permits, primarily for diagnostics/tests. */
  [[nodiscard]] std::size_t activeCount() const;

private:
  std::shared_ptr<State> state_;
};

}  // namespace manipulation_curobo::curobo

#endif  // MANIPULATION_CUROBO__PLANNING_ADMISSION_HPP_
