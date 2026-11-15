// Responsibility: Provides process-local exclusive ownership across sequence action facades.

#ifndef MANIPULATION__WORKFLOW__TRANSFER_SEQUENCE_GOAL_RESERVATION_HPP_
#define MANIPULATION__WORKFLOW__TRANSFER_SEQUENCE_GOAL_RESERVATION_HPP_

#include <cstdint>
#include <memory>
#include <optional>
#include <string>

namespace manipulation::workflow
{

/**
 * Grant non-blocking, process-local exclusive ownership to one sequence goal.
 *
 * A successful try_acquire() returns a move-only Lease. Destroying that lease
 * releases only the generation it acquired, so a stale token cannot release a
 * newer owner. The shared internal state lets a lease safely outlive this
 * wrapper. There is deliberately no queue, preemption, or global state.
 */
class TransferSequenceGoalReservation final
{
private:
  struct State;

public:
  /** Thread-safe diagnostic view of the shared sequence goal reservation. */
  struct Snapshot
  {
    bool reserved{false};
    std::uint64_t generation{0U};
    std::string owner;
  };

  /** Move-only RAII token for one successfully reserved generation. */
  class Lease final
  {
public:
    Lease(const Lease &) = delete;
    Lease & operator=(const Lease &) = delete;
    Lease(Lease && other) noexcept;
    Lease & operator=(Lease && other) noexcept;
    ~Lease() noexcept;

    /// Return true while this token still owns its acquired generation.
    [[nodiscard]] bool valid() const noexcept;

    /// Return the acquired generation, or zero for a moved-from token.
    [[nodiscard]] std::uint64_t generation() const noexcept;

private:
    friend class TransferSequenceGoalReservation;

    Lease(std::shared_ptr<State> state, std::uint64_t generation) noexcept;
    void release() noexcept;

    std::shared_ptr<State> state_;
    std::uint64_t generation_{0U};
  };

  TransferSequenceGoalReservation();
  ~TransferSequenceGoalReservation();

  TransferSequenceGoalReservation(const TransferSequenceGoalReservation &) = delete;
  TransferSequenceGoalReservation & operator=(const TransferSequenceGoalReservation &) = delete;
  TransferSequenceGoalReservation(TransferSequenceGoalReservation &&) = delete;
  TransferSequenceGoalReservation & operator=(TransferSequenceGoalReservation &&) = delete;

  /**
   * Try to reserve the slot without waiting.
   *
   * Returns an empty optional while another lease owns the slot. The owner
   * must contain at least one non-whitespace character; invalid names throw
   * std::invalid_argument.
   */
  [[nodiscard]] std::optional<Lease> try_acquire(std::string owner);

  /// Capture the current owner and the last successfully acquired generation.
  [[nodiscard]] Snapshot snapshot() const;

private:
  std::shared_ptr<State> state_;
};

}  // namespace manipulation::workflow

#endif  // MANIPULATION__WORKFLOW__TRANSFER_SEQUENCE_GOAL_RESERVATION_HPP_
