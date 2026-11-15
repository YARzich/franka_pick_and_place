// Responsibility: Implements token-checked exclusive sequence goal reservation.

#include "manipulation/workflow/transfer_sequence_goal_reservation.hpp"

#include <algorithm>
#include <cctype>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <utility>

namespace manipulation::workflow
{

struct TransferSequenceGoalReservation::State final
{
  void release(const std::uint64_t token_generation) noexcept
  {
    try {
      const std::lock_guard<std::mutex> lock(mutex);
      if (!reserved || generation != token_generation) {
        return;
      }
      reserved = false;
      owner.clear();
    } catch (...) {
      // Retaining the reservation is safer than releasing an unverified owner.
    }
  }

  mutable std::mutex mutex;
  bool reserved{false};
  std::uint64_t generation{0U};
  std::string owner;
};

TransferSequenceGoalReservation::Lease::Lease(
  std::shared_ptr<State> state, const std::uint64_t generation) noexcept
: state_(std::move(state)), generation_(generation)
{
}

TransferSequenceGoalReservation::Lease::Lease(Lease && other) noexcept
: state_(std::move(other.state_)),
  generation_(std::exchange(other.generation_, 0U))
{
}

TransferSequenceGoalReservation::Lease &
TransferSequenceGoalReservation::Lease::operator=(Lease && other) noexcept
{
  if (this == &other) {
    return *this;
  }
  release();
  state_ = std::move(other.state_);
  generation_ = std::exchange(other.generation_, 0U);
  return *this;
}

TransferSequenceGoalReservation::Lease::~Lease() noexcept
{
  release();
}

bool TransferSequenceGoalReservation::Lease::valid() const noexcept
{
  return state_ != nullptr && generation_ != 0U;
}

std::uint64_t TransferSequenceGoalReservation::Lease::generation() const noexcept
{
  return generation_;
}

void TransferSequenceGoalReservation::Lease::release() noexcept
{
  if (state_) {
    state_->release(generation_);
  }
  state_.reset();
  generation_ = 0U;
}

TransferSequenceGoalReservation::TransferSequenceGoalReservation()
: state_(std::make_shared<State>())
{
}

TransferSequenceGoalReservation::~TransferSequenceGoalReservation() = default;

std::optional<TransferSequenceGoalReservation::Lease>
TransferSequenceGoalReservation::try_acquire(std::string owner)
{
  const auto contains_non_whitespace = std::any_of(
    owner.cbegin(), owner.cend(),
    [](const unsigned char character) {return std::isspace(character) == 0;});
  if (!contains_non_whitespace) {
    throw std::invalid_argument(
            "sequence reservation owner must contain a non-whitespace character");
  }

  const std::lock_guard<std::mutex> lock(state_->mutex);
  if (state_->reserved ||
    state_->generation == std::numeric_limits<std::uint64_t>::max())
  {
    return std::nullopt;
  }

  ++state_->generation;
  state_->owner = std::move(owner);
  state_->reserved = true;
  return Lease{state_, state_->generation};
}

TransferSequenceGoalReservation::Snapshot
TransferSequenceGoalReservation::snapshot() const
{
  const std::lock_guard<std::mutex> lock(state_->mutex);
  return Snapshot{state_->reserved, state_->generation, state_->owner};
}

}  // namespace manipulation::workflow
