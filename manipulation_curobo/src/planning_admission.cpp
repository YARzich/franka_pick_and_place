// Responsibility: Implements shared permit accounting for bounded parallel planning requests.

#include "manipulation_curobo/planning_admission.hpp"

#include <mutex>
#include <stdexcept>
#include <utility>

namespace manipulation_curobo::curobo
{

struct PlanningAdmissionGate::State
{
  explicit State(const std::size_t configured_capacity)
  : capacity(configured_capacity)
  {
  }

  const std::size_t capacity;
  std::size_t active{0U};
  mutable std::mutex mutex;
};

PlanningAdmissionGate::Permit::Permit(std::shared_ptr<State> state) noexcept
: state_(std::move(state))
{
}

PlanningAdmissionGate::Permit::~Permit() noexcept
{
  release();
}

PlanningAdmissionGate::Permit::Permit(Permit && other) noexcept
: state_(std::move(other.state_))
{
}

PlanningAdmissionGate::Permit & PlanningAdmissionGate::Permit::operator=(
  Permit && other) noexcept
{
  if (this != &other) {
    release();
    state_ = std::move(other.state_);
  }
  return *this;
}

void PlanningAdmissionGate::Permit::release() noexcept
{
  if (!state_) {
    return;
  }
  {
    std::lock_guard<std::mutex> lock(state_->mutex);
    if (state_->active > 0U) {
      --state_->active;
    }
  }
  state_.reset();
}

PlanningAdmissionGate::PlanningAdmissionGate(const std::size_t capacity)
{
  if (capacity == 0U) {
    throw std::invalid_argument("planning admission capacity must be positive");
  }
  state_ = std::make_shared<State>(capacity);
}

std::optional<PlanningAdmissionGate::Permit> PlanningAdmissionGate::tryAcquire() const
{
  std::lock_guard<std::mutex> lock(state_->mutex);
  if (state_->active >= state_->capacity) {
    return std::nullopt;
  }
  ++state_->active;
  return Permit{state_};
}

std::size_t PlanningAdmissionGate::capacity() const noexcept
{
  return state_->capacity;
}

std::size_t PlanningAdmissionGate::activeCount() const
{
  std::lock_guard<std::mutex> lock(state_->mutex);
  return state_->active;
}

}  // namespace manipulation_curobo::curobo
