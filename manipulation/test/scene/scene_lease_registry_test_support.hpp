// Responsibility: Provides canonical resources and builders for scene-lease registry tests.

#ifndef SCENE__SCENE_LEASE_REGISTRY_TEST_SUPPORT_HPP_
#define SCENE__SCENE_LEASE_REGISTRY_TEST_SUPPORT_HPP_

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "manipulation/scene/scene_lease_registry.hpp"

namespace manipulation::scene::test
{

struct ManualClock
{
  SceneLeaseRegistry::TimePoint now{};

  void advance(const SceneLeaseRegistry::Duration duration)
  {
    now += duration;
  }
};

inline std::unique_ptr<SceneLeaseRegistry> make_registry(
  ManualClock & clock,
  const std::size_t operation_capacity = 4096,
  const std::uint64_t initial_revision = 0)
{
  SceneLeaseRegistry::Options options;
  options.session_id = "test-session";
  options.initial_revision = initial_revision;
  options.max_operation_records = operation_capacity;
  return std::make_unique<SceneLeaseRegistry>(
    std::move(options), [&clock]() {return clock.now;});
}

inline AcquireLeaseRequest lease_request(
  std::string owner,
  std::vector<std::string> resources,
  const SceneLeaseRegistry::Duration ttl = std::chrono::seconds{10})
{
  return AcquireLeaseRequest{std::move(owner), std::move(resources), ttl};
}

inline BeginOperationRequest operation_request(
  const std::string & lease_token,
  std::string operation_id,
  const std::uint64_t revision,
  std::vector<std::string> resources,
  std::string digest)
{
  return BeginOperationRequest{
    lease_token, std::move(operation_id), revision, std::move(resources), std::move(digest)};
}

}  // namespace manipulation::scene::test

#endif  // SCENE__SCENE_LEASE_REGISTRY_TEST_SUPPORT_HPP_
