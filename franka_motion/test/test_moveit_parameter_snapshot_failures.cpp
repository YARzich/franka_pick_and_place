// Responsibility: Verifies fail-closed and rollback behavior for incomplete MoveIt snapshots.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <string>
#include <thread>
#include <vector>

#include "gtest/gtest.h"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/parameter_client.hpp"

#include "franka_motion/moveit_parameter_snapshot_contract.hpp"
#include "franka_motion/moveit_parameter_sync.hpp"

#include "moveit_parameter_sync_test_support.hpp"
#include "parameter_snapshot_client.hpp"

namespace
{

using franka_motion::test_support::minimal_sync_options;
using franka_motion::test_support::NodeSpinner;
using franka_motion::test_support::RclcppTest;
using franka_motion::detail::ParameterSnapshotClient;
using namespace std::chrono_literals;

TEST_F(RclcppTest, LegacyRequiredPrefixRejectsUnknownUninitializedParameter)
{
  const std::string initialized_name = "legacy_required.initialized";
  const std::string uninitialized_name = "legacy_required.uninitialized";
  auto source = rclcpp::Node::make_shared("test_legacy_incomplete_source");
  source->declare_parameter<std::string>(initialized_name, "must_not_be_copied");
  source->declare_parameter(
    uninitialized_name, rclcpp::ParameterType::PARAMETER_STRING);
  NodeSpinner spinner(source);

  auto destination = rclcpp::Node::make_shared("test_legacy_incomplete_destination");
  auto options = minimal_sync_options(
    "test_legacy_incomplete_source", "test_legacy_incomplete_helper");
  options.required_prefixes = {"legacy_required"};

  EXPECT_FALSE(franka_motion::copy_moveit_parameters_from_move_group(
      destination, options));
  EXPECT_FALSE(destination->has_parameter(initialized_name));
  EXPECT_FALSE(destination->has_parameter(uninitialized_name));
}

TEST_F(RclcppTest, RequiredPrefixRejectsSnapshotContainingOnlyKnownUninitializedLimits)
{
  const std::string uninitialized_name =
    "robot_description_planning.joint_limits.panda_joint1.min_position";
  auto source = rclcpp::Node::make_shared("test_empty_legacy_limit_source");
  source->declare_parameter(
    uninitialized_name, rclcpp::ParameterType::PARAMETER_DOUBLE);
  NodeSpinner spinner(source);

  auto destination = rclcpp::Node::make_shared("test_empty_legacy_limit_destination");
  auto options = minimal_sync_options(
    "test_empty_legacy_limit_source", "test_empty_legacy_limit_helper");
  options.required_prefixes = {"robot_description_planning"};
  options.legacy_uninitialized_parameter_names =
    franka_motion::panda_urdf_position_limit_parameter_names();

  EXPECT_FALSE(franka_motion::copy_moveit_parameters_from_move_group(
      destination, options));
  EXPECT_FALSE(destination->has_parameter(uninitialized_name));
}

TEST_F(RclcppTest, ManifestDoesNotPermitKnownUninitializedLimit)
{
  const std::string initialized_name =
    "robot_description_planning.joint_limits.panda_joint1.max_acceleration";
  const std::string uninitialized_name =
    "robot_description_planning.joint_limits.panda_joint1.min_position";
  auto source = rclcpp::Node::make_shared("test_manifest_limit_source");
  source->declare_parameter<double>(initialized_name, 2.5);
  source->declare_parameter(
    uninitialized_name, rclcpp::ParameterType::PARAMETER_DOUBLE);
  source->declare_parameter<std::vector<std::string>>(
    franka_motion::kMoveItParameterSnapshotManifest,
    {initialized_name, uninitialized_name});
  NodeSpinner spinner(source);

  auto destination = rclcpp::Node::make_shared("test_manifest_limit_destination");
  auto options = minimal_sync_options(
    "test_manifest_limit_source", "test_manifest_limit_helper");
  options.required_prefixes = {"robot_description_planning"};
  options.legacy_uninitialized_parameter_names =
    franka_motion::panda_urdf_position_limit_parameter_names();

  EXPECT_FALSE(franka_motion::copy_moveit_parameters_from_move_group(
      destination, options));
  EXPECT_FALSE(destination->has_parameter(initialized_name));
  EXPECT_FALSE(destination->has_parameter(uninitialized_name));
}

TEST_F(RclcppTest, ManifestRejectsUndeclaredRequiredMember)
{
  const std::string missing_name = "manifest_required.missing";
  auto source = rclcpp::Node::make_shared("test_manifest_missing_source");
  source->declare_parameter<std::vector<std::string>>(
    franka_motion::kMoveItParameterSnapshotManifest, {missing_name});
  NodeSpinner spinner(source);

  auto destination = rclcpp::Node::make_shared("test_manifest_missing_destination");
  auto options = minimal_sync_options(
    "test_manifest_missing_source", "test_manifest_missing_helper");
  options.required_prefixes = {"manifest_required"};

  EXPECT_FALSE(franka_motion::copy_moveit_parameters_from_move_group(
      destination, options));
  EXPECT_FALSE(destination->has_parameter(missing_name));
}

TEST_F(RclcppTest, ManifestRequestTimeoutFailsClosed)
{
  auto source = rclcpp::Node::make_shared("test_unresponsive_parent_parameter_source");
  source->declare_parameter<std::string>("robot_description", "must_not_be_copied");

  auto probe = rclcpp::Node::make_shared("test_unresponsive_parent_parameter_probe");
  auto probe_client = std::make_shared<rclcpp::AsyncParametersClient>(
    probe, "test_unresponsive_parent_parameter_source");
  ASSERT_TRUE(probe_client->wait_for_service(1s));

  auto destination = rclcpp::Node::make_shared(
    "test_unresponsive_parent_parameter_destination");
  auto options = minimal_sync_options(
    "test_unresponsive_parent_parameter_source",
    "test_unresponsive_parent_parameter_helper");
  options.required_names = {"robot_description"};

  EXPECT_FALSE(franka_motion::copy_moveit_parameters_from_move_group(
      destination, options));
  EXPECT_FALSE(destination->has_parameter("robot_description"));
}

TEST_F(RclcppTest, ParameterReadRetriesAfterFirstResponseTimeout)
{
  auto source = rclcpp::Node::make_shared("test_delayed_manifest_source");
  source->declare_parameter<std::vector<std::string>>(
    franka_motion::kMoveItParameterSnapshotManifest, {"retry_succeeded"});

  auto helper = rclcpp::Node::make_shared("test_delayed_manifest_helper");
  ParameterSnapshotClient client(helper, "test_delayed_manifest_source");
  ASSERT_TRUE(client.wait_for_service(1s));

  // Discovery is complete before the timer starts. The source deliberately
  // begins processing after the two-second first-attempt deadline, so only the
  // second request can produce the successful result.
  NodeSpinner spinner(source, 3s);
  const auto started = ParameterSnapshotClient::Clock::now();
  const auto parameters = client.get_parameters_with_retry(
    {franka_motion::kMoveItParameterSnapshotManifest}, started + 4s, 2s);
  const auto elapsed = ParameterSnapshotClient::Clock::now() - started;

  ASSERT_TRUE(parameters.has_value());
  ASSERT_EQ(parameters->size(), 1U);
  EXPECT_EQ(parameters->front().as_string_array(), std::vector<std::string>{"retry_succeeded"});
  EXPECT_GE(elapsed, 2s);
  EXPECT_LT(elapsed, 4s);
}

TEST_F(RclcppTest, ParameterReadRetryDoesNotResetOriginalDeadline)
{
  auto source = rclcpp::Node::make_shared("test_bounded_retry_source");
  auto helper = rclcpp::Node::make_shared("test_bounded_retry_helper");
  ParameterSnapshotClient client(helper, "test_bounded_retry_source");
  ASSERT_TRUE(client.wait_for_service(1s));

  // Leave the discoverable source unspun so neither request can answer.
  const auto started = ParameterSnapshotClient::Clock::now();
  const auto parameters = client.get_parameters_with_retry(
    {franka_motion::kMoveItParameterSnapshotManifest}, started + 800ms, 500ms);
  const auto elapsed = ParameterSnapshotClient::Clock::now() - started;

  EXPECT_FALSE(parameters.has_value());
  EXPECT_GE(elapsed, 700ms);
  // Resetting the second attempt to a new 800 ms budget would take about
  // 1300 ms, which must not fit this bound.
  EXPECT_LT(elapsed, 1100ms);
}

TEST_F(RclcppTest, StopPredicateCancelsServiceDiscoveryWithoutDeadlineDelay)
{
  auto destination = rclcpp::Node::make_shared("test_canceled_snapshot_destination");
  auto options = minimal_sync_options(
    "test_missing_snapshot_source", "test_canceled_snapshot_helper");
  options.service_timeout = 30s;
  std::atomic<bool> stop_requested{false};
  std::thread stop_thread([&stop_requested]() {
      std::this_thread::sleep_for(50ms);
      stop_requested.store(true);
    });

  const auto started = std::chrono::steady_clock::now();
  const bool copied = franka_motion::copy_moveit_parameters_from_move_group(
    destination, options, [&stop_requested]() {return stop_requested.load();});
  const auto elapsed = std::chrono::steady_clock::now() - started;
  stop_thread.join();

  EXPECT_FALSE(copied);
  EXPECT_TRUE(stop_requested.load());
  EXPECT_LT(elapsed, 1s);
}

TEST_F(RclcppTest, IncompleteOptionalPrefixIsNotPartiallyApplied)
{
  constexpr char kPrefix[] = "optional_snapshot";
  const std::string copied_name = std::string(kPrefix) + ".a_value";
  const std::string missing_name = std::string(kPrefix) + ".z_uninitialized";
  auto source = rclcpp::Node::make_shared("test_optional_snapshot_source");
  source->declare_parameter<std::string>(copied_name, "must_not_be_copied");
  source->declare_parameter(missing_name, rclcpp::ParameterType::PARAMETER_STRING);
  source->declare_parameter<std::vector<std::string>>(
    franka_motion::kMoveItParameterSnapshotManifest,
    {copied_name, missing_name});
  NodeSpinner spinner(source);

  auto destination = rclcpp::Node::make_shared("test_optional_snapshot_destination");
  auto options = minimal_sync_options(
    "test_optional_snapshot_source", "test_optional_snapshot_helper");
  options.optional_prefixes = {kPrefix};

  EXPECT_TRUE(franka_motion::copy_moveit_parameters_from_move_group(
      destination, options));
  EXPECT_FALSE(destination->has_parameter(copied_name));
  EXPECT_FALSE(destination->has_parameter(missing_name));
}

TEST_F(RclcppTest, IncompleteRequiredPrefixFailsWithoutPartialApplication)
{
  constexpr char kPrefix[] = "required_snapshot";
  const std::string copied_name = std::string(kPrefix) + ".a_value";
  const std::string missing_name = std::string(kPrefix) + ".z_uninitialized";
  auto source = rclcpp::Node::make_shared("test_required_snapshot_source");
  source->declare_parameter<std::string>(copied_name, "must_not_be_copied");
  source->declare_parameter(missing_name, rclcpp::ParameterType::PARAMETER_STRING);
  source->declare_parameter<std::vector<std::string>>(
    franka_motion::kMoveItParameterSnapshotManifest,
    {copied_name, missing_name});
  NodeSpinner spinner(source);

  auto destination = rclcpp::Node::make_shared("test_required_snapshot_destination");
  auto options = minimal_sync_options(
    "test_required_snapshot_source", "test_required_snapshot_helper");
  options.required_prefixes = {kPrefix};

  EXPECT_FALSE(franka_motion::copy_moveit_parameters_from_move_group(
      destination, options));
  EXPECT_FALSE(destination->has_parameter(copied_name));
  EXPECT_FALSE(destination->has_parameter(missing_name));
}

TEST_F(RclcppTest, RequiredNamedSnapshotRejectionFailsClosed)
{
  const std::string accepted_name = "required_named.accepted";
  const std::string rejected_name = "required_named.rejected";
  auto source = rclcpp::Node::make_shared("test_required_named_rejection_source");
  source->declare_parameter<std::string>(accepted_name, "accepted");
  source->declare_parameter<std::string>(rejected_name, "rejected");
  NodeSpinner spinner(source);

  auto destination = rclcpp::Node::make_shared(
    "test_required_named_rejection_destination");
  const auto callback = destination->add_on_set_parameters_callback(
    [&rejected_name](const std::vector<rclcpp::Parameter> & parameters) {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = std::none_of(
        parameters.begin(), parameters.end(), [&rejected_name](const auto & parameter) {
          return parameter.get_name() == rejected_name &&
                 parameter.get_type() != rclcpp::ParameterType::PARAMETER_NOT_SET;
        });
      result.reason = result.successful ? "" : "test rejection";
      return result;
    });
  ASSERT_NE(callback, nullptr);
  auto options = minimal_sync_options(
    "test_required_named_rejection_source",
    "test_required_named_rejection_helper");
  options.required_names = {accepted_name, rejected_name};

  EXPECT_FALSE(franka_motion::copy_moveit_parameters_from_move_group(
      destination, options));
  EXPECT_FALSE(destination->has_parameter(accepted_name));
  EXPECT_FALSE(destination->has_parameter(rejected_name));
}

TEST_F(RclcppTest, CompletePrefixRejectionRollsBackTheWholeSnapshot)
{
  constexpr char kPrefix[] = "rejected_snapshot";
  const std::string accepted_name = std::string(kPrefix) + ".accepted";
  const std::string rejected_name = std::string(kPrefix) + ".rejected";
  auto source = rclcpp::Node::make_shared("test_prefix_rejection_source");
  source->declare_parameter<std::string>(accepted_name, "accepted");
  source->declare_parameter<std::string>(rejected_name, "rejected");
  source->declare_parameter<std::vector<std::string>>(
    franka_motion::kMoveItParameterSnapshotManifest,
    {accepted_name, rejected_name});
  NodeSpinner spinner(source);

  auto destination = rclcpp::Node::make_shared("test_prefix_rejection_destination");
  const auto callback = destination->add_on_set_parameters_callback(
    [&rejected_name](const std::vector<rclcpp::Parameter> & parameters) {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = std::none_of(
        parameters.begin(), parameters.end(), [&rejected_name](const auto & parameter) {
          return parameter.get_name() == rejected_name &&
                 parameter.get_type() != rclcpp::ParameterType::PARAMETER_NOT_SET;
        });
      result.reason = result.successful ? "" : "test rejection";
      return result;
    });
  ASSERT_NE(callback, nullptr);
  auto options = minimal_sync_options(
    "test_prefix_rejection_source", "test_prefix_rejection_helper");
  options.required_prefixes = {kPrefix};

  EXPECT_FALSE(franka_motion::copy_moveit_parameters_from_move_group(
      destination, options));
  EXPECT_FALSE(destination->has_parameter(accepted_name));
  EXPECT_FALSE(destination->has_parameter(rejected_name));
}

}  // namespace
