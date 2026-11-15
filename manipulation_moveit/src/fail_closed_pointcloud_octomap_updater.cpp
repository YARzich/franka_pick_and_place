// Responsibility: Suppresses occupancy updates whenever robot-shape transforms are incomplete.

#include "fail_closed_pointcloud_octomap_updater.hpp"

#include <algorithm>
#include <cinttypes>
#include <string>
#include <utility>

#include "pluginlib/class_list_macros.hpp"

namespace manipulation::moveit
{

bool hasCompleteShapeTransformCache(
  const ShapeHandleSet & active_handles,
  const occupancy_map_monitor::ShapeTransformCache & transform_cache)
{
  if (active_handles.empty() || active_handles.size() != transform_cache.size()) {
    return false;
  }
  return std::all_of(
    active_handles.begin(), active_handles.end(),
    [&transform_cache](const auto handle) {
      return transform_cache.find(handle) != transform_cache.end();
    });
}

FailClosedPointCloudOctomapUpdater::FailClosedPointCloudOctomapUpdater()
: logger_(rclcpp::get_logger("manipulation_moveit.fail_closed_octomap"))
{
}

void FailClosedPointCloudOctomapUpdater::start()
{
  wrapTransformProvider();
  occupancy_map_monitor::PointCloudOctomapUpdater::start();
  RCLCPP_INFO(
    logger_,
    "Fail-closed Octomap self-filter enabled; partial shape TF caches are rejected");
}

occupancy_map_monitor::ShapeHandle
FailClosedPointCloudOctomapUpdater::excludeShape(
  const shapes::ShapeConstPtr & shape)
{
  std::unique_lock<std::mutex> lock(active_shapes_mutex_);
  shape_lifecycle_condition_.wait(lock, [this]() {return !mask_in_progress_;});
  const auto handle =
    occupancy_map_monitor::PointCloudOctomapUpdater::excludeShape(shape);
  if (handle != 0U) {
    if (active_shape_handles_.insert(handle).second) {
      ++shape_generation_;
    }
  }
  return handle;
}

void FailClosedPointCloudOctomapUpdater::forgetShape(
  const occupancy_map_monitor::ShapeHandle handle)
{
  std::unique_lock<std::mutex> lock(active_shapes_mutex_);
  shape_lifecycle_condition_.wait(lock, [this]() {return !mask_in_progress_;});
  occupancy_map_monitor::PointCloudOctomapUpdater::forgetShape(handle);
  if (active_shape_handles_.erase(handle) > 0U) {
    ++shape_generation_;
  }
}

void FailClosedPointCloudOctomapUpdater::updateMask(
  const sensor_msgs::msg::PointCloud2 & cloud,
  const Eigen::Vector3d & sensor_origin,
  std::vector<int> & mask)
{
  occupancy_map_monitor::PointCloudOctomapUpdater::updateMask(
    cloud, sensor_origin, mask);
  {
    std::lock_guard<std::mutex> lock(active_shapes_mutex_);
    mask_in_progress_ = false;
  }
  shape_lifecycle_condition_.notify_all();
}

void FailClosedPointCloudOctomapUpdater::wrapTransformProvider()
{
  if (transform_provider_wrapped_) {
    return;
  }
  const auto upstream_provider = transform_provider_callback_;
  transform_provider_callback_ =
    [this, upstream_provider](
    const std::string & target_frame,
    const rclcpp::Time & target_time,
    occupancy_map_monitor::ShapeTransformCache & output_cache)
    {
      std::uint64_t shape_generation = 0U;
      {
        std::unique_lock<std::mutex> lock(active_shapes_mutex_);
        shape_lifecycle_condition_.wait(
          lock, [this]() {return !mask_in_progress_;});
        shape_generation = shape_generation_;
      }

      occupancy_map_monitor::ShapeTransformCache candidate_cache;
      if (!upstream_provider ||
        !upstream_provider(target_frame, target_time, candidate_cache))
      {
        output_cache.clear();
        return false;
      }

      std::size_t active_shape_count = 0U;
      bool complete = false;
      {
        std::unique_lock<std::mutex> lock(active_shapes_mutex_);
        shape_lifecycle_condition_.wait(
          lock, [this]() {return !mask_in_progress_;});
        active_shape_count = active_shape_handles_.size();
        complete = shape_generation == shape_generation_ &&
          hasCompleteShapeTransformCache(active_shape_handles_, candidate_cache);
        mask_in_progress_ = complete;
      }
      if (!complete) {
        const auto rejected =
          rejected_cloud_count_.fetch_add(1U, std::memory_order_relaxed) + 1U;
        RCLCPP_WARN_THROTTLE(
          logger_, steady_clock_, 2000,
          "Rejecting Octomap cloud: shape TF cache is incomplete (%zu/%zu, "
          "rejected=%" PRIu64 ")",
          candidate_cache.size(), active_shape_count, rejected);
        output_cache.clear();
        return false;
      }

      output_cache = std::move(candidate_cache);
      return true;
    };
  transform_provider_wrapped_ = true;
}

}  // namespace manipulation::moveit

PLUGINLIB_EXPORT_CLASS(
  manipulation::moveit::FailClosedPointCloudOctomapUpdater,
  occupancy_map_monitor::OccupancyMapUpdater)
