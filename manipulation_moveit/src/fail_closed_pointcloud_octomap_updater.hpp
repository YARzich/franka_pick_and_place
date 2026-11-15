// Responsibility: Declares the PointCloudOctomapUpdater variant that fails closed on missing shape
//   TF.

#ifndef FAIL_CLOSED_POINTCLOUD_OCTOMAP_UPDATER_HPP_
#define FAIL_CLOSED_POINTCLOUD_OCTOMAP_UPDATER_HPP_

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <set>
#include <vector>

#include "moveit/pointcloud_octomap_updater/pointcloud_octomap_updater.hpp"
#include "rclcpp/rclcpp.hpp"

namespace manipulation::moveit
{

using ShapeHandleSet = std::set<occupancy_map_monitor::ShapeHandle>;

/**
 * Return true only when every active exclusion shape has exactly one transform.
 *
 * An empty active set fails closed because it cannot prove robot self-filtering
 * is configured; extra cache entries also fail to prevent stale-handle reuse.
 */
bool hasCompleteShapeTransformCache(
  const ShapeHandleSet & active_handles,
  const occupancy_map_monitor::ShapeTransformCache & transform_cache);

/**
 * PointCloudOctomapUpdater variant which rejects partial self-filter caches.
 *
 * MoveIt 2.12 can report a successful transform-cache update after skipping
 * individual robot shapes. The stock updater then treats points on those
 * shapes as obstacles. This adapter keeps the upstream ray-casting and masking
 * implementation, but turns an incomplete cache into a failed cloud update.
 *
 * The occupancy monitor owns this plugin instance and may call shape lifecycle
 * methods concurrently with the point-cloud callback. `active_shapes_mutex_`
 * protects handles, generation and the validation-to-mask handoff. Shape add or
 * removal waits until the accepted cloud has consumed the exact cache; the
 * upstream transform provider is called without this mutex to avoid lock-order
 * inversion with MoveIt's PlanningSceneMonitor. Rejected clouds never reach the
 * Octomap mutation path.
 */
class FailClosedPointCloudOctomapUpdater final
  : public occupancy_map_monitor::PointCloudOctomapUpdater
{
public:
  FailClosedPointCloudOctomapUpdater();

  void start() override;
  occupancy_map_monitor::ShapeHandle excludeShape(
    const shapes::ShapeConstPtr & shape) override;
  void forgetShape(occupancy_map_monitor::ShapeHandle handle) override;

protected:
  void updateMask(
    const sensor_msgs::msg::PointCloud2 & cloud,
    const Eigen::Vector3d & sensor_origin,
    std::vector<int> & mask) override;

private:
  void wrapTransformProvider();

  std::mutex active_shapes_mutex_;
  std::condition_variable shape_lifecycle_condition_;
  ShapeHandleSet active_shape_handles_;
  std::uint64_t shape_generation_{0};
  bool mask_in_progress_{false};
  bool transform_provider_wrapped_{false};
  std::atomic<std::uint64_t> rejected_cloud_count_{0};
  rclcpp::Logger logger_;
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
};

}  // namespace manipulation::moveit

#endif  // FAIL_CLOSED_POINTCLOUD_OCTOMAP_UPDATER_HPP_
