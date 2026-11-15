"""
ROS output ownership for the stereo perception adapter.

Responsibility:
- Create stereo output publishers with their public message types and QoS.
- Publish optional marker results without leaking publisher checks into the node.
- Destroy every owned publisher deterministically and idempotently.

Does not own:
- Subscriptions, timers, TF, synchronization or image processing.
- Topic names and feature flags outside the validated node parameters.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    qos_profile_sensor_data,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import Float64MultiArray
from visualization_msgs.msg import MarkerArray

from manipulation_perception.common.stereo_node_parameters import StereoNodeParameters


def _reliable_qos(*, depth: int, transient_local: bool = False) -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=(
            DurabilityPolicy.TRANSIENT_LOCAL
            if transient_local
            else DurabilityPolicy.VOLATILE
        ),
    )


@dataclass
class StereoRosOutputs:
    """Own all publishers created by ``stereo_sgbm_pointcloud_node``."""

    disparity: Any
    depth: Any
    point_cloud: Any
    octomap_point_cloud: Any
    cluster_markers: Any | None
    table_plane_marker: Any | None
    table_plane_coeffs: Any

    @classmethod
    def create(cls, node: Any, parameters: StereoNodeParameters) -> "StereoRosOutputs":
        """
        Create the complete publisher set or roll back partial ownership.

        Sensor clouds use sensor-data QoS to bound latency. Fixed table output
        uses transient-local reliable QoS so late subscribers receive the last
        estimate. If any creation fails, already-created publishers are destroyed
        before the original exception is re-raised.
        """
        depth_qos = _reliable_qos(depth=5)
        marker_qos = _reliable_qos(depth=10)
        transient_marker_qos = _reliable_qos(depth=10, transient_local=True)
        plane_coeffs_qos = _reliable_qos(depth=1, transient_local=True)
        created_publishers: list[Any] = []

        def create_publisher(message_type: Any, topic: str, qos: Any) -> Any:
            publisher = node.create_publisher(message_type, topic, qos)
            created_publishers.append(publisher)
            return publisher

        try:
            disparity = create_publisher(
                Image,
                parameters.disparity_topic,
                qos_profile_sensor_data,
            )
            depth = create_publisher(Image, parameters.depth_topic, depth_qos)
            point_cloud = create_publisher(
                PointCloud2,
                parameters.point_cloud_topic,
                depth_qos,
            )
            octomap_point_cloud = create_publisher(
                PointCloud2,
                parameters.octomap_point_cloud_topic,
                qos_profile_sensor_data,
            )
            cluster_markers = None
            if parameters.publish_cluster_markers:
                cluster_markers = create_publisher(
                    MarkerArray,
                    parameters.cluster_marker_topic,
                    marker_qos,
                )
            table_plane_marker = None
            if parameters.publish_table_plane_marker:
                table_plane_marker = create_publisher(
                    MarkerArray,
                    parameters.table_plane_marker_topic,
                    transient_marker_qos,
                )
            table_plane_coeffs = create_publisher(
                Float64MultiArray,
                parameters.table_plane_coeffs_topic,
                plane_coeffs_qos,
            )
        except Exception as creation_error:
            for publisher in reversed(created_publishers):
                try:
                    node.destroy_publisher(publisher)
                except Exception as cleanup_error:
                    creation_error.add_note(
                        "Publisher rollback failed: "
                        f"{type(cleanup_error).__name__}: {cleanup_error}"
                    )
            raise

        return cls(
            disparity=disparity,
            depth=depth,
            point_cloud=point_cloud,
            octomap_point_cloud=octomap_point_cloud,
            cluster_markers=cluster_markers,
            table_plane_marker=table_plane_marker,
            table_plane_coeffs=table_plane_coeffs,
        )

    @property
    def has_table_plane_outputs(self) -> bool:
        """Return whether at least one table-plane publisher is still owned."""
        return self.table_plane_marker is not None or self.table_plane_coeffs is not None

    def publish_disparity(self, message: Image) -> None:
        """Publish a source-stamped pixel-disparity image on the public topic."""
        self.disparity.publish(message)

    def publish_depth(self, message: Image) -> None:
        """Publish a source-stamped metric 32FC1 depth image reliably."""
        self.depth.publish(message)

    def publish_cloud(self, point_cloud: PointCloud2, cluster_markers: MarkerArray | None) -> None:
        """Publish a public cloud and its optional same-stamp cluster markers."""
        self.point_cloud.publish(point_cloud)
        if cluster_markers is not None and self.cluster_markers is not None:
            self.cluster_markers.publish(cluster_markers)

    def publish_octomap_cloud(self, point_cloud: PointCloud2) -> None:
        """Publish the sensor-frame cloud consumed only by MoveIt Octomap."""
        self.octomap_point_cloud.publish(point_cloud)

    def publish_table_plane(
        self,
        marker: MarkerArray | None,
        coeffs: Float64MultiArray | None,
    ) -> None:
        """Publish whichever table outputs are enabled and present."""
        if marker is not None and self.table_plane_marker is not None:
            self.table_plane_marker.publish(marker)
        if coeffs is not None and self.table_plane_coeffs is not None:
            self.table_plane_coeffs.publish(coeffs)

    def destroy(self, node: Any) -> None:
        """Destroy every owned publisher; repeated calls are idempotent."""
        for attribute in (
            "cluster_markers",
            "table_plane_marker",
            "table_plane_coeffs",
            "octomap_point_cloud",
            "point_cloud",
            "depth",
            "disparity",
        ):
            publisher = getattr(self, attribute)
            if publisher is not None:
                node.destroy_publisher(publisher)
                setattr(self, attribute, None)
