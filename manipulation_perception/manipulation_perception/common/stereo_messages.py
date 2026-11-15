"""
ROS message builders for stereo perception outputs.

Responsibility:
- Build PointCloud2, MarkerArray and table-plane coeff messages.
- Preserve the stamped point-cloud and visualization payload contracts.

Does not own:
- ROS publishers/subscribers.
- Stereo/depth/TF algorithms.
- Table-plane detection or cluster extraction.
"""

from __future__ import annotations

import struct

import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Float64MultiArray, Header
from visualization_msgs.msg import Marker, MarkerArray

from manipulation_perception.common.stereo_config import ClusterMarkerConfig, TablePlaneConfig


class PointCloud2Builder:
    """
    Build XYZRGB PointCloud2 messages without owning a ROS publisher.

    Input XYZ values are metres in ``frame_id``. The builder preserves the
    supplied ROS sample stamp and uses a fixed packed RGB value for every point.
    """

    def __init__(self, rgb_uint: int = (255 << 16) | (0 << 8) | 0) -> None:
        self._rgb_uint = np.uint32(rgb_uint)
        self._fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="rgb", offset=12, datatype=PointField.FLOAT32, count=1),
        ]

    def build(self, points: np.ndarray, stamp, frame_id: str) -> PointCloud2:
        """Encode an ``N x 3`` metric array with the supplied stamped header."""
        header = Header()
        header.stamp = stamp
        header.frame_id = frame_id

        if points.shape[0] == 0:
            return point_cloud2.create_cloud(header, self._fields, [])

        rgb_uint = np.full(points.shape[0], self._rgb_uint, dtype=np.uint32)
        rgb_float = np.array(
            [struct.unpack("<f", struct.pack("<I", int(value)))[0] for value in rgb_uint],
            dtype=np.float32,
        )
        cloud_points = np.empty((points.shape[0], 4), dtype=np.float32)
        cloud_points[:, 0:3] = points
        cloud_points[:, 3] = rgb_float
        return point_cloud2.create_cloud(header, self._fields, cloud_points.tolist())


class TablePlaneMarkerBuilder:
    """Build a stable RViz tabletop marker in the coefficient's frame."""

    def __init__(self, config: TablePlaneConfig) -> None:
        self._config = config

    @staticmethod
    def clear(stamp, frame_id: str) -> MarkerArray:
        """Build a stamped DELETE command for the canonical table marker ID."""
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = frame_id
        marker.ns = "table_plane"
        marker.id = 0
        marker.action = Marker.DELETE
        marker_array = MarkerArray()
        marker_array.markers.append(marker)
        return marker_array

    def build(
        self,
        bounds: tuple[float, float, float, float, float],
        stamp,
        frame_id: str,
    ) -> MarkerArray:
        """Build the axis-aligned metric bounds marker for one table estimate."""
        x_min, x_max, y_min, y_max, plane_z = bounds
        center_x = 0.5 * (x_min + x_max)
        center_y = 0.5 * (y_min + y_max)
        extent_x = max(0.02, x_max - x_min)
        extent_y = max(0.02, y_max - y_min)

        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = frame_id
        marker.ns = "table_plane"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = center_x
        marker.pose.position.y = center_y
        marker.pose.position.z = plane_z
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = extent_x
        marker.scale.y = extent_y
        marker.scale.z = float(self._config.marker_thickness_m)
        marker.color.r = 0.1
        marker.color.g = 0.7
        marker.color.b = 1.0
        marker.color.a = self._config.marker_alpha
        marker_array = MarkerArray()
        marker_array.markers.append(marker)
        return marker_array


class ClusterMarkerBuilder:
    """Build metric bounding-box markers for size-admitted point clusters."""

    def __init__(self, config: ClusterMarkerConfig) -> None:
        self._config = config

    def build(self, points: np.ndarray, clusters: list[np.ndarray], stamp, frame_id: str) -> MarkerArray:
        """Return DELETEALL followed by boxes sharing the cloud header."""
        marker_array = MarkerArray()
        clear_marker = Marker()
        clear_marker.header.stamp = stamp
        clear_marker.header.frame_id = frame_id
        clear_marker.action = Marker.DELETEALL
        marker_array.markers.append(clear_marker)

        marker_id = 0
        for cluster_indices in clusters:
            cluster_points = points[cluster_indices]
            min_xyz = np.min(cluster_points, axis=0)
            max_xyz = np.max(cluster_points, axis=0)
            extents = max_xyz - min_xyz
            cluster_diagonal = float(np.linalg.norm(extents))
            if (
                cluster_diagonal < self._config.min_diagonal_m
                or cluster_diagonal > self._config.max_diagonal_m
            ):
                continue

            center = 0.5 * (min_xyz + max_xyz)
            marker = Marker()
            marker.header.stamp = stamp
            marker.header.frame_id = frame_id
            marker.ns = "tabletop_clusters"
            marker.id = marker_id
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = float(center[0])
            marker.pose.position.y = float(center[1])
            marker.pose.position.z = float(center[2])
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = float(max(extents[0], self._config.marker_min_edge_m))
            marker.scale.y = float(max(extents[1], self._config.marker_min_edge_m))
            marker.scale.z = float(max(extents[2], self._config.marker_min_edge_m))
            marker.color.r = 1.0
            marker.color.g = 0.85
            marker.color.b = 0.0
            marker.color.a = self._config.marker_alpha
            marker_array.markers.append(marker)
            marker_id += 1

        return marker_array


def table_plane_coeffs_message(coeffs: tuple[float, float, float, float]) -> Float64MultiArray:
    """Encode normalized plane coefficients ``[a, b, c, d]`` in metric form."""
    msg = Float64MultiArray()
    msg.data = [float(value) for value in coeffs]
    return msg
