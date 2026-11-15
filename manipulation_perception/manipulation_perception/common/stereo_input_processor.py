"""
Decode selected stereo inputs and hand depth ownership to its coordinator.

Responsibility:
    Convert synchronized local-stereo or external-disparity messages into one
    validated metric depth sample and invoke the depth coordinator.

Does not own:
    ROS subscriptions, TF policy, or cloud publication.
"""

from __future__ import annotations

from collections.abc import Callable
from typing import Any

import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from stereo_msgs.msg import DisparityImage

from manipulation_perception.common.depth_geometry import disparity_to_depth
from manipulation_perception.common.stereo_depth_pipeline import SgbmDepthEstimator
from manipulation_perception.common.stereo_disparity import (
    disparity_image_to_array,
    resolve_external_disparity_geometry,
)
from manipulation_perception.common.stereo_sample_geometry import (
    resolve_local_stereo_geometry,
)


class StereoInputProcessor:
    """
    Convert local/external disparity into depth without owning ROS transport.

    A successful handoff transfers completion timing to `StereoDepthCoordinator`,
    including any bounded wait for a matching detection. Pre-handoff failures
    complete their metric sample here.
    """

    def __init__(
        self,
        *,
        parameters: Any,
        bridge: Any,
        outputs: Any,
        depth_coordinator: Any,
        logger: Any,
        record_processed: Callable[[float], None],
        record_error: Callable[[], None],
    ) -> None:
        self._bridge = bridge
        self._outputs = outputs
        self._depth_coordinator = depth_coordinator
        self._logger = logger
        self._record_processed = record_processed
        self._record_error = record_error
        self._left_image_frame_id = getattr(parameters, "left_image_frame_id", "")
        self._right_image_frame_id = getattr(parameters, "right_image_frame_id", "")
        self._local_frame_id = parameters.local_stereo_frame_id
        self._baseline_m = parameters.baseline_m
        self._min_valid_disparity_px = parameters.min_valid_disparity_px
        self._min_depth_m = parameters.min_depth_m
        self._max_depth_m = parameters.max_depth_m
        self._sgbm_estimator = None
        if not parameters.use_external_disparity:
            self._sgbm_estimator = SgbmDepthEstimator(
                parameters.stereo_config,
                parameters.ecc_config,
            )

    def process_stereo_pair(
        self,
        left_message: Image,
        right_message: Image,
        camera_info: Any,
        *,
        frame_index: int,
        started_at: float,
    ) -> None:
        """Decode one exactly synchronized local image/calibration sample."""
        handed_off = False
        try:
            if self._sgbm_estimator is None:
                raise RuntimeError("local stereo callback has no SGBM estimator")
            geometry = resolve_local_stereo_geometry(
                left_message,
                right_message,
                camera_info,
                expected_left_frame=self._left_image_frame_id,
                expected_right_frame=self._right_image_frame_id,
            )
            left_gray = self._bridge.imgmsg_to_cv2(left_message, desired_encoding="mono8")
            right_gray = self._bridge.imgmsg_to_cv2(right_message, desired_encoding="mono8")
            disparity, ecc_error = self._sgbm_estimator.compute_disparity(
                left_gray,
                right_gray,
                frame_index,
                self._min_valid_disparity_px,
            )
            if ecc_error is not None:
                self._record_error()
                if self._sgbm_estimator.ecc_fail_count % 30 == 1:
                    self._logger.warn(
                        f"ECC не сошёлся, использую прошлую матрицу. Детали: {ecc_error}"
                    )

            output_header = Header()
            output_header.stamp = left_message.header.stamp
            output_header.frame_id = self._local_frame_id
            disparity_message = self._bridge.cv2_to_imgmsg(disparity, encoding="32FC1")
            disparity_message.header = output_header
            self._outputs.publish_disparity(disparity_message)
            depth = self._depth_from_disparity(
                disparity,
                geometry.intrinsics[0],
                self._baseline_m,
            )
            handed_off = True
            self._depth_coordinator.process_depth(
                depth,
                output_header,
                started_at=started_at,
                intrinsics=geometry.intrinsics,
            )
        except Exception as error:  # noqa: BLE001 - image/backend boundary
            if not handed_off:
                self._record_error()
            self._logger.error(f"Ошибка StereoSGBM pipeline: {error}")
        finally:
            if not handed_off:
                self._record_processed(started_at)

    def process_external_disparity(
        self,
        message: DisparityImage,
        camera_info: Any,
        *,
        started_at: float,
    ) -> None:
        """Decode one disparity sample with its exact stamped CameraInfo."""
        handed_off = False
        try:
            geometry = resolve_external_disparity_geometry(
                message,
                camera_info,
                self._baseline_m,
            )
            disparity = disparity_image_to_array(message.image)
            disparity[disparity <= self._min_valid_disparity_px] = np.nan
            depth = self._depth_from_disparity(
                disparity,
                geometry.intrinsics[0],
                geometry.baseline_m,
            )
            handed_off = True
            self._depth_coordinator.process_depth(
                depth,
                message.header,
                started_at=started_at,
                intrinsics=geometry.intrinsics,
            )
        except Exception as error:  # noqa: BLE001 - message/backend boundary
            if not handed_off:
                self._record_error()
            self._logger.error(f"Ошибка external disparity pipeline: {error}")
        finally:
            if not handed_off:
                self._record_processed(started_at)

    def _depth_from_disparity(
        self,
        disparity: np.ndarray,
        focal_length_x: float | None,
        baseline_m: float,
    ) -> np.ndarray:
        return disparity_to_depth(
            disparity,
            focal_length_x,
            baseline_m,
            self._min_valid_disparity_px,
            self._min_depth_m,
            self._max_depth_m,
        )


__all__ = ["StereoInputProcessor"]
