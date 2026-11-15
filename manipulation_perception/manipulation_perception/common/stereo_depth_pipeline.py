"""
OpenCV stereo depth pipeline helpers.

Responsibility:
- Own OpenCV SGBM and ECC alignment state.
- Return disparity arrays and ECC diagnostics without touching ROS graph.

Does not own:
- ROS Image conversion, publishers, subscriptions or TF lookup.
- Depth projection, workspace filtering or marker publication.
"""

from __future__ import annotations

import cv2
import numpy as np

from manipulation_perception.common.stereo_config import EccConfig, StereoSgbmConfig


class EccAligner:
    """
    Own the reusable right-to-left ECC warp for a stereo stream.

    The instance is stateful and intended for one serialized image callback
    path. Failed ECC updates retain the last valid warp, so callers receive an
    aligned image plus a diagnostic error instead of losing the whole sample.
    """

    def __init__(self, config: EccConfig) -> None:
        self._config = config
        self._warp_matrix = np.eye(2, 3, dtype=np.float32)
        self.fail_count = 0

    def align(
        self,
        left_gray: np.ndarray,
        right_gray: np.ndarray,
        frame_index: int,
    ) -> tuple[np.ndarray, str | None]:
        """Align the right image to the left, periodically updating the warp."""
        if not self._config.enabled:
            return right_gray, None

        error_message = None
        if frame_index % self._config.update_every_n_frames == 0:
            try:
                criteria = (
                    cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT,
                    self._config.max_iterations,
                    self._config.eps,
                )
                left_float = left_gray.astype(np.float32) / 255.0
                right_float = right_gray.astype(np.float32) / 255.0
                cv2.findTransformECC(
                    left_float,
                    right_float,
                    self._warp_matrix,
                    cv2.MOTION_EUCLIDEAN,
                    criteria,
                    None,
                    self._config.gauss_filter_size,
                )
                self.fail_count = 0
            except cv2.error as exc:
                self.fail_count += 1
                error_message = str(exc)

        aligned = cv2.warpAffine(
            right_gray,
            self._warp_matrix,
            (right_gray.shape[1], right_gray.shape[0]),
            flags=cv2.INTER_LINEAR + cv2.WARP_INVERSE_MAP,
            borderMode=cv2.BORDER_CONSTANT,
            borderValue=0,
        )
        return aligned, error_message


class SgbmDepthEstimator:
    """
    Own OpenCV SGBM/ECC state and compute left-referenced disparity.

    One instance must not be called concurrently because both OpenCV matchers
    and the ECC warp are mutable. Output disparity is in pixels; invalid or
    sub-threshold samples are represented by ``NaN``.
    """

    def __init__(self, stereo_config: StereoSgbmConfig, ecc_config: EccConfig) -> None:
        self._ecc_aligner = EccAligner(ecc_config)
        self._stereo = cv2.StereoSGBM_create(
            minDisparity=stereo_config.min_disparity,
            numDisparities=stereo_config.num_disparities,
            blockSize=stereo_config.block_size,
            P1=stereo_config.p1,
            P2=stereo_config.p2,
            disp12MaxDiff=stereo_config.disp12_max_diff,
            preFilterCap=stereo_config.pre_filter_cap,
            uniquenessRatio=stereo_config.uniqueness_ratio,
            speckleWindowSize=stereo_config.speckle_window_size,
            speckleRange=stereo_config.speckle_range,
            mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY,
        )

    @property
    def ecc_fail_count(self) -> int:
        """Return consecutive ECC update failures since the last success."""
        return self._ecc_aligner.fail_count

    def compute_disparity(
        self,
        left_gray: np.ndarray,
        right_gray: np.ndarray,
        frame_index: int,
        min_valid_disparity_px: float,
    ) -> tuple[np.ndarray, str | None]:
        """Return pixel disparity and an optional ECC diagnostic for one pair."""
        height = min(left_gray.shape[0], right_gray.shape[0])
        width = min(left_gray.shape[1], right_gray.shape[1])
        left_cropped = left_gray[:height, :width]
        right_cropped = right_gray[:height, :width]
        right_aligned, ecc_error = self._ecc_aligner.align(left_cropped, right_cropped, frame_index)

        disparity = self._stereo.compute(left_cropped, right_aligned).astype(np.float32) / 16.0
        disparity[disparity <= float(min_valid_disparity_px)] = np.nan
        return disparity, ecc_error
