"""
State holder for the YOLO OpenCV runtime control panel.

Responsibility:
- Track runtime control layout, image shape and mouse drag state.
- Apply mouse events to control slider values.
- Render the controls panel below a BGR frame.

Does not own:
- OpenCV window creation/destruction, imshow or waitKey.
- Runtime control normalization into detector settings.
- YOLO/SAHI inference, ROS callbacks or message publication.
"""

from __future__ import annotations

import cv2
import numpy as np

from manipulation_perception.yolo.yolo_debug_rendering import render_controls_panel
from manipulation_perception.yolo.yolo_runtime_controls import (
    build_controls_layout,
    set_control_value_from_x,
)


class YoloDebugControlPanel:
    """
    Own mutable mouse/slider state for one OpenCV window.

    ``on_mouse`` and ``render`` must run on the same GUI thread; this class does
    not lock shared controls. Rendering appends a panel below the source image
    and never mutates the BGR inference frame itself.
    """

    def __init__(self, controls: list[dict]) -> None:
        self.controls = controls
        self.layout: list[dict] = []
        self.panel_height = 0
        self.current_image_shape: tuple[int, int] = (0, 0)
        self.drag_control_index: int | None = None

    def on_mouse(self, event, x, y, _flags=None, _userdata=None) -> None:
        """Apply one OpenCV mouse event to the active slider drag."""
        image_h, _image_w = self.current_image_shape
        if image_h <= 0:
            return

        panel_y = y - image_h
        if panel_y < 0:
            if event == cv2.EVENT_LBUTTONUP:
                self.drag_control_index = None
            return

        if event == cv2.EVENT_LBUTTONDOWN:
            for row in self.layout:
                hit_y = row["hit_y0"] <= panel_y <= row["hit_y1"]
                hit_x = row["track_x0"] <= x <= row["track_x1"]
                if hit_y and hit_x:
                    self.drag_control_index = int(row["index"])
                    self._set_value_from_x(self.drag_control_index, x)
                    break
        elif event == cv2.EVENT_MOUSEMOVE and self.drag_control_index is not None:
            self._set_value_from_x(self.drag_control_index, x)
        elif event == cv2.EVENT_LBUTTONUP:
            self.drag_control_index = None

    def render(self, frame_bgr: np.ndarray) -> np.ndarray:
        """Render controls below a BGR frame and refresh mouse hit regions."""
        if not self.controls:
            return frame_bgr

        frame_h, frame_w = frame_bgr.shape[:2]
        self.layout, self.panel_height = build_controls_layout(self.controls, frame_w)
        self.current_image_shape = (frame_h, frame_w)
        return render_controls_panel(
            frame_bgr,
            self.controls,
            self.layout,
            self.panel_height,
        )

    def _set_value_from_x(self, control_index: int, x: int) -> None:
        set_control_value_from_x(
            self.controls,
            self.layout,
            control_index,
            x,
        )
