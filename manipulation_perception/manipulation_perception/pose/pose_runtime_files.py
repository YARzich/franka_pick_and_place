"""
Runtime file writers for MegaPose and GigaPose pose-estimation backends.

Responsibility:
- Write MegaPose example inputs and GigaPose runtime image/mask files.
- Prepare the mesh copy expected by MegaPose examples.
- Build camera/object JSON payloads in a testable form.

Does not own:
- ROS messages, callbacks or publishers.
- Subprocess execution and worker lifecycle.
- Backend model selection or JSON request payloads.
"""

from __future__ import annotations

import json
from pathlib import Path
import shutil
import tempfile
import threading
from typing import Sequence

import cv2
import numpy as np

from manipulation_perception.common.depth_geometry import depth_to_mm_u16
from manipulation_perception.pose.mesh_utils import (
    copy_material_assets,
    count_obj_vertices,
    mesh_unit_scale_to_mm,
    write_dense_scaled_obj,
    write_scaled_obj,
)
from manipulation_perception.pose.pose_worker_requests import camera_k_matrix


class PoseRuntimeWorkspace:
    """
    Own one private data-root view for the lifetime of a pose node's workers.

    Only immutable model/dataset trees are linked. All examples, logs, derived
    refiner meshes and caches are private. Cleanup is explicit: the owner must
    first stop processes and join every writer; an unfinished writer retains
    its directory rather than losing files through a temporary-object finalizer.
    """

    def __init__(self, megapose_assets: Path, gigapose_assets: Path) -> None:
        self.root = Path(tempfile.mkdtemp(prefix="manipulation-pose-"))
        self.megapose_data_dir = self.root / "megapose"
        self.gigapose_data_dir = self.root / "gigapose"
        self._gigapose_assets = gigapose_assets
        self._cache_lock = threading.Lock()
        self._cache_prepared = False
        self._closed = False
        try:
            self.megapose_data_dir.mkdir()
            self.gigapose_data_dir.mkdir()
            self._link_assets(megapose_assets, self.megapose_data_dir, ("megapose-models",))
            self._link_assets(gigapose_assets, self.gigapose_data_dir, ("pretrained",))
            self._link_gigapose_datasets(gigapose_assets / "datasets")
        except Exception:
            self.cleanup()
            raise

    @staticmethod
    def _link_assets(source: Path, destination: Path, names: tuple[str, ...]) -> None:
        for name in names:
            asset = source / name
            if asset.exists():
                (destination / name).symlink_to(asset.resolve(), target_is_directory=asset.is_dir())

    def _link_gigapose_datasets(self, source: Path) -> None:
        """Keep template preprocessing private while sharing read-only input assets."""
        if not source.is_dir():
            return
        destination = self.gigapose_data_dir / "datasets"
        destination.mkdir()
        self._link_assets(
            source, destination, tuple(item.name for item in source.iterdir() if item.name != "templates"),
        )
        templates = source / "templates"
        if not templates.is_dir():
            return
        template_view = destination / "templates"
        template_view.mkdir()
        for dataset in templates.iterdir():
            if not dataset.is_dir():
                self._link_assets(templates, template_view, (dataset.name,))
                continue
            dataset_view = template_view / dataset.name
            dataset_view.mkdir()
            # TemplateData creates this cache even in read_test_mode. Linking
            # the whole dataset would write back through the shared asset tree.
            (dataset_view / "preprocessed").mkdir()
            self._link_assets(
                dataset, dataset_view,
                tuple(item.name for item in dataset.iterdir() if item.name != "preprocessed"),
            )

    def prepare_gigapose_cache(self) -> None:
        """Lazily seed a private cache without writing through asset symlinks."""
        with self._cache_lock:
            if self._closed:
                raise RuntimeError("Pose runtime workspace is already closed")
            if self._cache_prepared:
                return
            source = self._gigapose_assets / "torch"
            destination = self.gigapose_data_dir / "torch"
            if source.is_dir():
                # Existing weights avoid a new download. Copy instead of linking:
                # torch may update cache files in place, unlike immutable datasets.
                shutil.copytree(source, destination, dirs_exist_ok=True, symlinks=False)
            else:
                destination.mkdir()
            self._cache_prepared = True

    def cleanup(self) -> None:
        """Remove only this instance's workspace after all writers have joined."""
        with self._cache_lock:
            if not self._closed:
                shutil.rmtree(self.root)
                self._closed = True


def _write_image(path: Path, image: np.ndarray) -> None:
    """Replace one runtime image and reject OpenCV write failures fail-closed."""
    path.unlink(missing_ok=True)
    if not cv2.imwrite(str(path), image):
        path.unlink(missing_ok=True)
        raise OSError(f"failed to write runtime image: {path}")


def megapose_camera_data(camera_k: Sequence[float], image_shape: tuple[int, int]) -> dict:
    """Build MegaPose camera JSON with pixel intrinsics and ``[height, width]``."""
    return {
        "K": camera_k_matrix(camera_k),
        "resolution": [int(image_shape[0]), int(image_shape[1])],
    }


def megapose_object_data(object_label: str, bbox: tuple[int, int, int, int]) -> list[dict]:
    """Build one labelled half-open pixel bbox for MegaPose input JSON."""
    x1, y1, x2, y2 = bbox
    return [
        {
            "label": object_label,
            "bbox_modal": [int(x1), int(y1), int(x2), int(y2)],
        }
    ]


def megapose_output_pose_paths(example_dir: Path) -> list[Path]:
    """Return supported output filenames in deterministic preference order."""
    return [
        example_dir / "outputs" / "object_data.json",
        example_dir / "outputs" / "object_data_inf.json",
    ]


class MegaPoseRuntimeExample:
    """
    Own and overwrite one filesystem example consumed by MegaPose.

    Calls are serialized by the backend runner. RGB, optional metric depth,
    pixel intrinsics and bbox files are replaced for every inference; mesh
    conversion into MegaPose millimetres is performed once per instance. Stale
    pose outputs are removed before invoking the backend.
    """

    def __init__(
        self,
        example_dir: Path,
        object_label: str,
        mesh_file_path: Path,
        mesh_units: str,
    ) -> None:
        self.example_dir = example_dir
        self.object_label = object_label
        self.mesh_file_path = mesh_file_path
        self.mesh_units = mesh_units
        self._mesh_prepared = False

    @property
    def output_pose_paths(self) -> list[Path]:
        """Return candidate result files owned by this runtime example."""
        return megapose_output_pose_paths(self.example_dir)

    def prepare(
        self,
        rgb: np.ndarray,
        depth: np.ndarray | None,
        use_depth: bool,
        camera_k: Sequence[float],
        bbox: tuple[int, int, int, int],
    ) -> None:
        """
        Atomically prepare the logical input set for the next inference.

        Depth is converted from metres to unsigned millimetres when enabled.
        The function performs filesystem side effects and is not thread-safe.
        """
        self.example_dir.mkdir(parents=True, exist_ok=True)
        (self.example_dir / "inputs").mkdir(exist_ok=True)
        (self.example_dir / "outputs").mkdir(exist_ok=True)
        self._prepare_mesh_once()
        for stale_output in self.output_pose_paths:
            stale_output.unlink(missing_ok=True)

        image_bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        _write_image(self.example_dir / "image_rgb.png", image_bgr)

        depth_path = self.example_dir / "image_depth.png"
        if use_depth and depth is not None:
            depth_mm = depth_to_mm_u16(depth, rgb.shape[:2])
            _write_image(depth_path, depth_mm)
        else:
            depth_path.unlink(missing_ok=True)

        (self.example_dir / "camera_data.json").write_text(
            json.dumps(megapose_camera_data(camera_k, rgb.shape[:2]))
        )
        (self.example_dir / "inputs" / "object_data.json").write_text(
            json.dumps(megapose_object_data(self.object_label, bbox))
        )

    def _prepare_mesh_once(self) -> None:
        if self._mesh_prepared:
            return

        mesh_dir = self.example_dir / "meshes" / self.object_label
        mesh_dir.mkdir(parents=True, exist_ok=True)
        target_mesh_path = mesh_dir / self.mesh_file_path.name

        if self.mesh_file_path.suffix.lower() == ".obj":
            scale = mesh_unit_scale_to_mm(self.mesh_units)
            if count_obj_vertices(self.mesh_file_path) < 2000:
                write_dense_scaled_obj(
                    self.mesh_file_path,
                    target_mesh_path,
                    scale=scale,
                    fallback="scaled",
                )
            else:
                write_scaled_obj(self.mesh_file_path, target_mesh_path, scale=scale)
            copy_material_assets(self.mesh_file_path, mesh_dir)
        else:
            shutil.copy2(self.mesh_file_path, target_mesh_path)

        self._mesh_prepared = True


class GigaPoseRuntimeInput:
    """
    Own replace-in-place RGB and optional binary-mask worker files.

    A runner serializes calls; returned paths are valid until the next
    ``prepare`` invocation overwrites the same files.
    """

    def __init__(self, runtime_dir: Path) -> None:
        self.runtime_dir = runtime_dir

    def prepare(self, rgb: np.ndarray, mask: np.ndarray | None) -> tuple[Path, Path | None]:
        """Write RGB/mask inputs and return paths for one worker request."""
        self.runtime_dir.mkdir(parents=True, exist_ok=True)
        image_path = self.runtime_dir / "image_rgb.png"
        _write_image(image_path, cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))

        mask_path = self.runtime_dir / "image_mask.png"
        if mask is None:
            mask_path.unlink(missing_ok=True)
            return image_path, None

        mask_image = np.asarray(mask, dtype=np.uint8)
        if mask_image.ndim > 2:
            mask_image = np.squeeze(mask_image)
        _write_image(mask_path, np.where(mask_image > 0, 255, 0).astype(np.uint8))
        return image_path, mask_path
