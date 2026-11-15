#!/usr/bin/env python3
"""Responsibility: Add edge/corner YOLO segmentation augmentations to an existing cube dataset."""

from __future__ import annotations

import argparse
import random
from pathlib import Path



EDGE_TARGETS = (
    (0.04, 0.08),
    (0.50, 0.08),
    (0.96, 0.08),
    (0.08, 0.50),
    (0.92, 0.50),
    (0.04, 0.92),
    (0.50, 0.92),
    (0.96, 0.92),
    (0.20, 0.20),
    (0.80, 0.20),
    (0.20, 0.80),
    (0.80, 0.80),
)


def _parse_args() -> argparse.Namespace:
    """Read augmentation parameters."""
    parser = argparse.ArgumentParser(description="Create edge-heavy YOLO segmentation augmentations")
    parser.add_argument("--dataset_root", required=True, help="YOLO dataset root with images/ and labels/")
    parser.add_argument("--splits", default="train,val", help="Comma-separated dataset splits")
    parser.add_argument("--per_positive", type=int, default=1, help="Augmented variants per positive source image")
    parser.add_argument("--seed", type=int, default=142, help="Random seed")
    parser.add_argument("--prefix", default="edge_aug", help="Filename prefix for generated samples")
    parser.add_argument("--jitter", type=float, default=0.035, help="Target center jitter in normalized image units")
    parser.add_argument("--min_area", type=float, default=0.00008, help="Minimum clipped polygon area")
    parser.add_argument("--overwrite", action="store_true", help="Remove existing augmented samples with prefix first")
    return parser.parse_args()


def _clip_polygon(points: list[tuple[float, float]]) -> list[tuple[float, float]]:
    """Clip a normalized polygon to the image rectangle."""

    def clip_edge(
        polygon: list[tuple[float, float]],
        inside,
        intersect,
    ) -> list[tuple[float, float]]:
        if not polygon:
            return []
        output: list[tuple[float, float]] = []
        previous = polygon[-1]
        previous_inside = inside(previous)
        for current in polygon:
            current_inside = inside(current)
            if current_inside:
                if not previous_inside:
                    output.append(intersect(previous, current))
                output.append(current)
            elif previous_inside:
                output.append(intersect(previous, current))
            previous = current
            previous_inside = current_inside
        return output

    def intersect_x(x_value: float):
        def fn(a: tuple[float, float], b: tuple[float, float]) -> tuple[float, float]:
            ax, ay = a
            bx, by = b
            denom = bx - ax
            ratio = 0.0 if abs(denom) < 1.0e-9 else (x_value - ax) / denom
            return x_value, ay + ratio * (by - ay)

        return fn

    def intersect_y(y_value: float):
        def fn(a: tuple[float, float], b: tuple[float, float]) -> tuple[float, float]:
            ax, ay = a
            bx, by = b
            denom = by - ay
            ratio = 0.0 if abs(denom) < 1.0e-9 else (y_value - ay) / denom
            return ax + ratio * (bx - ax), y_value

        return fn

    clipped = clip_edge(points, lambda p: p[0] >= 0.0, intersect_x(0.0))
    clipped = clip_edge(clipped, lambda p: p[0] <= 1.0, intersect_x(1.0))
    clipped = clip_edge(clipped, lambda p: p[1] >= 0.0, intersect_y(0.0))
    clipped = clip_edge(clipped, lambda p: p[1] <= 1.0, intersect_y(1.0))
    return [(max(0.0, min(1.0, x)), max(0.0, min(1.0, y))) for x, y in clipped]


def _polygon_area(points: list[tuple[float, float]]) -> float:
    """Compute normalized polygon area."""
    if len(points) < 3:
        return 0.0
    area = 0.0
    for index, (x0, y0) in enumerate(points):
        x1, y1 = points[(index + 1) % len(points)]
        area += x0 * y1 - x1 * y0
    return abs(area) * 0.5


def _read_seg_label(label_path: Path) -> list[tuple[float, float]] | None:
    """Read the first YOLO segmentation polygon from a label file."""
    text = label_path.read_text(encoding="utf-8").strip()
    if not text:
        return None
    parts = text.split()
    if len(parts) < 7:
        return None
    coords = [float(value) for value in parts[1:]]
    points = list(zip(coords[0::2], coords[1::2], strict=True))
    return points if len(points) >= 3 else None


def _write_seg_label(label_path: Path, points: list[tuple[float, float]]) -> None:
    """Write a single-class YOLO segmentation label."""
    coords: list[str] = []
    for x_value, y_value in points:
        coords.append(f"{x_value:.6f}")
        coords.append(f"{y_value:.6f}")
    label_path.write_text(f"0 {' '.join(coords)}\n", encoding="utf-8")


def _augment_split(args: argparse.Namespace, split: str, rng: random.Random) -> tuple[int, int]:
    """Create edge/corner translations for one dataset split."""
    root = Path(args.dataset_root).expanduser().resolve()
    image_dir = root / "images" / split
    label_dir = root / "labels" / split
    if not image_dir.is_dir() or not label_dir.is_dir():
        raise FileNotFoundError(f"Split directories not found: {split}")

    if args.overwrite:
        for path in [*image_dir.glob(f"{args.prefix}_*.png"), *label_dir.glob(f"{args.prefix}_*.txt")]:
            path.unlink()

    source_labels = [
        path
        for path in sorted(label_dir.glob("*.txt"))
        if not path.name.startswith(f"{args.prefix}_")
    ]
    created = 0
    skipped = 0
    target_index = 0
    for label_path in source_labels:
        points = _read_seg_label(label_path)
        if points is None:
            continue
        image_path = image_dir / f"{label_path.stem}.png"
        image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
        if image is None:
            skipped += 1
            continue
        height, width = image.shape[:2]
        x_center = (min(x for x, _ in points) + max(x for x, _ in points)) * 0.5
        y_center = (min(y for _, y in points) + max(y for _, y in points)) * 0.5

        for variant_index in range(max(1, int(args.per_positive))):
            target_x, target_y = EDGE_TARGETS[target_index % len(EDGE_TARGETS)]
            target_index += 1
            jitter = max(0.0, float(args.jitter))
            target_x += rng.uniform(-jitter, jitter)
            target_y += rng.uniform(-jitter, jitter)
            dx_norm = target_x - x_center
            dy_norm = target_y - y_center
            dx_px = int(round(dx_norm * width))
            dy_px = int(round(dy_norm * height))
            shifted_points = [(x + dx_px / width, y + dy_px / height) for x, y in points]
            clipped_points = _clip_polygon(shifted_points)
            if _polygon_area(clipped_points) < float(args.min_area):
                skipped += 1
                continue

            matrix = np.float32([[1.0, 0.0, dx_px], [0.0, 1.0, dy_px]])
            shifted_image = cv2.warpAffine(
                image,
                matrix,
                (width, height),
                flags=cv2.INTER_LINEAR,
                borderMode=cv2.BORDER_REFLECT_101,
            )
            output_stem = f"{args.prefix}_{split}_{created:06d}_{variant_index}"
            cv2.imwrite(str(image_dir / f"{output_stem}.png"), shifted_image)
            _write_seg_label(label_dir / f"{output_stem}.txt", clipped_points)
            created += 1
    return created, skipped


def main() -> int:
    """Run edge augmentation."""
    args = _parse_args()
    # Keep CLI inspection independent of optional image-processing packages.
    global cv2, np
    import cv2
    import numpy as np
    rng = random.Random(args.seed)
    for split in [item.strip() for item in args.splits.split(",") if item.strip()]:
        created, skipped = _augment_split(args, split, rng)
        print(f"{split}: created={created}, skipped={skipped}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
