#!/usr/bin/env python3
"""Responsibility: Create hard YOLO segmentation augmentations with shadows and camera-like warps."""

from __future__ import annotations

import argparse
import json
import random
import shutil
from pathlib import Path



IMAGE_SUFFIXES = (".png", ".jpg", ".jpeg")
EDGE_TARGETS = (
    (0.02, 0.04),
    (0.04, 0.08),
    (0.50, 0.03),
    (0.50, 0.06),
    (0.98, 0.04),
    (0.96, 0.08),
    (0.06, 0.50),
    (0.94, 0.50),
    (0.02, 0.96),
    (0.04, 0.92),
    (0.50, 0.97),
    (0.50, 0.94),
    (0.98, 0.96),
    (0.96, 0.92),
    (0.16, 0.22),
    (0.84, 0.22),
    (0.16, 0.78),
    (0.84, 0.78),
)


def _parse_args() -> argparse.Namespace:
    """Read CLI arguments."""
    parser = argparse.ArgumentParser(description="Create edge/near/far/shadow hard augmentations")
    parser.add_argument("--source_dataset", required=True, help="Source YOLO segmentation dataset root")
    parser.add_argument("--output_dir", required=True, help="Output YOLO dataset root")
    parser.add_argument("--splits", default="train", help="Comma-separated splits to augment")
    parser.add_argument("--variants_per_image", type=int, default=8, help="Augmented variants per image")
    parser.add_argument("--include_original", action="store_true", help="Copy originals too")
    parser.add_argument("--overwrite", action="store_true", help="Remove output directory first")
    parser.add_argument("--seed", type=int, default=5404, help="Random seed")
    parser.add_argument("--prefix", default="hard_aug", help="Output filename prefix")
    parser.add_argument("--shadow_probability", type=float, default=0.85, help="Probability of adding synthetic shadows")
    parser.add_argument("--warp_probability", type=float, default=0.70, help="Probability of perspective/radial warp")
    parser.add_argument(
        "--affine_probability",
        type=float,
        default=1.0,
        help="Probability of moving/scaling/rotating the cube before photometric augmentation",
    )
    parser.add_argument("--max_points", type=int, default=48, help="Maximum output polygon points")
    return parser.parse_args()


def _image_paths(image_dir: Path) -> list[Path]:
    """Collect image files in stable order."""
    paths: list[Path] = []
    for suffix in IMAGE_SUFFIXES:
        paths.extend(image_dir.glob(f"*{suffix}"))
    return sorted(paths)


def _read_polygons(label_path: Path) -> list[list[tuple[float, float]]]:
    """Read YOLO segmentation polygons."""
    if not label_path.is_file() or label_path.stat().st_size == 0:
        return []
    polygons: list[list[tuple[float, float]]] = []
    for line in label_path.read_text(encoding="utf-8").splitlines():
        parts = line.strip().split()
        if len(parts) < 7:
            continue
        values = [float(value) for value in parts[1:]]
        points = list(zip(values[0::2], values[1::2], strict=True))
        if len(points) >= 3:
            polygons.append(points)
    return polygons


def _polygons_to_mask(polygons: list[list[tuple[float, float]]], width: int, height: int) -> np.ndarray:
    """Rasterize normalized polygons."""
    mask = np.zeros((height, width), dtype=np.uint8)
    for polygon in polygons:
        pts = np.array(
            [[int(round(x_value * width)), int(round(y_value * height))] for x_value, y_value in polygon],
            dtype=np.int32,
        )
        cv2.fillPoly(mask, [pts], 255)
    return mask


def _mask_to_polygons(mask: np.ndarray, max_points: int) -> list[list[tuple[float, float]]]:
    """Convert a transformed mask back to normalized YOLO polygons."""
    height, width = mask.shape[:2]
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    polygons: list[list[tuple[float, float]]] = []
    for contour in sorted(contours, key=cv2.contourArea, reverse=True):
        area = cv2.contourArea(contour)
        if area < max(8.0, 0.00004 * width * height):
            continue
        epsilon = max(1.0, 0.006 * cv2.arcLength(contour, True))
        approx = cv2.approxPolyDP(contour, epsilon, True).reshape(-1, 2)
        if len(approx) < 3:
            continue
        if len(approx) > max_points:
            indices = np.linspace(0, len(approx) - 1, max_points, dtype=int)
            approx = approx[indices]
        polygon: list[tuple[float, float]] = []
        for x_value, y_value in approx:
            polygon.append((float(np.clip(x_value / width, 0.0, 1.0)), float(np.clip(y_value / height, 0.0, 1.0))))
        polygons.append(polygon)
        break
    return polygons


def _write_label(path: Path, polygons: list[list[tuple[float, float]]]) -> None:
    """Write YOLO segmentation labels."""
    lines: list[str] = []
    for polygon in polygons:
        coords: list[str] = []
        for x_value, y_value in polygon:
            coords.append(f"{x_value:.6f}")
            coords.append(f"{y_value:.6f}")
        lines.append(f"0 {' '.join(coords)}")
    path.write_text(("\n".join(lines) + "\n") if lines else "", encoding="utf-8")


def _mask_center(mask: np.ndarray) -> tuple[float, float]:
    """Return normalized center of a binary mask."""
    height, width = mask.shape[:2]
    moments = cv2.moments(mask, binaryImage=True)
    if abs(moments["m00"]) < 1.0e-6:
        return 0.5, 0.5
    return float(moments["m10"] / moments["m00"] / width), float(moments["m01"] / moments["m00"] / height)


def _apply_affine(image: np.ndarray, mask: np.ndarray, rng: random.Random, variant_index: int) -> tuple[np.ndarray, np.ndarray]:
    """Scale, rotate and translate the frame so the cube lands in hard image regions."""
    height, width = image.shape[:2]
    center_x, center_y = _mask_center(mask)
    source_center = np.array([center_x * width, center_y * height], dtype=np.float32)
    target_x, target_y = EDGE_TARGETS[variant_index % len(EDGE_TARGETS)]
    target_x += rng.uniform(-0.035, 0.035)
    target_y += rng.uniform(-0.035, 0.035)
    target = np.array([target_x * width, target_y * height], dtype=np.float32)

    scale_roll = rng.random()
    if scale_roll < 0.35:
        scale = rng.uniform(0.48, 0.78)  # far/small
    elif scale_roll < 0.70:
        scale = rng.uniform(1.15, 1.62)  # close/large
    else:
        scale = rng.uniform(0.85, 1.18)
    angle = rng.uniform(-10.0, 10.0)
    matrix = cv2.getRotationMatrix2D(tuple(source_center), angle, scale)
    matrix[:, 2] += target - source_center

    warped_image = cv2.warpAffine(
        image,
        matrix,
        (width, height),
        flags=cv2.INTER_LINEAR,
        borderMode=cv2.BORDER_REFLECT_101,
    )
    warped_mask = cv2.warpAffine(
        mask,
        matrix,
        (width, height),
        flags=cv2.INTER_NEAREST,
        borderMode=cv2.BORDER_CONSTANT,
        borderValue=0,
    )
    return warped_image, warped_mask


def _apply_perspective(image: np.ndarray, mask: np.ndarray, rng: random.Random) -> tuple[np.ndarray, np.ndarray]:
    """Apply a mild perspective warp to emulate edge-of-lens geometry."""
    height, width = image.shape[:2]
    max_dx = rng.uniform(0.018, 0.055) * width
    max_dy = rng.uniform(0.016, 0.050) * height
    src = np.float32([[0, 0], [width - 1, 0], [width - 1, height - 1], [0, height - 1]])
    dst = src + np.float32(
        [
            [rng.uniform(-max_dx, max_dx), rng.uniform(-max_dy, max_dy)],
            [rng.uniform(-max_dx, max_dx), rng.uniform(-max_dy, max_dy)],
            [rng.uniform(-max_dx, max_dx), rng.uniform(-max_dy, max_dy)],
            [rng.uniform(-max_dx, max_dx), rng.uniform(-max_dy, max_dy)],
        ]
    )
    matrix = cv2.getPerspectiveTransform(src, dst)
    warped_image = cv2.warpPerspective(
        image,
        matrix,
        (width, height),
        flags=cv2.INTER_LINEAR,
        borderMode=cv2.BORDER_REFLECT_101,
    )
    warped_mask = cv2.warpPerspective(
        mask,
        matrix,
        (width, height),
        flags=cv2.INTER_NEAREST,
        borderMode=cv2.BORDER_CONSTANT,
        borderValue=0,
    )
    return warped_image, warped_mask


def _apply_radial(image: np.ndarray, mask: np.ndarray, rng: random.Random) -> tuple[np.ndarray, np.ndarray]:
    """Apply barrel/pincushion-like remapping around a random optical center."""
    height, width = image.shape[:2]
    center_x = rng.uniform(0.42, 0.58) * width
    center_y = rng.uniform(0.42, 0.58) * height
    yy, xx = np.mgrid[0:height, 0:width].astype(np.float32)
    x_norm = (xx - center_x) / width
    y_norm = (yy - center_y) / height
    radius2 = x_norm * x_norm + y_norm * y_norm
    k1 = rng.uniform(-0.24, 0.22)
    k2 = rng.uniform(-0.055, 0.055)
    factor = 1.0 + k1 * radius2 + k2 * radius2 * radius2
    map_x = center_x + (xx - center_x) * factor
    map_y = center_y + (yy - center_y) * factor
    warped_image = cv2.remap(image, map_x, map_y, cv2.INTER_LINEAR, borderMode=cv2.BORDER_REFLECT_101)
    warped_mask = cv2.remap(mask, map_x, map_y, cv2.INTER_NEAREST, borderMode=cv2.BORDER_CONSTANT, borderValue=0)
    return warped_image, warped_mask


def _apply_shadows(image: np.ndarray, rng: random.Random, probability: float) -> np.ndarray:
    """Darken broad and sharp bands without changing labels."""
    if rng.random() > max(0.0, min(float(probability), 1.0)):
        return image
    output = image.astype(np.float32)
    height, width = image.shape[:2]
    yy, xx = np.mgrid[0:height, 0:width].astype(np.float32)

    for _ in range(rng.randint(1, 3)):
        mode = rng.choice(("band", "ellipse", "polygon"))
        strength = rng.uniform(0.18, 0.62)
        if mode == "band":
            angle = rng.uniform(0.0, np.pi)
            offset = rng.uniform(-0.55, 0.55) * max(width, height)
            distance = (xx - width * 0.5) * np.cos(angle) + (yy - height * 0.5) * np.sin(angle) - offset
            band_width = rng.uniform(12.0, 80.0)
            mask = np.exp(-(distance * distance) / (2.0 * band_width * band_width))
            if rng.random() < 0.45:
                mask = (distance > -band_width).astype(np.float32) * cv2.GaussianBlur(mask, (0, 0), rng.uniform(8.0, 24.0))
        elif mode == "ellipse":
            center_x = rng.uniform(-0.15, 1.15) * width
            center_y = rng.uniform(-0.10, 1.15) * height
            radius_x = rng.uniform(0.12, 0.65) * width
            radius_y = rng.uniform(0.10, 0.60) * height
            distance = ((xx - center_x) / max(1.0, radius_x)) ** 2 + ((yy - center_y) / max(1.0, radius_y)) ** 2
            mask = np.clip(1.0 - distance, 0.0, 1.0)
            mask = cv2.GaussianBlur(mask, (0, 0), rng.uniform(5.0, 22.0))
        else:
            points = np.array(
                [
                    [rng.uniform(-0.2, 1.2) * width, rng.uniform(-0.2, 1.2) * height],
                    [rng.uniform(-0.2, 1.2) * width, rng.uniform(-0.2, 1.2) * height],
                    [rng.uniform(-0.2, 1.2) * width, rng.uniform(-0.2, 1.2) * height],
                    [rng.uniform(-0.2, 1.2) * width, rng.uniform(-0.2, 1.2) * height],
                ],
                dtype=np.int32,
            )
            mask = np.zeros((height, width), dtype=np.float32)
            cv2.fillConvexPoly(mask, points, 1.0)
            mask = cv2.GaussianBlur(mask, (0, 0), rng.uniform(8.0, 32.0))
        output *= 1.0 - strength * np.clip(mask[..., None], 0.0, 1.0)

    return np.clip(output, 0.0, 255.0).astype(np.uint8)


def _apply_photometrics(image: np.ndarray, rng: random.Random) -> np.ndarray:
    """Apply small exposure/color/noise changes."""
    output = cv2.cvtColor(image, cv2.COLOR_BGR2RGB).astype(np.float32)
    gray = cv2.cvtColor(output.astype(np.uint8), cv2.COLOR_RGB2GRAY).astype(np.float32)
    target_luma = rng.uniform(95.0, 190.0)
    output *= target_luma / max(1.0, float(gray.mean()))
    output = (output - 127.5) * rng.uniform(0.72, 1.35) + 127.5
    gamma = rng.uniform(0.72, 1.42)
    output = 255.0 * np.power(np.clip(output, 0.0, 255.0) / 255.0, gamma)
    gains = np.array([rng.uniform(0.82, 1.22), rng.uniform(0.86, 1.14), rng.uniform(0.82, 1.22)], dtype=np.float32)
    output *= gains.reshape(1, 1, 3)
    if rng.random() < 0.45:
        noise_rng = np.random.default_rng(rng.randint(0, 2**31 - 1))
        output += noise_rng.normal(0.0, rng.uniform(0.5, 5.0), output.shape)
    if rng.random() < 0.20:
        output = cv2.GaussianBlur(output, (0, 0), rng.uniform(0.20, 0.85))
    return cv2.cvtColor(np.clip(output, 0.0, 255.0).astype(np.uint8), cv2.COLOR_RGB2BGR)


def _augment_sample(
    image: np.ndarray,
    polygons: list[list[tuple[float, float]]],
    rng: random.Random,
    args: argparse.Namespace,
    variant_index: int,
) -> tuple[np.ndarray, list[list[tuple[float, float]]]]:
    """Create one hard variant and matching polygons."""
    height, width = image.shape[:2]
    mask = _polygons_to_mask(polygons, width, height)
    augmented_image = image.copy()
    augmented_mask = mask.copy()
    if polygons and rng.random() < max(0.0, min(float(args.affine_probability), 1.0)):
        augmented_image, augmented_mask = _apply_affine(augmented_image, augmented_mask, rng, variant_index)
        if rng.random() < max(0.0, min(float(args.warp_probability), 1.0)):
            if rng.random() < 0.5:
                augmented_image, augmented_mask = _apply_perspective(augmented_image, augmented_mask, rng)
            else:
                augmented_image, augmented_mask = _apply_radial(augmented_image, augmented_mask, rng)
    augmented_image = _apply_shadows(augmented_image, rng, args.shadow_probability)
    augmented_image = _apply_photometrics(augmented_image, rng)
    return augmented_image, _mask_to_polygons(augmented_mask, int(args.max_points))


def _copy_original(
    image: np.ndarray,
    polygons: list[list[tuple[float, float]]],
    image_dir: Path,
    label_dir: Path,
    stem: str,
) -> None:
    """Copy one original sample."""
    cv2.imwrite(str(image_dir / f"{stem}.png"), image)
    _write_label(label_dir / f"{stem}.txt", polygons)


def _augment_split(args: argparse.Namespace, split: str, rng: random.Random) -> int:
    """Augment one dataset split."""
    source_root = Path(args.source_dataset).expanduser().resolve()
    output_root = Path(args.output_dir).expanduser().resolve()
    source_image_dir = source_root / "images" / split
    source_label_dir = source_root / "labels" / split
    target_image_dir = output_root / "images" / split
    target_label_dir = output_root / "labels" / split
    if not source_image_dir.is_dir() or not source_label_dir.is_dir():
        raise FileNotFoundError(f"Split directories not found: {split}")
    target_image_dir.mkdir(parents=True, exist_ok=True)
    target_label_dir.mkdir(parents=True, exist_ok=True)

    created = 0
    for image_path in _image_paths(source_image_dir):
        image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
        if image is None:
            continue
        polygons = _read_polygons(source_label_dir / f"{image_path.stem}.txt")
        if args.include_original:
            stem = f"{args.prefix}_{split}_{created:06d}_orig"
            _copy_original(image, polygons, target_image_dir, target_label_dir, stem)
            created += 1
        for variant_index in range(max(1, int(args.variants_per_image))):
            augmented_image, augmented_polygons = _augment_sample(image, polygons, rng, args, variant_index)
            if polygons and not augmented_polygons:
                continue
            stem = f"{args.prefix}_{split}_{created:06d}_{variant_index}"
            cv2.imwrite(str(target_image_dir / f"{stem}.png"), augmented_image)
            _write_label(target_label_dir / f"{stem}.txt", augmented_polygons)
            created += 1
    return created


def _write_manifest(output_root: Path) -> None:
    """Write a standalone YOLO manifest."""
    (output_root / "data.yaml").write_text(
        f"path: {json.dumps(str(output_root))}\n"
        "train: images/train\n"
        "val: images/val\n"
        "names:\n"
        "  0: cube\n",
        encoding="utf-8",
    )


def main() -> int:
    """Run hard augmentation."""
    args = _parse_args()
    # Keep CLI inspection independent of optional image-processing packages.
    global cv2, np
    import cv2
    import numpy as np
    output_root = Path(args.output_dir).expanduser().resolve()
    if args.overwrite and output_root.exists():
        shutil.rmtree(output_root)
    output_root.mkdir(parents=True, exist_ok=True)
    rng = random.Random(int(args.seed))
    for split in [item.strip() for item in str(args.splits).split(",") if item.strip()]:
        created = _augment_split(args, split, rng)
        print(f"{split}: created={created}", flush=True)
    _write_manifest(output_root)
    print(f"Dataset manifest written to: {output_root / 'data.yaml'}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
