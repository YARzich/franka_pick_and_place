#!/usr/bin/env python3
"""Responsibility: Create YOLO segmentation labels from Franka cube screenshot folders."""

from __future__ import annotations

import argparse
import json
import random
import shutil
from pathlib import Path



IMAGE_SUFFIXES = (".png", ".jpg", ".jpeg")


def _parse_args() -> argparse.Namespace:
    """Read command line arguments."""
    parser = argparse.ArgumentParser(description="Build a small hard-real cube screenshot segmentation dataset")
    parser.add_argument("--source_dir", action="append", required=True, help="Screenshot directory; can be repeated")
    parser.add_argument("--output_dir", required=True, help="Output YOLO dataset root")
    parser.add_argument("--split", choices=("auto", "train", "val"), default="auto", help="Target split mode")
    parser.add_argument("--val_ratio", type=float, default=0.20, help="Validation ratio for split=auto")
    parser.add_argument("--seed", type=int, default=5302, help="Deterministic split seed")
    parser.add_argument("--prefix", default="real_hard", help="Output filename prefix")
    parser.add_argument("--overwrite", action="store_true", help="Remove output directory before writing")
    parser.add_argument("--cover_fps_overlay", action="store_true", help="Cover the top-left debug FPS text")
    parser.add_argument("--contact_sheet", default="", help="Optional path for a contour contact sheet")
    parser.add_argument("--seg_approx_epsilon_ratio", type=float, default=0.008, help="Contour simplification ratio")
    parser.add_argument("--seg_max_points", type=int, default=40, help="Maximum points in YOLO polygon")
    parser.add_argument(
        "--mask_dilate_ratio",
        type=float,
        default=0.0,
        help="Dilate extracted masks by this fraction of the shorter image side before polygon export",
    )
    return parser.parse_args()


def _image_paths(source_dirs: list[str]) -> list[Path]:
    """Collect supported image paths in stable order."""
    paths: list[Path] = []
    for source_dir in source_dirs:
        root = Path(source_dir).expanduser().resolve()
        for path in sorted(root.iterdir()):
            if path.suffix.lower() in IMAGE_SUFFIXES:
                paths.append(path)
    return paths


def _cover_fps_overlay(image_bgr: np.ndarray) -> np.ndarray:
    """Hide debug-window FPS text so screenshots look closer to raw camera frames."""
    image = image_bgr.copy()
    height, width = image.shape[:2]
    x1 = int(round(0.31 * width))
    y1 = int(round(0.15 * height))
    patch_y0 = min(height - 1, y1)
    patch_y1 = min(height, max(y1 + 8, int(round(0.25 * height))))
    patch = image[patch_y0:patch_y1, 0:x1]
    if patch.size == 0:
        fill = np.array([128, 128, 128], dtype=np.uint8)
    else:
        fill = np.median(patch.reshape(-1, 3), axis=0).astype(np.uint8)
    image[0:y1, 0:x1] = fill.reshape(1, 1, 3)
    image[0:y1, 0:x1] = cv2.GaussianBlur(image[0:y1, 0:x1], (0, 0), 2.0)
    return image


def _candidate_mask(image_bgr: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Build a chroma mask and return it with HSV channels."""
    height, width = image_bgr.shape[:2]
    hsv = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)
    hue, saturation, value = hsv[:, :, 0], hsv[:, :, 1], hsv[:, :, 2]
    mask = ((saturation > 42) & (value > 35)).astype(np.uint8) * 255

    yy, xx = np.mgrid[0:height, 0:width]
    yellow_fps = (
        (yy < 0.16 * height)
        & (xx < 0.30 * width)
        & (hue >= 16)
        & (hue <= 46)
        & (saturation > 60)
        & (value > 100)
    )
    mask[yellow_fps] = 0

    kernel_size = max(3, int(round(min(width, height) * 0.005)) | 1)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    return mask, hue, saturation


def _extract_cube_mask(image_bgr: np.ndarray) -> np.ndarray:
    """Extract the most likely colored cube component from one screenshot."""
    height, width = image_bgr.shape[:2]
    mask, hue, saturation = _candidate_mask(image_bgr)
    component_count, labels, stats, centroids = cv2.connectedComponentsWithStats(mask, 8)
    candidates: list[tuple[float, int]] = []

    for index in range(1, component_count):
        x_value, y_value, box_width, box_height, area = stats[index]
        area_ratio = area / float(width * height)
        if area < max(12, 0.00006 * width * height) or area_ratio > 0.08:
            continue
        aspect = box_width / max(1, box_height)
        if aspect < 0.15 or aspect > 6.0:
            continue
        fill_ratio = area / max(1, box_width * box_height)
        if fill_ratio < 0.10:
            continue

        component = labels == index
        component_hue = hue[component]
        red_fraction = float(((component_hue <= 12) | (component_hue >= 165)).mean())
        cyan_fraction = float(((component_hue >= 75) & (component_hue <= 110)).mean())
        yellow_fraction = float(((component_hue >= 16) & (component_hue <= 46)).mean())
        mean_saturation = float(saturation[component].mean())
        center_x, center_y = centroids[index]

        score = mean_saturation / 255.0
        score += min(area_ratio / 0.025, 1.5) * 0.45
        score += fill_ratio * 0.45
        score += red_fraction * 2.8
        score += cyan_fraction * 0.25
        score -= yellow_fraction * 1.6

        # Penalize UI/large checkerboard-like components; keep compact colored objects.
        if center_y < 0.13 * height and center_x < 0.32 * width:
            score -= 1.2
        if box_width > 0.28 * width or box_height > 0.28 * height:
            score -= 1.4
        if y_value < 0.10 * height and box_height > 0.08 * height and box_width > 0.12 * width:
            score -= 1.0

        candidates.append((score, index))

    if not candidates:
        return np.zeros((height, width), dtype=np.uint8)

    _, selected_index = max(candidates, key=lambda item: item[0])
    selected = (labels == selected_index).astype(np.uint8) * 255
    close_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    return cv2.morphologyEx(selected, cv2.MORPH_CLOSE, close_kernel)


def _mask_to_polygon(mask: np.ndarray, epsilon_ratio: float, max_points: int) -> list[tuple[float, float]]:
    """Convert a binary mask into one normalized YOLO polygon."""
    height, width = mask.shape[:2]
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return []
    contour = max(contours, key=cv2.contourArea)
    if cv2.contourArea(contour) < 8.0:
        return []
    epsilon = max(1.0, float(epsilon_ratio) * cv2.arcLength(contour, True))
    approximated = cv2.approxPolyDP(contour, epsilon, True).reshape(-1, 2)
    if len(approximated) < 3:
        x_value, y_value, box_width, box_height = cv2.boundingRect(contour)
        approximated = np.array(
            [
                [x_value, y_value],
                [x_value + box_width, y_value],
                [x_value + box_width, y_value + box_height],
                [x_value, y_value + box_height],
            ],
            dtype=np.float32,
        )
    max_points = max(3, int(max_points))
    if len(approximated) > max_points:
        indices = np.linspace(0, len(approximated) - 1, max_points, dtype=int)
        approximated = approximated[indices]
    polygon: list[tuple[float, float]] = []
    for x_value, y_value in approximated:
        polygon.append((float(np.clip(x_value / width, 0.0, 1.0)), float(np.clip(y_value / height, 0.0, 1.0))))
    return polygon


def _dilate_mask(mask: np.ndarray, ratio: float) -> np.ndarray:
    """Slightly expand tiny color masks so hard edge screenshots teach detection, not single pixels."""
    ratio = max(0.0, float(ratio))
    if ratio <= 0.0 or not np.any(mask):
        return mask
    height, width = mask.shape[:2]
    radius = max(1, int(round(min(width, height) * ratio)))
    kernel_size = radius * 2 + 1
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))
    return cv2.dilate(mask, kernel, iterations=1)


def _write_label(path: Path, polygon: list[tuple[float, float]]) -> None:
    """Write one YOLO segmentation label."""
    coords: list[str] = []
    for x_value, y_value in polygon:
        coords.append(f"{x_value:.6f}")
        coords.append(f"{y_value:.6f}")
    path.write_text(f"0 {' '.join(coords)}\n", encoding="utf-8")


def _split_paths(paths: list[Path], args: argparse.Namespace) -> dict[str, list[Path]]:
    """Assign images to train/val splits."""
    if args.split == "train":
        return {"train": paths, "val": []}
    if args.split == "val":
        return {"train": [], "val": paths}

    rng = random.Random(int(args.seed))
    shuffled = paths[:]
    rng.shuffle(shuffled)
    val_count = max(1, int(round(len(shuffled) * max(0.0, min(float(args.val_ratio), 0.8)))))
    val_set = set(shuffled[:val_count])
    return {
        "train": [path for path in paths if path not in val_set],
        "val": [path for path in paths if path in val_set],
    }


def _draw_contact(visuals: list[np.ndarray], output_path: Path) -> None:
    """Write a compact sheet for quick annotation inspection."""
    if not visuals:
        return
    thumb_width = 220
    thumbs = []
    for visual in visuals:
        height, width = visual.shape[:2]
        thumb_height = int(round(height * thumb_width / width))
        thumbs.append(cv2.resize(visual, (thumb_width, thumb_height), interpolation=cv2.INTER_AREA))
    cols = 4
    rows = (len(thumbs) + cols - 1) // cols
    cell_height = max(thumb.shape[0] for thumb in thumbs)
    sheet = np.zeros((rows * cell_height, cols * thumb_width, 3), dtype=np.uint8) + 40
    for index, thumb in enumerate(thumbs):
        row = index // cols
        col = index % cols
        sheet[row * cell_height : row * cell_height + thumb.shape[0], col * thumb_width : col * thumb_width + thumb.shape[1]] = thumb
    output_path.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(output_path), sheet)


def _write_manifest(output_root: Path) -> None:
    """Write YOLO data.yaml."""
    (output_root / "data.yaml").write_text(
        f"path: {json.dumps(str(output_root))}\n"
        "train: images/train\n"
        "val: images/val\n"
        "names:\n"
        "  0: cube\n",
        encoding="utf-8",
    )


def main() -> int:
    """Create the dataset."""
    args = _parse_args()
    # Keep CLI inspection independent of optional image-processing packages.
    global cv2, np
    import cv2
    import numpy as np
    output_root = Path(args.output_dir).expanduser().resolve()
    if args.overwrite and output_root.exists():
        shutil.rmtree(output_root)
    for split in ("train", "val"):
        (output_root / "images" / split).mkdir(parents=True, exist_ok=True)
        (output_root / "labels" / split).mkdir(parents=True, exist_ok=True)

    paths = _image_paths(args.source_dir)
    split_paths = _split_paths(paths, args)
    visuals: list[np.ndarray] = []
    failed: list[Path] = []

    for split, split_images in split_paths.items():
        for index, source_path in enumerate(split_images):
            image = cv2.imread(str(source_path), cv2.IMREAD_COLOR)
            if image is None:
                failed.append(source_path)
                continue
            processed = _cover_fps_overlay(image) if args.cover_fps_overlay else image
            mask = _dilate_mask(_extract_cube_mask(image), args.mask_dilate_ratio)
            polygon = _mask_to_polygon(mask, args.seg_approx_epsilon_ratio, args.seg_max_points)
            if len(polygon) < 3:
                failed.append(source_path)
                continue

            stem = f"{args.prefix}_{split}_{index:04d}"
            cv2.imwrite(str(output_root / "images" / split / f"{stem}.png"), processed)
            _write_label(output_root / "labels" / split / f"{stem}.txt", polygon)

            visual = processed.copy()
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cv2.drawContours(visual, contours, -1, (0, 255, 0), 2)
            cv2.putText(visual, source_path.name[-12:-4], (8, visual.shape[0] - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)
            visuals.append(visual)

    _write_manifest(output_root)
    if args.contact_sheet:
        _draw_contact(visuals, Path(args.contact_sheet).expanduser().resolve())
    if failed:
        print("Failed to annotate:", flush=True)
        for path in failed:
            print(f"  {path}", flush=True)
        return 1
    print(f"Dataset written to: {output_root}", flush=True)
    print(f"train={len(split_paths['train'])} val={len(split_paths['val'])}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
