#!/usr/bin/env python3
"""Responsibility: Create photometric YOLO segmentation augmentations without changing labels."""

from __future__ import annotations

import argparse
import json
import random
import shutil
from pathlib import Path



def _parse_args() -> argparse.Namespace:
    """Read augmentation parameters."""
    parser = argparse.ArgumentParser(description="Create lighting/color augmentations for YOLO segmentation data")
    parser.add_argument("--source_dataset", required=True, help="Source YOLO dataset root with images/ and labels/")
    parser.add_argument("--output_dir", required=True, help="Output YOLO dataset root")
    parser.add_argument("--splits", default="train,val", help="Comma-separated dataset splits")
    parser.add_argument("--variants_per_image", type=int, default=2, help="Augmented variants per source image")
    parser.add_argument("--seed", type=int, default=5302, help="Random seed")
    parser.add_argument("--prefix", default="photo_aug", help="Filename prefix for generated samples")
    parser.add_argument("--include_original", action="store_true", help="Copy original source samples too")
    parser.add_argument("--overwrite", action="store_true", help="Remove output directory before writing")
    parser.add_argument("--target_luma_min", type=float, default=70.0, help="Minimum target mean luma")
    parser.add_argument("--target_luma_max", type=float, default=215.0, help="Maximum target mean luma")
    parser.add_argument("--shadow_probability", type=float, default=0.35, help="Probability of a soft synthetic shadow")
    return parser.parse_args()


def _image_paths(image_dir: Path) -> list[Path]:
    """Return images supported by OpenCV in a stable order."""
    paths: list[Path] = []
    for suffix in ("*.png", "*.jpg", "*.jpeg"):
        paths.extend(image_dir.glob(suffix))
    return sorted(paths)


def _apply_soft_shadow(image: np.ndarray, rng: random.Random) -> np.ndarray:
    """Darken a random broad image area to emulate robot/table shadows."""
    height, width = image.shape[:2]
    center_x = rng.uniform(-0.2 * width, 1.2 * width)
    center_y = rng.uniform(-0.2 * height, 1.2 * height)
    radius_x = rng.uniform(0.25 * width, 0.75 * width)
    radius_y = rng.uniform(0.20 * height, 0.70 * height)
    yy, xx = np.mgrid[0:height, 0:width]
    distance = ((xx - center_x) / max(1.0, radius_x)) ** 2 + ((yy - center_y) / max(1.0, radius_y)) ** 2
    mask = np.clip(1.0 - distance, 0.0, 1.0)
    mask = cv2.GaussianBlur(mask.astype(np.float32), (0, 0), sigmaX=rng.uniform(8.0, 28.0))
    strength = rng.uniform(0.10, 0.45)
    return image * (1.0 - strength * mask[..., None])


def _augment_image(image_bgr: np.ndarray, rng: random.Random, args: argparse.Namespace) -> np.ndarray:
    """Apply exposure, contrast, color-temperature and sensor-noise changes."""
    image = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB).astype(np.float32)
    gray = cv2.cvtColor(image.astype(np.uint8), cv2.COLOR_RGB2GRAY).astype(np.float32)
    current_luma = max(1.0, float(gray.mean()))
    target_min = max(25.0, min(float(args.target_luma_min), 230.0))
    target_max = max(target_min + 1.0, min(float(args.target_luma_max), 245.0))
    target_luma = rng.uniform(target_min, target_max)
    image *= target_luma / current_luma

    contrast = rng.uniform(0.68, 1.38)
    image = (image - 127.5) * contrast + 127.5

    gamma = rng.uniform(0.70, 1.45)
    image = 255.0 * np.power(np.clip(image, 0.0, 255.0) / 255.0, gamma)

    gains = np.array(
        [
            rng.uniform(0.80, 1.25),
            rng.uniform(0.86, 1.16),
            rng.uniform(0.80, 1.25),
        ],
        dtype=np.float32,
    )
    image *= gains.reshape(1, 1, 3)

    if rng.random() < max(0.0, min(float(args.shadow_probability), 1.0)):
        image = _apply_soft_shadow(image, rng)

    if rng.random() < 0.45:
        noise_rng = np.random.default_rng(rng.randint(0, 2**31 - 1))
        image += noise_rng.normal(0.0, rng.uniform(0.5, 5.5), image.shape)

    if rng.random() < 0.18:
        image = cv2.GaussianBlur(image, (0, 0), rng.uniform(0.20, 0.85))

    return cv2.cvtColor(np.clip(image, 0.0, 255.0).astype(np.uint8), cv2.COLOR_RGB2BGR)


def _copy_label(source_label: Path, target_label: Path) -> None:
    """Copy a YOLO label, creating an empty file for negative samples if needed."""
    if source_label.is_file():
        shutil.copy2(source_label, target_label)
    else:
        target_label.write_text("", encoding="utf-8")


def _augment_split(args: argparse.Namespace, split: str, rng: random.Random) -> int:
    """Create one augmented split."""
    source_root = Path(args.source_dataset).expanduser().resolve()
    output_root = Path(args.output_dir).expanduser().resolve()
    source_image_dir = source_root / "images" / split
    source_label_dir = source_root / "labels" / split
    target_image_dir = output_root / "images" / split
    target_label_dir = output_root / "labels" / split
    if not source_image_dir.is_dir() or not source_label_dir.is_dir():
        raise FileNotFoundError(f"Split directories not found in source dataset: {split}")

    target_image_dir.mkdir(parents=True, exist_ok=True)
    target_label_dir.mkdir(parents=True, exist_ok=True)

    created = 0
    for image_path in _image_paths(source_image_dir):
        label_path = source_label_dir / f"{image_path.stem}.txt"
        image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
        if image is None:
            continue

        if args.include_original:
            target_stem = f"{args.prefix}_{split}_{created:06d}_orig"
            cv2.imwrite(str(target_image_dir / f"{target_stem}.png"), image)
            _copy_label(label_path, target_label_dir / f"{target_stem}.txt")
            created += 1

        for variant_index in range(max(1, int(args.variants_per_image))):
            target_stem = f"{args.prefix}_{split}_{created:06d}_{variant_index}"
            augmented = _augment_image(image, rng, args)
            cv2.imwrite(str(target_image_dir / f"{target_stem}.png"), augmented)
            _copy_label(label_path, target_label_dir / f"{target_stem}.txt")
            created += 1

    return created


def _write_data_yaml(output_root: Path) -> None:
    """Write a standalone YOLO manifest for the augmented dataset."""
    yaml_text = (
        f"path: {json.dumps(str(output_root))}\n"
        "train: images/train\n"
        "val: images/val\n"
        "names:\n"
        "  0: cube\n"
    )
    (output_root / "data.yaml").write_text(yaml_text, encoding="utf-8")


def main() -> int:
    """Run photometric augmentation."""
    args = _parse_args()
    # Keep CLI inspection independent of optional image-processing packages.
    global cv2, np
    import cv2
    import numpy as np
    output_root = Path(args.output_dir).expanduser().resolve()
    if args.overwrite and output_root.exists():
        shutil.rmtree(output_root)
    output_root.mkdir(parents=True, exist_ok=True)

    rng = random.Random(args.seed)
    for split in [item.strip() for item in args.splits.split(",") if item.strip()]:
        created = _augment_split(args, split, rng)
        print(f"{split}: created={created}", flush=True)
    _write_data_yaml(output_root)
    print(f"Dataset manifest written to: {output_root / 'data.yaml'}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
