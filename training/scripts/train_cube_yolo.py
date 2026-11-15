#!/usr/bin/env python3
"""Responsibility: Minimal CLI wrapper for fine-tuning a single-class cube YOLO model."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path


def _parse_args() -> argparse.Namespace:
    """Read training parameters from the command line."""
    parser = argparse.ArgumentParser(description="Fine-tune Ultralytics YOLO for a single cube class")
    parser.add_argument("--data", required=True, help="Path to YOLO data.yaml")
    parser.add_argument("--task", choices=("detect", "segment"), default="detect", help="YOLO training task")
    parser.add_argument("--weights", required=True, help="Existing local base checkpoint; a missing input is rejected")
    parser.add_argument("--epochs", type=int, default=100, help="Number of training epochs")
    parser.add_argument("--imgsz", type=int, default=640, help="Training image size")
    parser.add_argument("--batch", type=int, default=16, help="Batch size")
    parser.add_argument("--device", default="0", help="Ultralytics device string, e.g. 0 or cpu")
    parser.add_argument("--project", default="runs/train", help="Output project directory")
    parser.add_argument("--name", default="", help="Training run name")
    parser.add_argument("--workers", type=int, default=8, help="Number of dataloader workers")
    parser.add_argument("--patience", type=int, default=25, help="Early stopping patience")
    parser.add_argument("--optimizer", default="auto", help="Ultralytics optimizer, e.g. auto, SGD, AdamW")
    parser.add_argument("--lr0", type=float, default=None, help="Initial learning rate override")
    parser.add_argument("--lrf", type=float, default=None, help="Final LR fraction override")
    parser.add_argument("--freeze", type=int, default=None, help="Freeze first N model layers")
    parser.add_argument("--close_mosaic", type=int, default=None, help="Disable mosaic for final N epochs")
    parser.add_argument("--mosaic", type=float, default=None, help="Mosaic augmentation probability")
    parser.add_argument("--mixup", type=float, default=None, help="MixUp augmentation probability")
    parser.add_argument("--copy_paste", type=float, default=None, help="Copy-paste augmentation probability for segmentation")
    parser.add_argument("--hsv_h", type=float, default=None, help="HSV hue augmentation gain")
    parser.add_argument("--hsv_s", type=float, default=None, help="HSV saturation augmentation gain")
    parser.add_argument("--hsv_v", type=float, default=None, help="HSV value augmentation gain")
    parser.add_argument("--degrees", type=float, default=None, help="Image rotation augmentation range in degrees")
    parser.add_argument("--translate", type=float, default=None, help="Image translation augmentation fraction")
    parser.add_argument("--scale", type=float, default=None, help="Image scale augmentation gain")
    parser.add_argument("--perspective", type=float, default=None, help="Perspective augmentation gain")
    parser.add_argument("--fliplr", type=float, default=None, help="Horizontal flip augmentation probability")
    parser.add_argument("--flipud", type=float, default=None, help="Vertical flip augmentation probability")
    parser.add_argument("--fraction", type=float, default=None, help="Fraction of training data to use")
    parser.add_argument(
        "--cache",
        choices=("false", "ram", "disk"),
        default="false",
        help="Ultralytics cache mode",
    )
    parser.add_argument("--resume", action="store_true", help="Resume the existing run")
    parser.add_argument("--seed", type=int, default=42, help="Random seed")
    args = parser.parse_args()
    if not args.name:
        args.name = "cube_yolo26n_seg" if args.task == "segment" else "cube_yolo26n"
    return args


def _fail(message: str) -> int:
    """Print a short actionable error and stop."""
    print(message, file=sys.stderr)
    return 1


def _resolve_path(path_value: str) -> Path:
    """Normalize paths before validation and logging."""
    return Path(path_value).expanduser().resolve()


def _validate_args(args: argparse.Namespace) -> tuple[Path, Path]:
    """Check input files early so training fails fast and clearly."""
    data_path = _resolve_path(args.data)
    if not data_path.is_file():
        raise FileNotFoundError(f"Data file not found: {data_path}")

    weights_path = _resolve_path(args.weights)
    if not weights_path.is_file():
        raise FileNotFoundError(f"Weights file not found: {weights_path}")
    return data_path, weights_path


def _cache_value(raw_value: str) -> bool | str:
    """Convert CLI cache mode to the value expected by Ultralytics."""
    return False if raw_value == "false" else raw_value


def main() -> int:
    """Validate local inputs and train only in the explicitly selected Python."""
    args = _parse_args()

    try:
        data_path, resolved_weights = _validate_args(args)
    except FileNotFoundError as exc:
        return _fail(str(exc))

    try:
        from ultralytics import YOLO
    except ImportError:
        return _fail(
            "Ultralytics is not installed in the current Python environment. "
            "Install it first and rerun training."
        )

    model = YOLO(str(resolved_weights), task=args.task)
    train_kwargs = {
        "data": str(data_path),
        "task": args.task,
        "epochs": args.epochs,
        "imgsz": args.imgsz,
        "batch": args.batch,
        "device": args.device,
        "project": args.project,
        "name": args.name,
        "workers": args.workers,
        "patience": args.patience,
        "optimizer": args.optimizer,
        "cache": _cache_value(args.cache),
        "resume": args.resume,
        "seed": args.seed,
    }
    for key in (
        "lr0",
        "lrf",
        "freeze",
        "close_mosaic",
        "mosaic",
        "mixup",
        "copy_paste",
        "hsv_h",
        "hsv_s",
        "hsv_v",
        "degrees",
        "translate",
        "scale",
        "perspective",
        "fliplr",
        "flipud",
        "fraction",
    ):
        value = getattr(args, key)
        if value is not None:
            train_kwargs[key] = value

    results = model.train(
        **train_kwargs,
    )

    save_dir = getattr(results, "save_dir", None)
    if save_dir is not None:
        best_path = Path(save_dir) / "weights" / "best.pt"
        if best_path.is_file():
            print(f"best.pt: {best_path}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
