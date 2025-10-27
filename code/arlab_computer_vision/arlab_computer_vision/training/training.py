#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Training script for YOLO11 (Ultralytics) – Detection OR Segmentation.

This script has been adapted for the arlab_computer_vision ROS2 package.

Example:
from arlab_computer_vision.training import train_yolo

# Segmentation
train_yolo(
    task="segment",
    data_path="/path/to/ycb_dataset/data.yaml",
    epochs=100,
    imgsz=640,
    batch=16,
    model_size="n",
    project="runs_ycb",
    name="yolo11n_seg_ycb"
)
"""

import os
import sys
from pathlib import Path
from typing import Optional

import torch
from ultralytics import YOLO

# ============================================================================
# FIXED PATHS - You can adjust the paths here
# ============================================================================

# Path to model weights (Pre-trained Weights)
# Please change this path to your desired directory
base_path = Path(__file__).parent.parent.parent
WEIGHTS_DIR = base_path / "yolo_weights"

# Path for training outputs (where trained models are saved)
# Please change this path to your desired directory
OUTPUT_PROJECT_DIR = base_path / "runs"


def get_package_root() -> Path:
    """Returns the fixed output path."""
    return OUTPUT_PROJECT_DIR


def build_default_weights(task: str, size: str, package_root: Path) -> str:
    """Create default weights path based on task and size.

    Args:
        task: 'segment' or 'detect'
        size: 'n', 's', 'm', 'l', 'x'
        package_root: Not used here, but for API compatibility

    Returns:
        Path to weights
    """
    size = size.lower()

    # Use the fixed WEIGHTS_DIR
    if WEIGHTS_DIR.exists():
        if task == "segment":
            weight_file = WEIGHTS_DIR / f"yolo11{size}-seg.pt"
            if weight_file.exists():
                return str(weight_file)
        elif task == "detect":
            weight_file = WEIGHTS_DIR / f"yolo11{size}.pt"
            if weight_file.exists():
                return str(weight_file)

    # Fallback to Ultralytics download
    if task == "segment":
        return f"yolo11{size}-seg.pt"
    elif task == "detect":
        return f"yolo11{size}.pt"
    else:
        raise ValueError("task must be 'segment' or 'detect'")


def ensure_yaml(data_yaml: str, names_csv: str | None):
    """Optional: Generate a minimal YAML if it's missing.

    Expects folder structure in YOLO format:
      images/train, images/val, images/test (optional)
    """
    p = Path(data_yaml)
    if p.exists():
        return
    if names_csv is None:
        msg = (
            f"{data_yaml} does not exist. "
            "Either specify YAML or use --names 'class1,class2,...'."
        )
        raise FileNotFoundError(msg)
    root = p.parent
    # Heuristic: folders relative to YAML
    images_train = "images/train"
    images_val = "images/val"
    images_test = "images/test"
    names = [n.strip() for n in names_csv.split(",") if n.strip()]
    text = [
        f"path: {root.resolve()}",
        f"train: {images_train}",
        f"val: {images_val}",
        f"test: {images_test}",
        "",
        "names:",
    ]
    for i, n in enumerate(names):
        text.append(f"  {i}: {n}")
    p.write_text("\n".join(text), encoding="utf-8")
    print(f"[INFO] YAML created: {p}")


def maybe_convert_masks_to_yolo_seg(masks_dir: str, images_dir: str, labels_out: str):
    """Optional: Instance masks -> YOLO-Seg polygons (.txt)

    Expects binary/instance masks per image (*_0.png, *_1.png, ...).
    """
    from ultralytics.data.converter import convert_segment_masks_to_yolo_seg

    os.makedirs(labels_out, exist_ok=True)
    convert_segment_masks_to_yolo_seg(
        masks_dir=masks_dir, output_dir=labels_out, images_dir=images_dir
    )
    print(f"[INFO] Masks -> YOLO-Seg labels converted to: {labels_out}")


def detect_device():
    """Automatic GPU detection."""
    try:
        import torch  # noqa: F401

        if torch.cuda.is_available():
            device = "0"  # First GPU
            print(f"[INFO] GPU found: {torch.cuda.get_device_name(0)}")
            return device
        else:
            print("[INFO] No GPU found, using CPU.")
            return "cpu"
    except ImportError:
        print("[INFO] PyTorch not installed, using CPU.")
        return "cpu"


def train_yolo(
    task: str = "segment",
    data_path: str = "",
    epochs: int = 100,
    imgsz: int = 640,
    batch: int = 16,
    model_size: str = "n",
    project: str = "runs_ycb",
    name: Optional[str] = None,
    weights: Optional[str] = None,
    device: Optional[str] = None,
    workers: int = 8,
    patience: int = 20,
    resume: bool = False,
    seed: int = 0,
    cos_lr: bool = False,
    lr0: float = 0.01,
    freeze: int = 0,
    **kwargs,
):
    """Train a YOLO11 model.

    Args:
        task: 'segment' or 'detect'
        data_path: Path to data.yaml
        epochs: Number of epochs
        imgsz: Image size
        batch: Batch size
        model_size: Model size ('n', 's', 'm', 'l', 'x')
        project: Project folder
        name: Run name
        weights: Path to pre-trained weights
        device: Device ('0', 'cpu', etc.)
        workers: Number of worker threads
        patience: Early stopping patience
        resume: Resume training
        seed: Random seed
        cos_lr: Cosine LR Scheduler
        lr0: Initial learning rate
        freeze: Number of layers to freeze
        **kwargs: Additional Ultralytics parameters

    Returns:
        Training results
    """
    package_root = get_package_root()

    # Default weights with package integration
    weights = weights or build_default_weights(task, model_size, package_root)
    print(f"[INFO] Using weights: {weights}")

    # Device auto-detection
    device = device or detect_device()
    print(f"[INFO] Device: {device}")

    # Load model
    print(f"[INFO] Loading model: {weights}")
    model = YOLO(weights)

    # Assemble training task
    train_kwargs = dict(
        data=data_path,
        epochs=epochs,
        imgsz=imgsz,
        batch=batch,
        device=device,
        project=str(OUTPUT_PROJECT_DIR),
        name=name,
        workers=workers,
        patience=patience,
        resume=resume,
        seed=seed,
        cos_lr=cos_lr,
        lr0=lr0,
        freeze=freeze,
        # Sensible augmentation defaults for arlab
        hsv_h=0.015,
        hsv_s=0.7,
        hsv_v=0.4,
        mosaic=1.0,
        # Additional sensible settings
        dropout=0.0,
        amp=True,  # Mixed precision training
        **kwargs,
    )

    # Start training
    print("\n" + "=" * 60)
    print("[INFO] Starting training:")
    print(f"  Task:        {task}")
    print(f"  Weights:     {weights}")
    print(f"  Data:        {data_path}")
    print(f"  Model-Size:  {model_size}")
    print(f"  Device:      {device}")
    print(f"  Epochs:      {epochs}")
    print(f"  Image-Size:  {imgsz}")
    print(f"  Batch-Size:  {batch}")
    print("=" * 60 + "\n")

    results = model.train(**train_kwargs)

    # Val (optional, usually already included in training)
    print("\n[INFO] Final evaluation...")
    model.val(data=data_path, imgsz=imgsz, device=device)

    print("\n[INFO] Best model:")
    if hasattr(results, "best") and results.best is not None:
        print(f"  Path: {results.best}")

    # Export (ONNX as example)
    if results.best:
        print("\n[INFO] Exporting best model to ONNX...")
        try:
            model_best = YOLO(str(results.best))
            model_best.export(format="onnx", opset=12, simplify=True)
            print("[OK] ONNX export successful.")
        except Exception as e:
            print(f"[WARN] ONNX export failed: {e}")

    print(f"\n[DONE] Training completed. See: {OUTPUT_PROJECT_DIR}/{name or 'train'}")

    return results


# For direct script execution
if __name__ == "__main__":
    # Example usage
    import sys

    if len(sys.argv) < 2:
        print("Usage: train_yolo(data_path='/path/to/data.yaml', ...)")
        print("\nExample:")
        print("train_yolo(")
        print("    task='segment',")
        print("    data_path='/path/to/data.yaml',")
        print("    epochs=100,")
        print("    imgsz=640,")
        print("    batch=16")
        print(")")
        sys.exit(1)

    # For simple usage: path to data.yaml as first argument
    data_path = sys.argv[1]

    train_yolo(
        task=sys.argv[2] if len(sys.argv) > 2 else "segment",
        data_path=data_path,
        epochs=int(sys.argv[3]) if len(sys.argv) > 3 else 100,
        imgsz=int(sys.argv[4]) if len(sys.argv) > 4 else 640,
        batch=int(sys.argv[5]) if len(sys.argv) > 5 else 16,
    )
