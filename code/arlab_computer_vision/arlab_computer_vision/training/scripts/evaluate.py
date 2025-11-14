#!/usr/bin/env python3
"""
Evaluate YOLO segmentation models on images.

Supports two modes:
1. Single image evaluation (banner image)
2. Batch evaluation (all images in test_data directory)
"""

import argparse
from pathlib import Path
from typing import Iterable

import cv2
import numpy as np
from ultralytics import YOLO


def overlay_masks(frame: np.ndarray, results) -> np.ndarray:
    """Draw segmentation masks, boxes, and labels on a copy of the frame."""
    if len(results) == 0:
        return frame

    result = results[0]
    boxes = getattr(result, "boxes", None)
    masks = getattr(result, "masks", None)
    names = result.names

    if boxes is None or len(boxes) == 0:
        return frame

    vis = frame.copy()

    if masks is not None:
        masks_data = masks.data.cpu().numpy()
    else:
        masks_data = None

    xyxy = boxes.xyxy.cpu().numpy()
    scores = boxes.conf.cpu().numpy()
    class_ids = boxes.cls.cpu().numpy().astype(int)

    for i in range(len(boxes)):
        x1, y1, x2, y2 = map(int, xyxy[i])
        label = names[class_ids[i]]
        score = scores[i]

        cv2.rectangle(vis, (x1, y1), (x2, y2), (0, 255, 0), 2)

        caption = f"{label} {score:.2f}"
        cv2.putText(
            vis,
            caption,
            (x1, max(0, y1 - 10)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 0),
            2,
        )

        if masks_data is not None:
            mask = masks_data[i]
            mask_resized = cv2.resize(
                mask, (vis.shape[1], vis.shape[0]), interpolation=cv2.INTER_NEAREST
            )
            colored_mask = np.zeros_like(vis)
            colored_mask[:] = (0, 255, 0)
            alpha = 0.4
            mask_indices = mask_resized > 0.5
            vis[mask_indices] = cv2.addWeighted(
                vis[mask_indices], 1 - alpha, colored_mask[mask_indices], alpha, 0
            )

    return vis


def iter_images(root: Path) -> Iterable[Path]:
    """Yield all image files recursively under root."""
    for path in root.rglob("*"):
        if path.is_file() and path.suffix.lower() in {".jpg", ".jpeg", ".png"}:
            yield path


def evaluate_single_image(
    model: YOLO, image_path: Path, output_path: Path, conf: float = 0.1
):
    """Evaluate model on a single image."""
    frame_bgr = cv2.imread(str(image_path))
    if frame_bgr is None:
        raise RuntimeError(f"Failed to load image: {image_path}")

    frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)

    results = model(frame_rgb, conf=conf, verbose=False)

    result = results[0]
    boxes = getattr(result, "boxes", None)
    if boxes is None or len(boxes) == 0:
        print(f"⚠️  No detections found in {image_path.name}")

    vis_rgb = overlay_masks(frame_rgb, results)
    vis_bgr = cv2.cvtColor(vis_rgb, cv2.COLOR_RGB2BGR)

    output_path.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(output_path), vis_bgr)
    print(f"Saved visualization to {output_path}")


def evaluate_batch(model: YOLO, eval_root: Path, output_root: Path, conf: float = 0.1):
    """Evaluate model on all images in eval_root, preserving directory structure."""
    image_paths = list(iter_images(eval_root))
    if not image_paths:
        print(f"No images found under {eval_root}")
        return

    for idx, image_path in enumerate(image_paths, start=1):
        frame_bgr = cv2.imread(str(image_path))
        if frame_bgr is None:
            print(f"[WARN] Failed to load image: {image_path}")
            continue

        frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        results = model(frame_rgb, conf=conf, verbose=False)

        vis_rgb = overlay_masks(frame_rgb, results)
        vis_bgr = cv2.cvtColor(vis_rgb, cv2.COLOR_RGB2BGR)

        relative_path = image_path.relative_to(eval_root)
        output_path = output_root / relative_path
        output_path.parent.mkdir(parents=True, exist_ok=True)

        cv2.imwrite(str(output_path), vis_bgr)
        print(f"[{idx}/{len(image_paths)}] Saved {output_path}")

    print("Done.")


def main() -> None:
    parser = argparse.ArgumentParser(description="Evaluate YOLO segmentation model")
    parser.add_argument(
        "--model",
        type=str,
        required=True,
        help="Model name (e.g., yolo11n-seg-YCB)",
    )
    parser.add_argument(
        "--mode",
        type=str,
        choices=["single", "batch"],
        default="single",
        help="Evaluation mode: single image or batch",
    )
    parser.add_argument(
        "--image",
        type=Path,
        help="Path to single image (for single mode)",
    )
    parser.add_argument(
        "--conf",
        type=float,
        default=0.1,
        help="Confidence threshold",
    )

    args = parser.parse_args()

    # Determine base directory (assuming script is in training/scripts/)
    script_dir = Path(__file__).parent
    training_dir = script_dir.parent
    # training_dir is: .../arlab_computer_vision/training
    # We need: .../arlab_computer_vision (where yolo_weights/ is located)
    base_dir = training_dir.parent.parent

    model_path = base_dir / f"yolo_weights/{args.model}.pt"
    if not model_path.exists():
        raise FileNotFoundError(f"Model checkpoint not found: {model_path}")

    model = YOLO(str(model_path))

    if args.mode == "single":
        if args.image is None:
            # Default to banner image
            image_path = training_dir / "data/test_images/ycb_banner.jpg"
        else:
            image_path = args.image

        if not image_path.exists():
            raise FileNotFoundError(f"Image not found: {image_path}")

        output_path = (
            training_dir
            / f"evaluation/results/{args.model}/{image_path.stem}_prediction.png"
        )
        evaluate_single_image(model, image_path, output_path, args.conf)

    else:  # batch mode
        eval_root = training_dir / "evaluation/test_data/atHome24-dataset"
        if not eval_root.exists():
            raise FileNotFoundError(f"Evaluation root not found: {eval_root}")

        output_root = training_dir / f"evaluation/results/{args.model}"
        evaluate_batch(model, eval_root, output_root, args.conf)


if __name__ == "__main__":
    main()
