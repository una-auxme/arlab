#!/usr/bin/env python3
"""Convert PBM masks to YOLO format."""

import argparse
import shutil
from pathlib import Path

import cv2
import numpy as np
from sklearn.model_selection import train_test_split


def convert_pbm_to_yolo_txt(pbm_path, txt_output_path, class_id=0):
    """Convert PBM mask to YOLO format."""
    mask = cv2.imread(str(pbm_path), cv2.IMREAD_GRAYSCALE)
    if mask is None:
        return False

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if len(contours) == 0:
        Path(txt_output_path).touch()
        return True

    h, w = mask.shape[:2]
    Path(txt_output_path).parent.mkdir(parents=True, exist_ok=True)

    with open(txt_output_path, "w") as f:
        for contour in contours:
            polygon = contour.reshape(-1, 2)
            normalized = [f"{pt[0] / w:.6f} {pt[1] / h:.6f}" for pt in polygon]
            f.write(f"{class_id} {' '.join(normalized)}\n")

    return True


def get_class_id(filename):
    """Extract class ID from filename (NP1 -> 0, NP2 -> 1, ...)."""
    return int(filename[2]) - 1


def process_dataset(source_dir="001_chips_can", output_dir="data", test_size=0.2):
    """Process dataset: convert masks and organize for YOLO."""
    source, output = Path(source_dir), Path(output_dir)

    # Create directory structure
    for dir_ in [
        output / "train" / "images",
        output / "train" / "labels",
        output / "validation" / "images",
        output / "validation" / "labels",
    ]:
        dir_.mkdir(parents=True, exist_ok=True)

    # Get images and split
    image_files = sorted(source.glob("NP*.jpg"))
    masks_dir = source / "masks"

    if len(image_files) == 0:
        print(f"Error: No images in {source_dir}")
        return

    train_files, val_files = train_test_split(
        image_files, test_size=test_size, random_state=42
    )

    # Process sets
    def process_set(files, img_dir, lbl_dir):
        for img_file in files:
            shutil.copy2(img_file, img_dir / img_file.name)
            convert_pbm_to_yolo_txt(
                masks_dir / f"{img_file.stem}_mask.pbm",
                lbl_dir / f"{img_file.stem}.txt",
                get_class_id(img_file.name),
            )

    print(f"Processing {len(train_files)} train + {len(val_files)} val images...")
    process_set(train_files, output / "train" / "images", output / "train" / "labels")
    process_set(
        val_files, output / "validation" / "images", output / "validation" / "labels"
    )
    print(f"✅ Conversion complete! Output: {output_dir}/")


def main():
    parser = argparse.ArgumentParser(description="Convert PBM masks to YOLO format")
    parser.add_argument(
        "--source", type=str, default="001_chips_can", help="Source directory"
    )
    parser.add_argument("--output", type=str, default="data", help="Output directory")
    parser.add_argument("--test-size", type=float, default=0.2, help="Validation size")
    args = parser.parse_args()

    process_dataset(args.source, args.output, args.test_size)


if __name__ == "__main__":
    main()
