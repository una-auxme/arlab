"""
Test script for visualizing mask contours.

This is a simple test script. Update the path to point to your YCB dataset.
"""
from pathlib import Path

import cv2
import matplotlib.pyplot as plt

# Paths relative to training/ directory (script is in scripts/ subdirectory)
script_dir = Path(__file__).parent
training_dir = script_dir.parent

# Example path - adjust to your actual YCB dataset location
# This assumes YCB data is in data/raw/ or similar
mask_path = training_dir / "data/raw/ycb/001_chips_can/masks/NP1_0_mask.pbm"

# If the example path doesn't exist, try to find any PBM mask
if not mask_path.exists():
    # Try alternative locations
    alt_paths = [
        training_dir / "data/datasets/YCB_at_home24/../raw/ycb/001_chips_can/masks/NP1_0_mask.pbm",
    ]
    for alt in alt_paths:
        if alt.exists():
            mask_path = alt
            break
    else:
        print(f"⚠️  Mask file not found at {mask_path}")
        print("Please update the path in this script to point to your YCB dataset.")
        exit(1)

m = cv2.imread(str(mask_path), cv2.IMREAD_GRAYSCALE)
if m is None:
    raise RuntimeError(f"Failed to load mask: {mask_path}")

m = 255 - m  # invertieren
cnts, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
out = cv2.cvtColor(m, cv2.COLOR_GRAY2BGR)
cv2.drawContours(out, cnts, -1, (0, 255, 0), 2)
plt.imshow(out[..., ::-1])
plt.show()
