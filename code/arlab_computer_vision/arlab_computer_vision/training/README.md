# Training Directory

This directory contains all scripts, data, and results related to YOLO model training and evaluation.

## Directory Structure

```
training/
├── scripts/                    # Training and evaluation scripts
│   ├── train.py               # Main training script
│   ├── evaluate.py            # Model evaluation (single image or batch)
│   ├── convert_ycb_pbm_to_yolo.py  # Convert YCB PBM masks to YOLO format
│   └── test_mask_visualization.py   # Test script for mask visualization
│
├── data/                       # Training data
│   ├── datasets/              # Active datasets (YOLO format)
│   │   ├── freiburg-groceries/
│   │   ├── Robocup-Home-24-dataset/
│   │   ├── YCB_at_home24/
│   │   └── YCB_converted/
│   ├── raw/                   # Original ZIP archives and README files
│   └── test_images/           # Test images (e.g., ycb_banner.jpg)
│
├── evaluation/                # Model evaluation results
│   ├── test_data/             # Test images for evaluation
│   │   └── atHome24-dataset/
│   └── results/               # Model predictions organized by model
│       ├── yolo11n-seg/
│       ├── yolo11n-seg-trained-atHome24/
│       ├── yolo11n-seg-trained-freiburg/
│       ├── yolo11n-seg-Robocup-Home-24-dataset/
│       └── yolo11n-seg-YCB/
│
└── runs/                      # YOLO training outputs (auto-generated)
    ├── detect/
    └── segment/
```

## Scripts

### Training

**`scripts/train.py`**
- Trains a YOLO segmentation model
- Edit the script to change model, dataset, epochs, etc.
- Saves best model to `../../yolo_weights/`

Example:
```bash
cd training
python scripts/train.py
```

### Evaluation

**`scripts/evaluate.py`**
- Evaluates a trained model on images
- Supports single image or batch mode

Single image (banner):
```bash
python scripts/evaluate.py --model yolo11n-seg-YCB --mode single
```

Batch evaluation (all test images):
```bash
python scripts/evaluate.py --model yolo11n-seg-YCB --mode batch
```

With custom image:
```bash
python scripts/evaluate.py --model yolo11n-seg-YCB --mode single --image path/to/image.jpg
```

### Data Conversion

**`scripts/convert_ycb_pbm_to_yolo.py`**
- Converts YCB dataset format (JPG + PBM masks) to YOLO segmentation format

Usage:
```bash
python scripts/convert_ycb_pbm_to_yolo.py \
  --ycb-root /path/to/ycb/data \
  --out /path/to/output/dataset \
  --train-ratio 0.8
```

## Datasets

### Available Datasets

1. **freiburg-groceries**: Grocery items dataset
2. **Robocup-Home-24-dataset**: RoboCup home dataset
3. **YCB_at_home24**: YCB objects dataset
4. **YCB_converted**: Converted YCB dataset

Each dataset contains:
- `data.yaml`: Dataset configuration
- `train/`: Training images and labels
- `val/` or `valid/`: Validation images and labels

## Trained Models

Models are saved to `../../yolo_weights/` after training:

- `yolo11n-seg-YCB.pt`: Trained on YCB_converted dataset
- `yolo11n-seg-trained-atHome24.pt`: Trained on atHome24 dataset
- `yolo11n-seg-trained-freiburg.pt`: Trained on freiburg-groceries
- `yolo11n-seg-Robocup-Home-24-dataset.pt`: Trained on Robocup dataset

## Evaluation Results

Evaluation results are stored in `evaluation/results/<model_name>/`:
- Predictions for each test image
- Banner prediction images (`*_prediction.png`)

## Notes

- Training outputs (metrics, plots, weights) are automatically saved to `runs/segment/` by YOLO
- Test images are in `evaluation/test_data/`
- Model weights are stored in `../../yolo_weights/` (parent directory)

