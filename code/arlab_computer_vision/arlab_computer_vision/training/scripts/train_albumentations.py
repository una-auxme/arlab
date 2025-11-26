from pathlib import Path
import shutil
from datetime import datetime
from ultralytics import YOLO

# Projekt-Root (training/)
project_root = Path(__file__).resolve().parent.parent

# Pfad zu den ursprünglichen YOLO-Gewichten
weights_path = project_root / "yolo_weights" / "yolo11n-seg.pt"

# Pfad zum YCB-Dataset
data_path = project_root / "data" / "datasets" / "YCB_converted" / "data.yaml"

# Lade YOLO-Modell
model = YOLO(str(weights_path))

# ---------------------------
# Beispiel Albumentations-Augmentation
# ---------------------------
import albumentations as A
ycb_aug = [
    A.RandomBrightnessContrast(brightness_limit=0.25, contrast_limit=0.25, p=0.7),
    A.HueSaturationValue(hue_shift_limit=8, sat_shift_limit=25, val_shift_limit=25, p=0.5),
    A.Rotate(limit=25, border_mode=0, p=0.7),
    A.GaussianBlur(blur_limit=3, p=0.25),
    A.RandomGamma(gamma_limit=(80, 120), p=0.35),
    A.CoarseDropout(max_holes=1, max_height=25, max_width=25, p=0.25),
]

# ---------------------------
# Training
# ---------------------------
results = model.train(
    data=str(data_path),
    epochs=100,
    imgsz=640,
    batch=16,
    device=0,   # GPU verwenden, sonst -1 für CPU
    augmentations=ycb_aug,
)

# ---------------------------
# Speichern mit Versionierung
# ---------------------------
best = Path(results.save_dir) / "weights" / "best.pt"
if best.exists():
    # sicherstellen, dass yolo_weights Ordner existiert
    (project_root / "yolo_weights").mkdir(exist_ok=True)
    
    now = datetime.now().strftime("%Y%m%d-%H%M%S")
    output_path = project_root / "yolo_weights" / f"yolo11n-seg-YCB-Albumentations-{now}.pt"
    shutil.copy2(best, output_path)
    print(f"✅ Weights saved to {output_path}")
