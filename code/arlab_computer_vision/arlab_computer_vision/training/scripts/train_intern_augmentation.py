# train_ycb_yolo_aug.py
import shutil
from pathlib import Path
from datetime import datetime
from ultralytics import YOLO

# ---------------------------
# Projekt-Root (training/)
# ---------------------------
project_root = Path(__file__).resolve().parent.parent

# Pfad zu den ursprünglichen YOLO-Gewichten
weights_path = project_root / "yolo_weights" / "yolo11n-seg.pt"

# Pfad zum YCB-Dataset
data_path = project_root / "data" / "datasets" / "YCB_converted" / "data.yaml"

# Lade YOLO-Modell
model = YOLO(str(weights_path))

# ---------------------------
# Training mit YOLO-interner Augmentation
# ---------------------------
results = model.train(
    data=str(data_path),
    epochs=100,
    imgsz=640,
    batch=16,
    device=0,   # GPU verwenden, sonst -1 für CPU
    # YOLO interne Augmentationen
    hsv_h=0.03,
    hsv_s=0.6,
    hsv_v=0.5,
    fliplr=0.5,
    translate=0.1,
    scale=0.1,
    mosaic=1.0,
)

# ---------------------------
# Speichern des besten Modells mit Versionierung
# ---------------------------
best = Path(results.save_dir) / "weights" / "best.pt"
if best.exists():
    # sicherstellen, dass yolo_weights Ordner existiert
    (project_root / "yolo_weights").mkdir(exist_ok=True)

    now = datetime.now().strftime("%Y%m%d-%H%M%S")
    output_path = project_root / "yolo_weights" / f"yolo11n-seg-YCB-YOLO-{now}.pt"
    shutil.copy2(best, output_path)
    print(f"✅ Weights saved to {output_path}")
