import shutil
from pathlib import Path

from ultralytics import YOLO

model = YOLO("../../yolo_weights/yolo11n.pt")
results = model.train(
    data="data/freiburg-groceries/data.yaml", epochs=100, imgsz=640, device=-1
)

# Speichere bestes Modell
best = Path(results.save_dir) / "weights" / "best.pt"
if best.exists():
    shutil.copy2(best, Path("../../yolo_weights/yolo11n-trained.pt"))
    print("✅ Weights saved to yolo_weights/")
