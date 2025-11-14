import shutil
from pathlib import Path

from ultralytics import YOLO

# Paths relative to training/ directory (script is in scripts/ subdirectory)
script_dir = Path(__file__).parent
training_dir = script_dir.parent

model = YOLO(str(training_dir.parent.parent / "yolo_weights/yolo11n-seg.pt"))
results = model.train(
    data=str(training_dir / "data/datasets/YCB_converted/data.yaml"),
    epochs=100,
    imgsz=640,
    device=-1,
)

# Speichere bestes Modell
best = Path(results.save_dir) / "weights" / "best.pt"
if best.exists():
    output_path = training_dir.parent.parent / "yolo_weights/yolo11n-seg-YCB.pt"
    shutil.copy2(best, output_path)
    print(f"✅ Weights saved to {output_path}")
