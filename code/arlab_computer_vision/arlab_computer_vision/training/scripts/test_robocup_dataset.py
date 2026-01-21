from ultralytics import YOLO
from pathlib import Path

# --------------------------------------------------
# Pfade anpassen (NUR HIER)
# --------------------------------------------------

# Pfad zu deinem finalen Modell (moderate geometry)
WEIGHTS_PATH = "/workspace/src/arlab/code/arlab_computer_vision/arlab_computer_vision/training/yolo_weights/yolo11n-seg-YCB-strong_geom-20260110-122154.pt"
# 👆 ggf. Dateinamen anpassen

# Pfad zum Test-Ordner (darf Unterordner enthalten)
TESTSET_PATH = "/workspace/src/arlab/code/arlab_computer_vision/arlab_computer_vision/training/evaluation/eval_data/atHome24-dataset"
# 👆 z.B. images/test oder eigener Demo-Ordner

# Ausgabeordner
PROJECT_NAME = "runs/segment"
RUN_NAME = "demo_predict_testset_moderate"

# --------------------------------------------------
# Modell laden
# --------------------------------------------------

print("🔹 Loading model:", WEIGHTS_PATH)
model = YOLO(WEIGHTS_PATH)

# -------------------------------------------------- 
# Prediction auf Testset
# --------------------------------------------------

print("🚀 Running prediction on test set...")
results = model.predict(
    source=TESTSET_PATH,   # Ordner mit (Unter-)Ordnern
    imgsz=640,
    conf=0.4,              # Demo-freundlich (sauber)
    iou=0.5,
    save=True,             # speichert Bilder mit Overlay
    save_conf=True,        # zeigt Confidence im Bild
    save_txt=False,        # keine TXT-Ausgabe nötig
    project=PROJECT_NAME,
    name=RUN_NAME,
    exist_ok=True,
)

# --------------------------------------------------
# Abschluss
# --------------------------------------------------

output_dir = Path(PROJECT_NAME) / RUN_NAME
print("\n✅ Demo finished successfully!")
print("📁 Results saved to:", output_dir.resolve())
