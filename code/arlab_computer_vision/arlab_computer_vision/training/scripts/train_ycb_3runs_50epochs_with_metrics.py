"""
Trainiert 3 Varianten (baseline / moderate_geom / strong_geom) für YCB-Segmentation
- 50 Epochen pro Run (schneller Vergleich)
- ohne Mosaic
- nach jedem Training: best.pt versioniert speichern
- danach: sauberes model.val() mit fixen Parametern (conf/iou) -> fairer Vergleich
- liest Maskenmetriken aus val/results.csv und berechnet F1
- schreibt zusätzlich eine Summary-CSV (1 Zeile pro Run) für Excel/Präsi
"""

import shutil
import time
import csv
from pathlib import Path
from datetime import datetime
from typing import Dict, Optional, Any, List

from ultralytics import YOLO


# ============================================================
# 1) PFADE / SETTINGS (HIER passt du nur DATA_YAML an)
# ============================================================

# Projekt-Root: angenommen Script liegt in project_root/training/...
project_root = Path(__file__).resolve().parent.parent

# Pretrained weights (Segmentation)
WEIGHTS_PATH = project_root / "yolo_weights" / "yolo11n-seg.pt"

# Dataset liegt bei dir außerhalb -> absoluter Pfad zur data.yaml:
DATA_YAML = Path("/workspace/src/arlab/code/arlab_computer_vision/arlab_computer_vision/trainingYCB/dataset/data.yaml")  # <-- HIER ANPASSEN

# Trainingseinstellungen
EPOCHS = 500
IMGSZ = 640
BATCH = 16          # RTX 3080: sicher. Wenn OOM: 4
DEVICE = 0         # GPU. CPU wäre -1
PATIENCE = 30

# ============================================================
# 2) VAL-SETTINGS (wichtig: fix für alle Runs!)
# ============================================================
# Damit Precision/Recall/F1 fair vergleichbar sind, müssen conf/iou fix sein.
VAL_ARGS = dict(
    data=str(DATA_YAML),
    imgsz=IMGSZ,
    batch=BATCH,
    device=DEVICE,
    conf=0.25,      # feste Conf-Schwelle (für P/R/F1 Vergleich!)
    iou=0.50,       # feste IoU-Schwelle
    split="val",
)

# ============================================================
# 3) 3 EXPERIMENTE (ohne mosaic)
# ============================================================
# Idee: nur GEOMETRIE variiert. Farbe bleibt realistisch aktiv.
EXPERIMENTS = [
    {
        "name": "baseline",
        "params": dict(
            # Farb- / Beleuchtungsvariation
            hsv_h=0.03, hsv_s=0.6, hsv_v=0.5,
            # leichte Geometrie
            fliplr=0.5,
            translate=0.1,
            scale=0.1,
            # mosaic bewusst NICHT gesetzt
        ),
    },
    {
        "name": "moderate_geom",
        "params": dict(
            hsv_h=0.02, hsv_s=0.4, hsv_v=0.4,
            fliplr=0.5,
            flipud=0.1,
            translate=0.15,
            scale=0.2,
            degrees=10.0,
        ),
    },
    {
        "name": "strong_geom",
        "params": dict(
            hsv_h=0.02, hsv_s=0.4, hsv_v=0.4,
            fliplr=0.5,
            flipud=0.2,
            translate=0.2,
            scale=0.3,
            degrees=15.0,
            shear=5.0,
            perspective=0.0005,
        ),
    },
]


# ============================================================
# 4) HILFSFUNKTIONEN
# ============================================================
def format_time(seconds: float) -> str:
    """Zeit in h/m/s formatieren."""
    seconds = int(seconds)
    h = seconds // 3600
    m = (seconds % 3600) // 60
    s = seconds % 60
    if h > 0:
        return f"{h}h {m}m {s}s"
    if m > 0:
        return f"{m}m {s}s"
    return f"{s}s"


def safe_float(x: Any) -> Optional[float]:
    """Robust float-Parsing."""
    try:
        return float(x)
    except Exception:
        return None


def compute_f1(p: Optional[float], r: Optional[float]) -> Optional[float]:
    """F1 aus Precision und Recall."""
    if p is None or r is None:
        return None
    if p + r == 0:
        return 0.0
    return 2.0 * p * r / (p + r)


def find_col(keys: List[str], candidates: List[str]) -> Optional[str]:
    """
    Findet eine Spalte in results.csv.
    Ultralytics-Spaltennamen können je nach Version leicht variieren.
    Deshalb matchen wir per substring (case-insensitive).
    """
    for cand in candidates:
        cl = cand.lower()
        for k in keys:
            if cl in k.lower():
                return k
    return None


def read_val_metrics(results_csv: Path) -> Dict[str, Optional[float]]:
    """
    Liest results.csv aus dem val()-Run und extrahiert Maskenmetriken:
    - precision(M), recall(M)
    - segm mAP50, segm mAP50-95
    und berechnet F1.
    """
    if not results_csv.exists():
        return {}

    with results_csv.open("r", newline="") as f:
        reader = csv.DictReader(f)
        rows = list(reader)

    if not rows or reader.fieldnames is None:
        return {}

    keys = list(reader.fieldnames)

    # Kandidatenlisten (Masken zuerst, dann fallback)
    col_p = find_col(keys, ["metrics/precision(m)", "metrics/precision"])
    col_r = find_col(keys, ["metrics/recall(m)", "metrics/recall"])
    col_m50 = find_col(keys, ["metrics/segm_map50", "metrics/segm_mAP50", "metrics/map50"])
    col_m5095 = find_col(keys, ["metrics/segm_map50-95", "metrics/segm_mAP50-95", "metrics/map50-95"])

    def best(col: Optional[str]) -> Optional[float]:
        if col is None:
            return None
        vals = []
        for r in rows:
            v = safe_float(r.get(col))
            if v is not None:
                vals.append(v)
        return max(vals) if vals else None

    p = best(col_p)
    r = best(col_r)

    return {
        "precision": p,
        "recall": r,
        "f1": compute_f1(p, r),
        "mAP50": best(col_m50),
        "mAP50-95": best(col_m5095),
        "_cols_used": {"precision": col_p, "recall": col_r, "mAP50": col_m50, "mAP50-95": col_m5095},
    }


# ============================================================
# 5) EIN RUN: TRAIN -> best.pt sichern -> VAL -> Metriken lesen
# ============================================================
def run_experiment(idx: int, exp: Dict) -> Dict:
    name = exp["name"]
    params = exp["params"]

    print("\n" + "=" * 90)
    print(f"▶ Training {idx + 1}/{len(EXPERIMENTS)}: {name}")
    print("=" * 90)

    t0 = time.time()

    # Modell frisch laden (kein Weitertrainieren vom vorherigen Run)
    model = YOLO(str(WEIGHTS_PATH))

    # Train-Args: Basis + Augmentationsparameter
    train_args = dict(
        data=str(DATA_YAML),
        epochs=EPOCHS,
        patience=PATIENCE,
        imgsz=IMGSZ,
        batch=BATCH,
        device=DEVICE,
        name=name,  # -> runs/segment/train/<name>
    )
    train_args.update(params)

    # 1) TRAIN
    results = model.train(**train_args)
    train_dir = Path(results.save_dir)

    # 2) best.pt finden
    best_pt = train_dir / "weights" / "best.pt"
    if not best_pt.exists():
        raise FileNotFoundError(f"best.pt nicht gefunden: {best_pt}")

    # 3) best.pt versioniert kopieren
    out_dir = project_root / "yolo_weights"
    out_dir.mkdir(exist_ok=True)

    ts = datetime.now().strftime("%Y%m%d-%H%M%S")
    exported = out_dir / f"yolo11n-seg-YCB-{name}-{ts}.pt"
    shutil.copy2(best_pt, exported)
    print(f"✅ best.pt exportiert nach: {exported}")

    # 4) VAL (fair: gleiche conf/iou/imgsz/batch für ALLE Runs)
    print("🔍 Validation (fixed conf/iou) ...")
    val_model = YOLO(str(best_pt))
    val_results = val_model.val(**VAL_ARGS, name=f"VAL_{name}")
    val_dir = Path(val_results.save_dir)

    # 5) Metriken aus val/results.csv lesen
    metrics = read_val_metrics(val_dir / "results.csv")

    dt = time.time() - t0
    print(f"⏱ Dauer Run '{name}': {format_time(dt)}")

    if metrics and metrics.get("precision") is not None:
        print(
            f"📊 Val(M): P={metrics.get('precision'):.3f}  "
            f"R={metrics.get('recall'):.3f}  "
            f"F1={metrics.get('f1'):.3f}  "
            f"mAP50={metrics.get('mAP50'):.3f}  "
            f"mAP50-95={metrics.get('mAP50-95'):.3f}"
        )
    else:
        print("⚠️ Konnte keine Metriken aus results.csv lesen (Spaltennamen evtl. anders).")

    return {
        "name": name,
        "train_dir": train_dir,
        "val_dir": val_dir,
        "best_pt": best_pt,
        "exported": exported,
        "metrics": metrics,
        "duration_s": dt,
    }


# ============================================================
# 6) MAIN: 3 Runs nacheinander + Summary CSV + bestes Modell
# ============================================================
def main():
    # Mini-Checks, damit du nicht mit falschen Pfaden losläufst
    print("project_root:", project_root)
    print("WEIGHTS_PATH:", WEIGHTS_PATH, "exists:", WEIGHTS_PATH.exists())
    print("DATA_YAML   :", DATA_YAML, "exists:", DATA_YAML.exists())
    if not WEIGHTS_PATH.exists():
        raise FileNotFoundError(f"Weights nicht gefunden: {WEIGHTS_PATH}")
    if not DATA_YAML.exists():
        raise FileNotFoundError(f"data.yaml nicht gefunden: {DATA_YAML}")

    overall0 = time.time()
    all_results = []

    # 3 Trainingsläufe
    for i, exp in enumerate(EXPERIMENTS):
        all_results.append(run_experiment(i, exp))

        elapsed = time.time() - overall0
        avg = elapsed / (i + 1)
        remaining = avg * (len(EXPERIMENTS) - (i + 1))
        print("-" * 90)
        print(f"⏱ Gesamt bisher: {format_time(elapsed)} | ⏳ Rest (≈): {format_time(remaining)}")
        print("-" * 90)

    # -----------------------
    # Summary CSV (1 Zeile pro Run) -> für Excel/Präsi
    # -----------------------
    out_dir = project_root / "yolo_weights"
    out_dir.mkdir(exist_ok=True)

    summary_csv = out_dir / f"summary_3runs_{EPOCHS}epochs_{datetime.now().strftime('%Y%m%d-%H%M%S')}.csv"
    with summary_csv.open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["experiment", "precision_M", "recall_M", "f1_M", "mAP50_M", "mAP50-95_M", "epochs", "weights_file"])
        for r in all_results:
            m = r.get("metrics", {})
            writer.writerow([
                r["name"],
                m.get("precision"),
                m.get("recall"),
                m.get("f1"),
                m.get("mAP50"),
                m.get("mAP50-95"),
                EPOCHS,
                str(r["exported"]),
            ])

    print(f"\n📄 Summary CSV gespeichert: {summary_csv}")

    # -----------------------
    # Bestes Modell nach Val-F1 auswählen
    # -----------------------
    best_name = None
    best_f1 = float("-inf")
    best_pt = None

    print("\n" + "#" * 90)
    print("📊 SUMMARY (nur val() zählt)")
    print("#" * 90)

    for r in all_results:
        m = r.get("metrics", {})
        f1v = m.get("f1")
        print(f"- {r['name']}: P={m.get('precision')} R={m.get('recall')} F1={m.get('f1')} mAP50={m.get('mAP50')}")

        if isinstance(f1v, float) and f1v > best_f1:
            best_f1 = f1v
            best_name = r["name"]
            best_pt = r["best_pt"]

    if best_pt is not None:
        stable = out_dir / "yolo11n-seg-YCB-BEST-overall.pt"
        shutil.copy2(best_pt, stable)
        print("\n🏆 BEST (nach Val-F1)")
        print(f"   Name: {best_name}")
        print(f"   F1  : {best_f1:.3f}")
        print(f"   Gespeichert als: {stable}")
    else:
        print("\n⚠️ Konnte kein bestes Modell bestimmen (F1 nicht verfügbar).")

    print(f"\n🎉 Fertig. Gesamtzeit: {format_time(time.time() - overall0)}")


if __name__ == "__main__":
    main()
