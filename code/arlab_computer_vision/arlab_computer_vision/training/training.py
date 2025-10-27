#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Trainingsskript für YOLO11 (Ultralytics) – Detection ODER Segmentation.

Dieses Skript wurde für das arlab_computer_vision ROS2-Package angepasst.

Beispiel:
from arlab_computer_vision.training import train_yolo

# Segmentation
train_yolo(
    task="segment",
    data_path="/path/to/ycb_dataset/data.yaml",
    epochs=100,
    imgsz=640,
    batch=16,
    model_size="n",
    project="runs_ycb",
    name="yolo11n_seg_ycb"
)
"""

import os
import sys
from pathlib import Path
from typing import Optional

import torch
from ultralytics import YOLO

# ============================================================================
# FESTE PFADE - Hier können Sie die Pfade anpassen
# ============================================================================

# Pfad zu den Modell-Gewichten (Pre-trained Weights)
# Bitte ändern Sie diesen Pfad zu Ihrem gewünschten Verzeichnis
base_path = Path(__file__).parent.parent.parent
WEIGHTS_DIR = base_path / "yolo_weights"

# Pfad für die Training-Outputs (wo die trainierten Modelle gespeichert werden)
# Bitte ändern Sie diesen Pfad zu Ihrem gewünschten Verzeichnis
OUTPUT_PROJECT_DIR = base_path / "runs"


def get_package_root() -> Path:
    """Gibt den festen Output-Pfad zurück."""
    return OUTPUT_PROJECT_DIR


def build_default_weights(task: str, size: str, package_root: Path) -> str:
    """Erstelle Standardgewichtspfad basierend auf Task und Größe.

    Args:
        task: 'segment' oder 'detect'
        size: 'n', 's', 'm', 'l', 'x'
        package_root: Wird hier nicht verwendet, aber für API-Kompatibilität

    Returns:
        Pfad zu den Gewichten
    """
    size = size.lower()

    # Nutze den festen WEIGHTS_DIR
    if WEIGHTS_DIR.exists():
        if task == "segment":
            weight_file = WEIGHTS_DIR / f"yolo11{size}-seg.pt"
            if weight_file.exists():
                return str(weight_file)
        elif task == "detect":
            weight_file = WEIGHTS_DIR / f"yolo11{size}.pt"
            if weight_file.exists():
                return str(weight_file)

    # Fallback auf Ultralytics-Download
    if task == "segment":
        return f"yolo11{size}-seg.pt"
    elif task == "detect":
        return f"yolo11{size}.pt"
    else:
        raise ValueError("task muss 'segment' oder 'detect' sein")


def ensure_yaml(data_yaml: str, names_csv: str | None):
    """Optional: Wenn die YAML fehlt, eine Minimal-YAML erzeugen.

    Erwartet Ordnerstruktur im YOLO-Format:
      images/train, images/val, images/test (optional)
    """
    p = Path(data_yaml)
    if p.exists():
        return
    if names_csv is None:
        msg = (
            f"{data_yaml} existiert nicht. "
            "Entweder YAML angeben oder --names 'class1,class2,...' nutzen."
        )
        raise FileNotFoundError(msg)
    root = p.parent
    # Heuristik: Ordner relativ zur YAML
    images_train = "images/train"
    images_val = "images/val"
    images_test = "images/test"
    names = [n.strip() for n in names_csv.split(",") if n.strip()]
    text = [
        f"path: {root.resolve()}",
        f"train: {images_train}",
        f"val: {images_val}",
        f"test: {images_test}",
        "",
        "names:",
    ]
    for i, n in enumerate(names):
        text.append(f"  {i}: {n}")
    p.write_text("\n".join(text), encoding="utf-8")
    print(f"[INFO] YAML erzeugt: {p}")


def maybe_convert_masks_to_yolo_seg(masks_dir: str, images_dir: str, labels_out: str):
    """Optional: Instanzmasken -> YOLO-Seg-Polygone (.txt)

    Erwartet binäre/instanzweise Masken je Bild (*_0.png, *_1.png, ...).
    """
    from ultralytics.data.converter import convert_segment_masks_to_yolo_seg

    os.makedirs(labels_out, exist_ok=True)
    convert_segment_masks_to_yolo_seg(
        masks_dir=masks_dir, output_dir=labels_out, images_dir=images_dir
    )
    print(f"[INFO] Masken -> YOLO-Seg Labels konvertiert nach: {labels_out}")


def detect_device():
    """Automatische GPU-Erkennung."""
    try:
        import torch  # noqa: F401

        if torch.cuda.is_available():
            device = "0"  # Erste GPU
            print(f"[INFO] GPU gefunden: {torch.cuda.get_device_name(0)}")
            return device
        else:
            print("[INFO] Keine GPU gefunden, verwende CPU.")
            return "cpu"
    except ImportError:
        print("[INFO] PyTorch nicht installiert, verwende CPU.")
        return "cpu"


def train_yolo(
    task: str = "segment",
    data_path: str = "",
    epochs: int = 100,
    imgsz: int = 640,
    batch: int = 16,
    model_size: str = "n",
    project: str = "runs_ycb",
    name: Optional[str] = None,
    weights: Optional[str] = None,
    device: Optional[str] = None,
    workers: int = 8,
    patience: int = 20,
    resume: bool = False,
    seed: int = 0,
    cos_lr: bool = False,
    lr0: float = 0.01,
    freeze: int = 0,
    **kwargs,
):
    """Trainiere ein YOLO11-Modell.

    Args:
        task: 'segment' oder 'detect'
        data_path: Pfad zur data.yaml
        epochs: Anzahl der Epochen
        imgsz: Bildgröße
        batch: Batch-Größe
        model_size: Modellgröße ('n', 's', 'm', 'l', 'x')
        project: Projektordner
        name: Run-Name
        weights: Pfad zu vorab trainierten Gewichten
        device: Device ('0', 'cpu', etc.)
        workers: Anzahl Worker-Threads
        patience: Early stopping patience
        resume: Training fortsetzen
        seed: Random seed
        cos_lr: Cosine LR Scheduler
        lr0: Initiale Lernrate
        freeze: Anzahl Layer einfrieren
        **kwargs: Weitere Ultralytics-Parameter

    Returns:
        Training-Ergebnisse
    """
    package_root = get_package_root()

    # Default-Gewichte mit Package-Integration
    weights = weights or build_default_weights(task, model_size, package_root)
    print(f"[INFO] Verwende Gewichte: {weights}")

    # Device-Automatik
    device = device or detect_device()
    print(f"[INFO] Device: {device}")

    # Modell laden
    print(f"[INFO] Lade Modell: {weights}")
    model = YOLO(weights)

    # Train-Task zusammensetzen
    train_kwargs = dict(
        data=data_path,
        epochs=epochs,
        imgsz=imgsz,
        batch=batch,
        device=device,
        project=str(OUTPUT_PROJECT_DIR),
        name=name,
        workers=workers,
        patience=patience,
        resume=resume,
        seed=seed,
        cos_lr=cos_lr,
        lr0=lr0,
        freeze=freeze,
        # Sinnvolle Augment-Defaults für arlab
        hsv_h=0.015,
        hsv_s=0.7,
        hsv_v=0.4,
        mosaic=1.0,
        # Weitere sinnvolle Einstellungen
        dropout=0.0,
        amp=True,  # Mixed precision training
        **kwargs,
    )

    # Train starten
    print("\n" + "=" * 60)
    print("[INFO] Starte Training:")
    print(f"  Task:        {task}")
    print(f"  Gewichte:    {weights}")
    print(f"  Daten:       {data_path}")
    print(f"  Model-Size:  {model_size}")
    print(f"  Device:      {device}")
    print(f"  Epochs:      {epochs}")
    print(f"  Image-Size:  {imgsz}")
    print(f"  Batch-Size:  {batch}")
    print("=" * 60 + "\n")

    results = model.train(**train_kwargs)

    # Val (optional, meist im Training bereits enthalten)
    print("\n[INFO] Finale Evaluierung...")
    model.val(data=data_path, imgsz=imgsz, device=device)

    print("\n[INFO] Bestes Modell:")
    if hasattr(results, "best") and results.best is not None:
        print(f"  Pfad: {results.best}")

    # Export (ONNX als Beispiel)
    if results.best:
        print("\n[INFO] Exportiere bestes Modell nach ONNX...")
        try:
            model_best = YOLO(str(results.best))
            model_best.export(format="onnx", opset=12, simplify=True)
            print("[OK] ONNX-Export erfolgreich.")
        except Exception as e:
            print(f"[WARN] ONNX-Export fehlgeschlagen: {e}")

    print(
        f"\n[DONE] Training abgeschlossen. "
        f"Siehe: {OUTPUT_PROJECT_DIR}/{name or 'train'}"
    )

    return results


# Für direkten Aufruf als Skript
if __name__ == "__main__":
    # Beispiel-Aufruf
    import sys

    if len(sys.argv) < 2:
        print("Benutze: train_yolo(data_path='/path/to/data.yaml', ...)")
        print("\nBeispiel:")
        print("train_yolo(")
        print("    task='segment',")
        print("    data_path='/path/to/data.yaml',")
        print("    epochs=100,")
        print("    imgsz=640,")
        print("    batch=16")
        print(")")
        sys.exit(1)

    # Für einfache Nutzung: Pfad zur data.yaml als erstes Argument
    data_path = sys.argv[1]

    train_yolo(
        task=sys.argv[2] if len(sys.argv) > 2 else "segment",
        data_path=data_path,
        epochs=int(sys.argv[3]) if len(sys.argv) > 3 else 100,
        imgsz=int(sys.argv[4]) if len(sys.argv) > 4 else 640,
        batch=int(sys.argv[5]) if len(sys.argv) > 5 else 16,
    )
