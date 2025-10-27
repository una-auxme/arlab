# YOLO Training Dataset Struktur

## Verzeichnisstruktur

```
training/data/
├── data.yaml              # Konfigurationsdatei (WICHTIG: Hier anpassen!)
├── train/
│   ├── images/            # Trainingsbilder (.jpg, .png, etc.)
│   └── labels/            # Trainingslabels (.txt im YOLO-Format)
└── validation/
    ├── images/            # Validierungsbilder
    └── labels/            # Validierungslabels

# Optional:
└── test/
    ├── images/
    └── labels/
```

## Label-Format (YOLO)

Für **Detection** - Datei: `train/labels/IMG_001.txt`

```txt
# Format: class_id center_x center_y width height
# Alle Werte normalisiert (0.0 - 1.0)
# Beispiel: 2 Objekte im Bild (Klasse 0 und Klasse 1)

0 0.5 0.5 0.2 0.3    # Klasse 0: zentriert, 20% Breite, 30% Höhe
1 0.8 0.2 0.1 0.15   # Klasse 1: rechts oben, 10% Breite, 15% Höhe
```

Für **Segmentation** - Datei: `train/labels/IMG_001.txt`

```txt
# Format: class_id x1 y1 x2 y2 x3 y3 ... (Polygon-Koordinaten)
# Alle Werte normalisiert (0.0 - 1.0)
# Polygon beschreibt die Objektkontur

0 0.4 0.4 0.5 0.4 0.5 0.6 0.4 0.6    # Klasse 0: Polygon mit 4 Punkten
1 0.7 0.1 0.9 0.1 0.9 0.3 0.7 0.3    # Klasse 1: Rechteck (4 Ecken)
```

## Beispiel: data.yaml

Siehe `example_data.yaml` für ein vollständiges Beispiel.

Kurzes Beispiel für 2 Klassen:

```yaml
path: .
train: train/images
val: validation/images

nc: 2
names:
  0: object_1
  1: object_2
```

## Wie die Pfade funktionieren

1. **`path`**: Basis-Verzeichnis (meist `.` für aktuelles Verzeichnis)
2. **`train`**: Relativer Pfad zu Trainingsbildern (`path` + `train`)
3. **`val`**: Relativer Pfad zu Validierungsbildern (`path` + `val`)
4. **`nc`**: Anzahl der Klassen
5. **`names`**: Mapping von Klassen-ID → Klassenname

## Training starten

```python
from arlab_computer_vision.training import train_yolo

train_yolo(
    task="segment",  # oder "detect"
    data_path="training/data/data.yaml",
    epochs=100,
    imgsz=640,
    batch=16
)
```

## Tipps

1. **Bildtitel sollten eindeutig sein** (z.B. IMG_001.jpg)
2. **Jedes Bild braucht ein Label-File** mit gleichem Namen
3. **Labels sind normalisiert** (0.0 bis 1.0, nicht in Pixeln!)
4. **Mindestens 100 Bilder pro Klasse** empfohlen
