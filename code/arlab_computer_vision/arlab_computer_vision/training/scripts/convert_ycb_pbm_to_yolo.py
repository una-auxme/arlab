"""
Konvertiert eine YCB-ähnliche Struktur mit JPG-Bildern + PBM-Masken
in YOLOv8/11 Segmentation-Format.

Ordnerstruktur (Eingabe):
ycb/
  001_chips_can/
    N1_0.jpg
    N1_1.jpg
    masks/
      N1_0_mask.pbm
      N1_1_mask.pbm
  002_master_chef_can/
    ...

Ausgabe (YOLO-kompatibel):
dataset/
  images/
    train/*.jpg
    val/*.jpg
  labels/
    train/*.txt
    val/*.txt
  data.yaml
"""

import argparse
import math
import random
import shutil
from collections import defaultdict
from pathlib import Path

import cv2
import numpy as np

# ---------- Funktionen ----------


def load_mask_binary(p: Path, invert="yes"):
    """Liest PBM-Maske und invertiert sie (weil Objekt schwarz, Hintergrund weiß)."""
    m = cv2.imread(str(p), cv2.IMREAD_GRAYSCALE)
    if m is None:
        raise RuntimeError(f"Maske nicht lesbar: {p}")
    _, m = cv2.threshold(m, 127, 255, cv2.THRESH_BINARY)
    if invert in ("yes", True):
        m = 255 - m
    return m


def find_contours(mask_bin, min_area=20):
    """Findet Außenkonturen in der Maske."""
    cnts, _ = cv2.findContours(mask_bin, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    return [c for c in cnts if cv2.contourArea(c) >= min_area]


def simplify(cnt, eps_frac):
    """Vereinfacht Polygonkonturen."""
    eps = eps_frac * cv2.arcLength(cnt, True)
    return cv2.approxPolyDP(cnt, eps, True) if eps > 0 else cnt


def write_yolo_seg(label_path: Path, contours, w, h, class_id, eps_frac):
    """Schreibt eine YOLO-Segmentation-Labeldatei."""
    lines = []
    for c in contours:
        c = simplify(c, eps_frac)
        if len(c) < 3:
            continue
        pts = c.reshape(-1, 2).astype(np.float32)
        pts[:, 0] /= w
        pts[:, 1] /= h
        pts = np.clip(pts, 0.0, 1.0)
        coords = " ".join(f"{x:.6f} {y:.6f}" for x, y in pts)
        lines.append(f"{class_id} {coords}")
    label_path.parent.mkdir(parents=True, exist_ok=True)
    label_path.write_text("\n".join(lines) if lines else "", encoding="utf-8")


def list_classes(root: Path):
    """Liest alle Klassenordner im ycb-root."""
    return sorted([d.name for d in root.iterdir() if d.is_dir()])


def iter_image_mask_pairs(class_dir: Path):
    """
    Liefert (Bild, Maske)-Paare aus einem Klassenordner.
    Erwartet: masken liegen in class_dir/masks/<bildname>_mask.pbm
    """
    masks_dir = class_dir / "masks"
    if not masks_dir.exists():
        return
    img_exts = {".jpg", ".jpeg", ".png"}
    for img in class_dir.iterdir():
        if not img.is_file() or img.suffix.lower() not in img_exts:
            continue
        mask = masks_dir / f"{img.stem}_mask.pbm"
        if mask.exists():
            yield img, mask
        else:
            print(f"[WARN] Keine Maske für {img.name} in {masks_dir}")


def stratified_split(items, labels, train_ratio=0.8, seed=0):
    """Teilt die Daten stratifiziert (jede Klasse bleibt proportional vertreten)."""
    rng = random.Random(seed)
    by_cls = defaultdict(list)
    for it, lb in zip(items, labels):
        by_cls[lb].append(it)
    train, val = [], []
    for lb, arr in by_cls.items():
        rng.shuffle(arr)
        n_tr = int(math.floor(len(arr) * train_ratio))
        train += arr[:n_tr]
        val += arr[n_tr:]
    rng.shuffle(train)
    rng.shuffle(val)
    return train, val


# ---------- Hauptprogramm ----------


def main():
    ap = argparse.ArgumentParser(
        description="JPG + PBM (schwarzes Objekt) → YOLO Segmentation"
    )
    ap.add_argument(
        "--ycb-root",
        required=True,
        type=Path,
        help="Root-Ordner mit Klassenordnern (wie 001_chips_can/)",
    )
    ap.add_argument("--out", required=True, type=Path, help="Zielordner (dataset)")
    ap.add_argument(
        "--approx",
        type=float,
        default=0.002,
        help="Polygon-Vereinfachung (Anteil Umfang)",
    )
    ap.add_argument(
        "--min-area", type=int, default=20, help="Minimale Konturfläche in Pixeln"
    )
    ap.add_argument(
        "--train-ratio", type=float, default=0.8, help="Train/Val-Verhältnis"
    )
    ap.add_argument("--seed", type=int, default=0, help="Zufallssaat für Split")
    args = ap.parse_args()

    classes = list_classes(args.ycb_root)
    if not classes:
        raise SystemExit("Keine Klassenordner gefunden.")

    name2id = {name: i for i, name in enumerate(classes)}

    out_images = args.out / "images"
    out_labels = args.out / "labels"
    for sub in ["train", "val"]:
        (out_images / sub).mkdir(parents=True, exist_ok=True)
        (out_labels / sub).mkdir(parents=True, exist_ok=True)

    # Sammle alle (Bild,Maske)-Paare
    pairs, cls_labels = [], []
    for cname in classes:
        cdir = args.ycb_root / cname
        for img, msk in iter_image_mask_pairs(cdir):
            pairs.append((cname, img, msk))
            cls_labels.append(name2id[cname])
    if not pairs:
        raise SystemExit("Keine Bild/Masken-Paare gefunden (prüfe *_mask.pbm).")

    train_items, val_items = stratified_split(
        pairs, cls_labels, args.train_ratio, args.seed
    )

    def process(items, subset):
        for cname, img, msk in items:
            cls_id = name2id[cname]
            # Bild einfach kopieren
            dst_img = out_images / subset / f"{cname}_{img.name}"
            shutil.copy2(img, dst_img)

            # Maske laden → invertieren → Konturen finden → Label schreiben
            m = load_mask_binary(msk, invert="yes")
            h, w = m.shape[:2]
            cnts = find_contours(m, args.min_area)

            dst_lbl = out_labels / subset / f"{cname}_{img.stem}.txt"
            write_yolo_seg(dst_lbl, cnts, w, h, cls_id, args.approx)

    process(train_items, "train")
    process(val_items, "val")

    # data.yaml schreiben
    yaml_text = "\n".join(
        [
            f"path: {args.out}",
            "train: images/train",
            "val: images/val",
            "names: [" + ", ".join(classes) + "]",
        ]
    )
    (args.out / "data.yaml").write_text(yaml_text, encoding="utf-8")

    print(
        f"Fertig. train={len(train_items)} | val={len(val_items)} | total={len(pairs)}"
    )
    print(f"data.yaml → {args.out / 'data.yaml'}")
    print("Klassen (index:name):")
    for i, n in enumerate(classes):
        print(f"  {i}: {n}")


if __name__ == "__main__":
    main()
