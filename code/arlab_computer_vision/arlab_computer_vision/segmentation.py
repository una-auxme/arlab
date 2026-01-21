from pathlib import Path
import yaml
import cv2
import numpy as np

# ================== ANPASSEN ==================
yaml_path = Path("/workspace/src/arlab/code/arlab_computer_vision/arlab_computer_vision/trainingYCB/dataset/data.yaml")
img_dir   = Path("/workspace/src/arlab/code/arlab_computer_vision/arlab_computer_vision/trainingYCB/dataset/images/train")
lbl_dir   = Path("/workspace/src/arlab/code/arlab_computer_vision/arlab_computer_vision/trainingYCB/dataset/labels/train")

out_dir = Path("label_viz_sampled")
out_dir.mkdir(exist_ok=True)

EVERY_N = 25   # 👈 jedes 100. Bild
MAX_IMAGES = 500  # 👈 Sicherheitslimit (optional)
# ==============================================

def load_names(yaml_path: Path):
    data = yaml.safe_load(yaml_path.read_text())
    names = data["names"]
    nc = data.get("nc", len(names))
    assert len(names) == nc, f"len(names)={len(names)} passt nicht zu nc={nc}"
    return names

def parse_yolo_seg(line):
    parts = line.strip().split()
    cls = int(parts[0])
    pts = np.array(list(map(float, parts[1:])), dtype=np.float32).reshape(-1, 2)
    return cls, pts

def main():
    names = load_names(yaml_path)

    images = sorted([p for p in img_dir.iterdir()
                     if p.suffix.lower() in [".jpg", ".png", ".jpeg"]])

    print(f"📸 Gefundene Bilder: {len(images)}")
    print(f"🔎 Visualisiere jedes {EVERY_N}. Bild")

    saved = 0
    for i, img_path in enumerate(images):
        if i % EVERY_N != 0:
            continue
        if saved >= MAX_IMAGES:
            break

        lbl_path = lbl_dir / (img_path.stem + ".txt")
        img = cv2.imread(str(img_path))
        if img is None:
            continue

        h, w = img.shape[:2]
        overlay = img.copy()

        if lbl_path.exists():
            for line in lbl_path.read_text().splitlines():
                if not line.strip():
                    continue

                cls, pts_norm = parse_yolo_seg(line)

                pts = pts_norm.copy()
                pts[:, 0] *= w
                pts[:, 1] *= h
                pts = pts.astype(np.int32)

                # Maske
                cv2.fillPoly(overlay, [pts], (0, 255, 0))
                cv2.polylines(img, [pts], True, (0, 255, 0), 2)

                name = names[cls] if 0 <= cls < len(names) else f"CLS_{cls}"
                x, y = int(pts[0, 0]), int(pts[0, 1])
                cv2.putText(
                    img,
                    f"{cls}: {name}",
                    (x, max(20, y)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 0, 255),
                    2
                )

            img = cv2.addWeighted(overlay, 0.35, img, 0.65, 0)

        out_path = out_dir / img_path.name
        cv2.imwrite(str(out_path), img)
        saved += 1

    print(f"✅ Fertig. {saved} Bilder gespeichert in: {out_dir.resolve()}")

if __name__ == "__main__":
    main()