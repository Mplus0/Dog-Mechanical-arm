#!/usr/bin/env python3
import argparse
import random
import shutil
from pathlib import Path

import cv2
from ultralytics import YOLO


IMAGE_EXTS = {".jpg", ".jpeg", ".png", ".bmp"}


def yolo_line_from_xyxy(cls_id, x1, y1, x2, y2, w, h):
    x1 = max(0.0, min(float(w - 1), float(x1)))
    y1 = max(0.0, min(float(h - 1), float(y1)))
    x2 = max(0.0, min(float(w - 1), float(x2)))
    y2 = max(0.0, min(float(h - 1), float(y2)))

    if x2 <= x1 or y2 <= y1:
        return None

    xc = ((x1 + x2) / 2.0) / float(w)
    yc = ((y1 + y2) / 2.0) / float(h)
    bw = (x2 - x1) / float(w)
    bh = (y2 - y1) / float(h)

    return f"{cls_id} {xc:.6f} {yc:.6f} {bw:.6f} {bh:.6f}"


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--src", default="/home/sunrise/dog/ros2_red_block_ws/hard_samples")
    parser.add_argument("--out", default="/home/sunrise/dog/ros2_red_block_ws/datasets/red_block_v2")
    parser.add_argument("--model", default="/home/sunrise/dog/ros2_red_block_ws/src/red_block_grasp_ros2/models/red_block_yolo11n.pt")
    parser.add_argument("--conf", type=float, default=0.10)
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--val-ratio", type=float, default=0.2)
    parser.add_argument("--seed", type=int, default=42)
    args = parser.parse_args()

    src = Path(args.src)
    out = Path(args.out)

    images = sorted(
        p for p in src.rglob("*")
        if p.suffix.lower() in IMAGE_EXTS
        and "manual_test" not in p.name
    )

    if not images:
        raise RuntimeError(f"No images found in {src}")

    random.seed(args.seed)
    random.shuffle(images)

    # Clean old dataset
    if out.exists():
        shutil.rmtree(out)

    for sub in ["images/train", "images/val", "labels/train", "labels/val", "preview"]:
        (out / sub).mkdir(parents=True, exist_ok=True)

    val_count = max(1, int(len(images) * args.val_ratio))
    val_set = set(images[:val_count])

    model = YOLO(args.model)

    train_count = 0
    val_count_real = 0
    labeled_count = 0
    empty_count = 0

    for idx, img_path in enumerate(images):
        split = "val" if img_path in val_set else "train"
        stem = f"{img_path.stem}_{idx:05d}"

        dst_img = out / "images" / split / f"{stem}.jpg"
        dst_label = out / "labels" / split / f"{stem}.txt"

        img = cv2.imread(str(img_path))
        if img is None:
            continue

        h, w = img.shape[:2]
        cv2.imwrite(str(dst_img), img)

        results = model.predict(
            source=img,
            conf=args.conf,
            imgsz=args.imgsz,
            verbose=False,
        )

        lines = []
        preview = img.copy()

        if results and results[0].boxes is not None:
            for box in results[0].boxes:
                cls_id = int(box.cls[0].item())
                if cls_id != 0:
                    continue

                conf = float(box.conf[0].item())
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                line = yolo_line_from_xyxy(0, x1, y1, x2, y2, w, h)
                if line is None:
                    continue

                lines.append(line)

                cv2.rectangle(
                    preview,
                    (int(x1), int(y1)),
                    (int(x2), int(y2)),
                    (0, 255, 255),
                    2,
                )
                cv2.putText(
                    preview,
                    f"auto {conf:.2f}",
                    (int(x1), max(25, int(y1) - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 255, 255),
                    2,
                )

        dst_label.write_text("\n".join(lines) + ("\n" if lines else ""), encoding="utf-8")

        if lines:
            labeled_count += 1
        else:
            empty_count += 1

        cv2.imwrite(str(out / "preview" / f"{stem}.jpg"), preview)

        if split == "train":
            train_count += 1
        else:
            val_count_real += 1

    data_yaml = out / "red_block_v2.yaml"
    data_yaml.write_text(
        f"path: {out}\n"
        f"train: images/train\n"
        f"val: images/val\n"
        f"names:\n"
        f"  0: red_block\n",
        encoding="utf-8",
    )

    print("===== dataset prepared =====")
    print("source images:", len(images))
    print("train images :", train_count)
    print("val images   :", val_count_real)
    print("auto labeled :", labeled_count)
    print("empty labels :", empty_count)
    print("dataset dir  :", out)
    print("data yaml    :", data_yaml)
    print("preview dir  :", out / "preview")


if __name__ == "__main__":
    main()
