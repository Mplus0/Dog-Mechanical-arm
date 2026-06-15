#!/usr/bin/env python3
import argparse
import random
import shutil
from pathlib import Path

import cv2
import numpy as np


IMAGE_EXTS = {".jpg", ".jpeg", ".png", ".bmp"}


def yolo_line_from_xyxy(cls_id, x1, y1, x2, y2, w, h):
    x1 = max(0, min(w - 1, int(x1)))
    y1 = max(0, min(h - 1, int(y1)))
    x2 = max(0, min(w - 1, int(x2)))
    y2 = max(0, min(h - 1, int(y2)))

    if x2 <= x1 or y2 <= y1:
        return None

    xc = ((x1 + x2) / 2.0) / w
    yc = ((y1 + y2) / 2.0) / h
    bw = (x2 - x1) / w
    bh = (y2 - y1) / h

    return f"{cls_id} {xc:.6f} {yc:.6f} {bw:.6f} {bh:.6f}"


def extract_yellow_box(img):
    """
    Extract the yellow debug box drawn by target_localizer_node.
    The saved sample image usually contains a yellow rectangle around the selected target.
    """
    h, w = img.shape[:2]

    # BGR threshold for yellow: high G/R, low B.
    b, g, r = cv2.split(img)
    mask = ((g > 170) & (r > 170) & (b < 120)).astype(np.uint8) * 255

    # Remove tiny text fragments and connect rectangle edges.
    kernel_close = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel_close, iterations=2)

    kernel_dilate = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    mask = cv2.dilate(mask, kernel_dilate, iterations=1)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    candidates = []

    for c in contours:
        x, y, bw, bh = cv2.boundingRect(c)
        area = bw * bh

        if bw < 20 or bh < 20:
            continue
        if area < 800:
            continue

        # Exclude text line near the top if it becomes connected.
        if y < 20 and bh < 60:
            continue

        # Exclude extremely wide thin text-like regions.
        aspect = bw / max(bh, 1)
        if aspect > 5.5 or aspect < 0.15:
            continue

        # Target boxes should not cover almost the whole image.
        if bw > 0.9 * w or bh > 0.9 * h:
            continue

        candidates.append((area, x, y, x + bw, y + bh))

    if not candidates:
        return None

    # The detection rectangle is usually the largest valid yellow contour.
    candidates.sort(reverse=True)
    _, x1, y1, x2, y2 = candidates[0]

    pad = 2
    x1 = max(0, x1 + pad)
    y1 = max(0, y1 + pad)
    x2 = min(w - 1, x2 - pad)
    y2 = min(h - 1, y2 - pad)

    if x2 <= x1 or y2 <= y1:
        return None

    return x1, y1, x2, y2


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--src", default="/home/sunrise/dog/ros2_red_block_ws/hard_samples")
    parser.add_argument("--out", default="/home/sunrise/dog/ros2_red_block_ws/datasets/red_block_v2_full")
    parser.add_argument("--val-ratio", type=float, default=0.2)
    parser.add_argument("--seed", type=int, default=42)
    args = parser.parse_args()

    src = Path(args.src)
    out = Path(args.out)

    images = sorted(
        p for p in src.rglob("*")
        if p.suffix.lower() in IMAGE_EXTS and "manual_test" not in p.name
    )

    if not images:
        raise RuntimeError(f"No images found in {src}")

    if out.exists():
        shutil.rmtree(out)

    for sub in ["images/train", "images/val", "labels/train", "labels/val", "preview"]:
        (out / sub).mkdir(parents=True, exist_ok=True)

    random.seed(args.seed)
    random.shuffle(images)

    val_count = max(1, int(len(images) * args.val_ratio))
    val_set = set(images[:val_count])

    labeled_count = 0
    empty_count = 0
    train_count = 0
    val_count_real = 0

    for idx, img_path in enumerate(images):
        split = "val" if img_path in val_set else "train"
        stem = f"{img_path.stem}_{idx:05d}"

        img = cv2.imread(str(img_path))
        if img is None:
            continue

        h, w = img.shape[:2]
        dst_img = out / "images" / split / f"{stem}.jpg"
        dst_label = out / "labels" / split / f"{stem}.txt"
        preview_path = out / "preview" / f"{stem}.jpg"

        cv2.imwrite(str(dst_img), img)

        box = extract_yellow_box(img)

        preview = img.copy()
        lines = []

        if box is not None:
            x1, y1, x2, y2 = box
            line = yolo_line_from_xyxy(0, x1, y1, x2, y2, w, h)
            if line is not None:
                lines.append(line)
                labeled_count += 1
                cv2.rectangle(preview, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(
                    preview,
                    "label_from_saved_box",
                    (x1, max(25, y1 - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 255, 0),
                    2,
                )
            else:
                empty_count += 1
        else:
            empty_count += 1

        dst_label.write_text("\n".join(lines) + ("\n" if lines else ""), encoding="utf-8")
        cv2.imwrite(str(preview_path), preview)

        if split == "train":
            train_count += 1
        else:
            val_count_real += 1

    yaml_path = out / "red_block_v2_full.yaml"
    yaml_path.write_text(
        f"path: {out.resolve()}\n"
        f"train: images/train\n"
        f"val: images/val\n"
        f"names:\n"
        f"  0: red_block\n",
        encoding="utf-8",
    )

    print("===== v2 full dataset built =====")
    print("source images:", len(images))
    print("train images :", train_count)
    print("val images   :", val_count_real)
    print("labeled      :", labeled_count)
    print("empty labels :", empty_count)
    print("dataset dir  :", out)
    print("yaml         :", yaml_path)
    print("preview dir  :", out / "preview")


if __name__ == "__main__":
    main()
