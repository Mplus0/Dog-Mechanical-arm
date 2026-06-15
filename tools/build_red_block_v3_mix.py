#!/usr/bin/env python3
import argparse
import shutil
from pathlib import Path


IMG_EXTS = [".jpg", ".jpeg", ".png", ".bmp"]


def find_images(image_dir):
    result = []
    for ext in IMG_EXTS:
        result.extend(image_dir.glob(f"*{ext}"))
        result.extend(image_dir.glob(f"*{ext.upper()}"))
    return sorted(result)


def valid_label_text(text):
    if text is None:
        return False, "missing_label"

    text = text.strip()
    if text == "":
        return True, "empty_label"

    for line in text.splitlines():
        parts = line.strip().split()
        if len(parts) != 5:
            return False, f"bad_line:{line}"

        try:
            cls_id = int(float(parts[0]))
            nums = [float(x) for x in parts[1:]]
        except Exception:
            return False, f"bad_number:{line}"

        if cls_id != 0:
            return False, f"bad_class:{cls_id}"

        for v in nums:
            if v < 0.0 or v > 1.0:
                return False, f"out_of_range:{line}"

    return True, "ok"


def copy_dataset(src, dst, split, prefix, keep_empty):
    src = Path(src)
    dst = Path(dst)

    src_img_dir = src / "images" / split
    src_lab_dir = src / "labels" / split
    dst_img_dir = dst / "images" / split
    dst_lab_dir = dst / "labels" / split

    dst_img_dir.mkdir(parents=True, exist_ok=True)
    dst_lab_dir.mkdir(parents=True, exist_ok=True)

    stats = {
        "source": str(src),
        "split": split,
        "seen": 0,
        "copied": 0,
        "missing": 0,
        "empty": 0,
        "skipped_empty": 0,
        "invalid": 0,
    }

    for img in find_images(src_img_dir):
        stats["seen"] += 1

        label = src_lab_dir / f"{img.stem}.txt"
        if not label.exists():
            stats["missing"] += 1
            print(f"[SKIP missing label] {img}")
            continue

        text = label.read_text(encoding="utf-8")
        ok, reason = valid_label_text(text)

        if not ok:
            stats["invalid"] += 1
            print(f"[SKIP invalid label] {label} reason={reason}")
            continue

        if reason == "empty_label":
            stats["empty"] += 1
            if not keep_empty:
                stats["skipped_empty"] += 1
                continue

        new_stem = f"{prefix}_{split}_{img.stem}"
        new_img = dst_img_dir / f"{new_stem}{img.suffix.lower()}"
        new_lab = dst_lab_dir / f"{new_stem}.txt"

        shutil.copy2(img, new_img)
        new_lab.write_text(text.strip() + ("\n" if text.strip() else ""), encoding="utf-8")
        stats["copied"] += 1

    return stats


def count_output(dst):
    dst = Path(dst)
    for split in ["train", "val"]:
        img_dir = dst / "images" / split
        lab_dir = dst / "labels" / split

        images = find_images(img_dir)
        labels = sorted(lab_dir.glob("*.txt"))
        empty = [p for p in labels if p.stat().st_size == 0]

        print(f"{split} images: {len(images)}")
        print(f"{split} labels: {len(labels)}")
        print(f"{split} empty labels: {len(empty)}")


def write_yaml(dst):
    dst = Path(dst)
    yaml_path = dst / "red_block_v3_mix.yaml"
    yaml_path.write_text(
        f"path: {dst}\n"
        "train: images/train\n"
        "val: images/val\n"
        "names:\n"
        "  0: red_block\n",
        encoding="utf-8",
    )
    return yaml_path


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--old", default="/home/sunrise/dog/red_block_grasp/dataset/red_block_dataset")
    parser.add_argument("--new", default="/home/sunrise/dog/ros2_red_block_ws/datasets/red_block_v2_full")
    parser.add_argument("--out", default="/home/sunrise/dog/ros2_red_block_ws/datasets/red_block_v3_mix")
    parser.add_argument("--overwrite", action="store_true")
    parser.add_argument("--keep-empty", action="store_true")
    args = parser.parse_args()

    out = Path(args.out)

    if out.exists():
        if not args.overwrite:
            raise RuntimeError(f"Output exists: {out}. Use --overwrite.")
        shutil.rmtree(out)

    out.mkdir(parents=True, exist_ok=True)

    stats = []
    for split in ["train", "val"]:
        stats.append(copy_dataset(args.old, out, split, "old", keep_empty=args.keep_empty))
        stats.append(copy_dataset(args.new, out, split, "new", keep_empty=args.keep_empty))

    yaml_path = write_yaml(out)

    print("")
    print("===== copy stats =====")
    for s in stats:
        print(
            f"{s['source']} [{s['split']}] "
            f"seen={s['seen']} copied={s['copied']} "
            f"missing={s['missing']} empty={s['empty']} "
            f"skipped_empty={s['skipped_empty']} invalid={s['invalid']}"
        )

    print("")
    print("===== output counts =====")
    count_output(out)

    print("")
    print("yaml:", yaml_path)
    print("done")


if __name__ == "__main__":
    main()
