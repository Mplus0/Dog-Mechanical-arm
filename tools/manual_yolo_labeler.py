#!/usr/bin/env python3
import argparse
from pathlib import Path

import cv2


IMG_EXTS = {".jpg", ".jpeg", ".png", ".bmp"}


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def normalize_box(box):
    x1, y1, x2, y2 = box
    return [min(x1, x2), min(y1, y2), max(x1, x2), max(y1, y2)]


class YoloLabeler:
    def __init__(self, dataset, split, scale=1.0, only_empty=False, name_filter="", start_index=0):
        self.dataset = Path(dataset).expanduser().resolve()
        self.split = split
        self.scale = float(scale)
        self.only_empty = bool(only_empty)
        self.filters = [x.strip() for x in name_filter.split(",") if x.strip()]
        self.image_dir = self.dataset / "images" / self.split
        self.label_dir = self.dataset / "labels" / self.split
        self.label_dir.mkdir(parents=True, exist_ok=True)

        self.images = self.collect_images()
        if not self.images:
            raise RuntimeError(f"No images found: {self.image_dir}")

        self.index = clamp(int(start_index), 0, len(self.images) - 1)
        self.image = None
        self.image_path = None
        self.label_path = None
        self.boxes = []
        self.selected = -1

        self.mode = None
        self.drag_start = None
        self.current_box = None
        self.move_origin = None
        self.move_start = None

        self.window = "manual_yolo_labeler"
        cv2.namedWindow(self.window, cv2.WINDOW_NORMAL)
        cv2.setMouseCallback(self.window, self.on_mouse)

    def collect_images(self):
        images = []
        for p in sorted(self.image_dir.iterdir()):
            if p.suffix.lower() not in IMG_EXTS:
                continue
            if self.filters and not any(f in p.name for f in self.filters):
                continue
            if self.only_empty:
                lp = self.image_to_label(p)
                if lp.exists() and lp.stat().st_size > 0:
                    continue
            images.append(p)
        return images

    def image_to_label(self, image_path):
        return self.label_dir / (image_path.stem + ".txt")

    def load_current(self):
        self.image_path = self.images[self.index]
        self.label_path = self.image_to_label(self.image_path)
        self.image = cv2.imread(str(self.image_path))
        if self.image is None:
            raise RuntimeError(f"Failed to read image: {self.image_path}")
        self.boxes = self.load_labels(self.label_path, self.image.shape[1], self.image.shape[0])
        self.selected = -1
        self.mode = None
        self.current_box = None

    def load_labels(self, label_path, width, height):
        boxes = []
        if not label_path.exists():
            return boxes

        for line in label_path.read_text(encoding="utf-8").splitlines():
            parts = line.strip().split()
            if len(parts) != 5:
                continue
            try:
                cls_id = int(float(parts[0]))
                cx = float(parts[1])
                cy = float(parts[2])
                bw = float(parts[3])
                bh = float(parts[4])
            except Exception:
                continue

            if cls_id != 0:
                continue

            x1 = int((cx - bw / 2.0) * width)
            y1 = int((cy - bh / 2.0) * height)
            x2 = int((cx + bw / 2.0) * width)
            y2 = int((cy + bh / 2.0) * height)

            x1 = clamp(x1, 0, width - 1)
            x2 = clamp(x2, 0, width - 1)
            y1 = clamp(y1, 0, height - 1)
            y2 = clamp(y2, 0, height - 1)

            if abs(x2 - x1) >= 3 and abs(y2 - y1) >= 3:
                boxes.append(normalize_box([x1, y1, x2, y2]))

        return boxes

    def save_labels(self):
        h, w = self.image.shape[:2]
        lines = []

        for box in self.boxes:
            x1, y1, x2, y2 = normalize_box(box)
            x1 = clamp(x1, 0, w - 1)
            x2 = clamp(x2, 0, w - 1)
            y1 = clamp(y1, 0, h - 1)
            y2 = clamp(y2, 0, h - 1)

            bw = max(1, x2 - x1)
            bh = max(1, y2 - y1)
            cx = x1 + bw / 2.0
            cy = y1 + bh / 2.0

            lines.append(
                f"0 {cx / w:.6f} {cy / h:.6f} {bw / w:.6f} {bh / h:.6f}"
            )

        self.label_path.write_text("\n".join(lines) + ("\n" if lines else ""), encoding="utf-8")
        print(f"Saved: {self.label_path} boxes={len(self.boxes)}")

    def to_image_xy(self, x, y):
        return int(x / self.scale), int(y / self.scale)

    def find_box(self, x, y):
        for i in range(len(self.boxes) - 1, -1, -1):
            x1, y1, x2, y2 = normalize_box(self.boxes[i])
            if x1 <= x <= x2 and y1 <= y <= y2:
                return i
        return -1

    def on_mouse(self, event, x, y, flags, param):
        if self.image is None:
            return

        ix, iy = self.to_image_xy(x, y)
        h, w = self.image.shape[:2]
        ix = clamp(ix, 0, w - 1)
        iy = clamp(iy, 0, h - 1)

        if event == cv2.EVENT_LBUTTONDOWN:
            hit = self.find_box(ix, iy)
            if hit >= 0:
                self.selected = hit
                self.mode = "move"
                self.move_start = (ix, iy)
                self.move_origin = list(self.boxes[hit])
            else:
                self.selected = -1
                self.mode = "draw"
                self.drag_start = (ix, iy)
                self.current_box = [ix, iy, ix, iy]

        elif event == cv2.EVENT_MOUSEMOVE:
            if self.mode == "draw" and self.drag_start is not None:
                x0, y0 = self.drag_start
                self.current_box = normalize_box([x0, y0, ix, iy])
            elif self.mode == "move" and self.selected >= 0:
                sx, sy = self.move_start
                dx = ix - sx
                dy = iy - sy
                x1, y1, x2, y2 = self.move_origin
                bw = x2 - x1
                bh = y2 - y1
                nx1 = clamp(x1 + dx, 0, w - 1 - bw)
                ny1 = clamp(y1 + dy, 0, h - 1 - bh)
                self.boxes[self.selected] = [nx1, ny1, nx1 + bw, ny1 + bh]

        elif event == cv2.EVENT_LBUTTONUP:
            if self.mode == "draw" and self.current_box is not None:
                x1, y1, x2, y2 = normalize_box(self.current_box)
                if abs(x2 - x1) >= 5 and abs(y2 - y1) >= 5:
                    self.boxes.append([x1, y1, x2, y2])
                    self.selected = len(self.boxes) - 1
                    print(f"Added box: {self.boxes[-1]}")
            self.mode = None
            self.current_box = None

        elif event == cv2.EVENT_RBUTTONDOWN:
            hit = self.find_box(ix, iy)
            if hit >= 0:
                removed = self.boxes.pop(hit)
                self.selected = -1
                print(f"Removed box: {removed}")

    def draw(self):
        display = self.image.copy()
        h, w = display.shape[:2]

        for i, box in enumerate(self.boxes):
            x1, y1, x2, y2 = normalize_box(box)
            color = (0, 255, 255) if i == self.selected else (0, 255, 0)
            thickness = 3 if i == self.selected else 2
            cv2.rectangle(display, (x1, y1), (x2, y2), color, thickness)
            cv2.putText(
                display,
                f"red_block {i}",
                (x1, max(20, y1 - 6)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                color,
                2,
            )

        if self.current_box is not None:
            x1, y1, x2, y2 = normalize_box(self.current_box)
            cv2.rectangle(display, (x1, y1), (x2, y2), (255, 255, 0), 2)

        title = f"{self.index + 1}/{len(self.images)} {self.image_path.name} boxes={len(self.boxes)}"
        help_text = "Left drag:add | drag box:move | right click:delete | s:save | space/n:save next | p:prev | c:clear | u:undo | e:empty next | q:save quit | x:quit no save"

        cv2.putText(display, title, (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2)
        cv2.putText(display, help_text, (10, h - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)

        if self.scale != 1.0:
            display = cv2.resize(display, (int(w * self.scale), int(h * self.scale)))
        return display

    def next_image(self):
        if self.index < len(self.images) - 1:
            self.index += 1
            self.load_current()
        else:
            print("Already at last image.")

    def prev_image(self):
        if self.index > 0:
            self.index -= 1
            self.load_current()
        else:
            print("Already at first image.")

    def run(self):
        self.load_current()
        print("")
        print("Controls:")
        print("  Left drag        add new box")
        print("  Drag inside box  move selected box")
        print("  Right click      delete box")
        print("  s                save")
        print("  space / n        save and next")
        print("  p                save and previous")
        print("  c                clear all boxes")
        print("  u                undo last box")
        print("  e                mark empty, save and next")
        print("  q / ESC          save and quit")
        print("  x                quit without saving current image")
        print("")

        while True:
            cv2.imshow(self.window, self.draw())
            key = cv2.waitKey(30) & 0xFF

            if key == 255:
                continue

            if key in (ord("s"),):
                self.save_labels()

            elif key in (ord("n"), ord(" "), 13):
                self.save_labels()
                self.next_image()

            elif key == ord("p"):
                self.save_labels()
                self.prev_image()

            elif key == ord("c"):
                self.boxes.clear()
                self.selected = -1
                print("Cleared all boxes. Press s or next to save.")

            elif key == ord("u"):
                if self.boxes:
                    removed = self.boxes.pop()
                    self.selected = -1
                    print(f"Undo removed: {removed}")

            elif key == ord("d"):
                if self.selected >= 0:
                    removed = self.boxes.pop(self.selected)
                    self.selected = -1
                    print(f"Deleted selected: {removed}")
                elif self.boxes:
                    removed = self.boxes.pop()
                    print(f"Deleted last: {removed}")

            elif key == ord("e"):
                self.boxes.clear()
                self.selected = -1
                self.save_labels()
                self.next_image()

            elif key in (ord("q"), 27):
                self.save_labels()
                break

            elif key == ord("x"):
                print("Quit without saving current image.")
                break

        cv2.destroyAllWindows()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--dataset", required=True)
    parser.add_argument("--split", default="train", choices=["train", "val"])
    parser.add_argument("--scale", type=float, default=1.0)
    parser.add_argument("--only-empty", action="store_true")
    parser.add_argument("--filter", default="")
    parser.add_argument("--start-index", type=int, default=0)
    args = parser.parse_args()

    app = YoloLabeler(
        dataset=args.dataset,
        split=args.split,
        scale=args.scale,
        only_empty=args.only_empty,
        name_filter=args.filter,
        start_index=args.start_index,
    )
    app.run()


if __name__ == "__main__":
    main()
