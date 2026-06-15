#!/usr/bin/env python3
from dataclasses import dataclass
from ultralytics import YOLO


@dataclass
class Detection:
    class_id: int
    class_name: str
    conf: float
    x1: int
    y1: int
    x2: int
    y2: int

    @property
    def center(self):
        return int((self.x1 + self.x2) / 2), int((self.y1 + self.y2) / 2)

    @property
    def width(self):
        return self.x2 - self.x1

    @property
    def height(self):
        return self.y2 - self.y1

    def to_dict(self, index):
        cx, cy = self.center
        return {
            "id": index,
            "class_id": self.class_id,
            "class_name": self.class_name,
            "confidence": self.conf,
            "bbox": {
                "x1": self.x1,
                "y1": self.y1,
                "x2": self.x2,
                "y2": self.y2,
                "w": self.width,
                "h": self.height
            },
            "center": {
                "x": cx,
                "y": cy
            }
        }


class YoloRedBlockDetector:
    def __init__(self, model_path, conf_thres=0.35, max_targets=4):
        self.model_path = model_path
        self.conf_thres = conf_thres
        self.max_targets = max_targets
        self.model = None

    def load(self):
        print("Loading YOLO model:", self.model_path)
        self.model = YOLO(self.model_path)
        print("YOLO model loaded.")

    def detect(self, bgr_image):
        if self.model is None:
            self.load()

        img_h, img_w = bgr_image.shape[:2]
        results = self.model.predict(source=bgr_image, conf=self.conf_thres, verbose=False)
        detections = []

        if not results or results[0].boxes is None:
            return detections

        for box in results[0].boxes:
            cls_id = int(box.cls[0].item())
            conf = float(box.conf[0].item())

            if cls_id != 0:
                continue

            x1, y1, x2, y2 = box.xyxy[0].tolist()
            x1 = int(max(0, min(img_w - 1, x1)))
            y1 = int(max(0, min(img_h - 1, y1)))
            x2 = int(max(0, min(img_w - 1, x2)))
            y2 = int(max(0, min(img_h - 1, y2)))

            if x2 <= x1 or y2 <= y1:
                continue

            detections.append(
                Detection(
                    class_id=cls_id,
                    class_name="red_block",
                    conf=conf,
                    x1=x1,
                    y1=y1,
                    x2=x2,
                    y2=y2,
                )
            )

        detections.sort(key=lambda item: item.conf, reverse=True)
        return detections[:self.max_targets]
