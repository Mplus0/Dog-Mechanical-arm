"""OpenCV-based two-dimensional AprilTag detector."""

from dataclasses import dataclass
from typing import Iterable, List, Sequence, Tuple

import cv2
import numpy as np


Point2D = Tuple[float, float]


@dataclass(frozen=True)
class AprilTagDetection2D:
    """One allowed AprilTag detection in detector corner order."""

    tag_id: int
    corners: Tuple[Point2D, Point2D, Point2D, Point2D]
    center: Point2D
    area_px2: float
    perimeter_px: float


@dataclass(frozen=True)
class AprilTagDetectionBatch:
    """Detections from one image."""

    detections: Tuple[AprilTagDetection2D, ...]
    ignored_ids: Tuple[int, ...]
    rejected_candidate_count: int


class OpenCvAprilTag25h9Detector:
    """Detect tag25h9 markers and retain only configured IDs."""

    def __init__(self, allowed_ids: Iterable[int] = (0, 1)) -> None:
        if not hasattr(cv2, "aruco"):
            raise RuntimeError("OpenCV was built without cv2.aruco")
        aruco = cv2.aruco
        if not hasattr(aruco, "DICT_APRILTAG_25h9"):
            raise RuntimeError("OpenCV does not provide DICT_APRILTAG_25h9")

        ids = tuple(dict.fromkeys(int(value) for value in allowed_ids))
        if not ids:
            raise ValueError("allowed_ids must contain at least one ID")
        unsupported_ids = sorted(set(ids) - {0, 1})
        if unsupported_ids:
            raise ValueError(
                "stage-1 detector only permits tag IDs 0 and 1; unsupported IDs: "
                + ", ".join(str(value) for value in unsupported_ids)
            )
        self.allowed_ids = ids
        self.allowed_id_set = frozenset(ids)

        self.dictionary = aruco.getPredefinedDictionary(aruco.DICT_APRILTAG_25h9)
        self.parameters = aruco.DetectorParameters()
        if hasattr(aruco, "CORNER_REFINE_SUBPIX"):
            self.parameters.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX

        detector_type = getattr(aruco, "ArucoDetector", None)
        self.detector = (
            detector_type(self.dictionary, self.parameters)
            if detector_type is not None
            else None
        )

    def detect(self, bgr: np.ndarray) -> AprilTagDetectionBatch:
        if bgr is None or bgr.ndim != 3 or bgr.shape[2] != 3:
            raise ValueError("bgr must have shape (height, width, 3)")
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)

        if self.detector is not None:
            corners, marker_ids, rejected = self.detector.detectMarkers(gray)
        else:
            corners, marker_ids, rejected = cv2.aruco.detectMarkers(
                gray,
                self.dictionary,
                parameters=self.parameters,
            )

        accepted: List[AprilTagDetection2D] = []
        ignored: List[int] = []
        if marker_ids is not None:
            for raw_corners, raw_id in zip(corners, marker_ids.reshape(-1)):
                tag_id = int(raw_id)
                points = np.asarray(raw_corners, dtype=np.float32).reshape(4, 2)
                if tag_id not in self.allowed_id_set:
                    ignored.append(tag_id)
                    continue

                center = np.mean(points, axis=0)
                area = abs(float(cv2.contourArea(points)))
                perimeter = float(cv2.arcLength(points, True))
                accepted.append(
                    AprilTagDetection2D(
                        tag_id=tag_id,
                        corners=tuple(
                            (float(point[0]), float(point[1])) for point in points
                        ),
                        center=(float(center[0]), float(center[1])),
                        area_px2=area,
                        perimeter_px=perimeter,
                    )
                )

        order = {tag_id: index for index, tag_id in enumerate(self.allowed_ids)}
        accepted.sort(key=lambda item: order[item.tag_id])
        return AprilTagDetectionBatch(
            detections=tuple(accepted),
            ignored_ids=tuple(sorted(set(ignored))),
            rejected_candidate_count=0 if rejected is None else len(rejected),
        )


def draw_detections(
    bgr: np.ndarray,
    detections: Sequence[AprilTagDetection2D],
) -> np.ndarray:
    """Return an annotated copy without changing the input image."""

    display = bgr.copy()
    for detection in detections:
        points = np.rint(np.asarray(detection.corners)).astype(np.int32)
        cv2.polylines(display, [points], True, (0, 255, 0), 2, cv2.LINE_AA)
        for index, point in enumerate(points):
            cv2.circle(display, tuple(point), 4, (255, 0, 255), -1)
            cv2.putText(
                display,
                str(index),
                (int(point[0]) + 5, int(point[1]) - 5),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                (255, 0, 255),
                1,
                cv2.LINE_AA,
            )
        center = tuple(np.rint(detection.center).astype(np.int32))
        cv2.circle(display, center, 5, (0, 255, 255), -1)
        cv2.putText(
            display,
            f"ID {detection.tag_id} area={detection.area_px2:.0f}px2",
            (int(points[0][0]), max(20, int(points[0][1]) - 12)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.60,
            (0, 255, 0),
            2,
            cv2.LINE_AA,
        )
    return display
