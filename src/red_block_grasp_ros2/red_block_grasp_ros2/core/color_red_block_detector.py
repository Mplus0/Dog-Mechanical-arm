#!/usr/bin/env python3
from dataclasses import dataclass, field

import cv2
import numpy as np


@dataclass
class RedColorDetection:
    class_id: int
    class_name: str
    conf: float
    x1: int
    y1: int
    x2: int
    y2: int
    contour: object = None
    mask: object = None
    depth_mm: float = None
    valid_depth: bool = False
    score: float = 0.0
    debug_info: dict = field(default_factory=dict)

    @property
    def center(self):
        return int((self.x1 + self.x2) / 2), int((self.y1 + self.y2) / 2)

    @property
    def width(self):
        return int(self.x2 - self.x1)

    @property
    def height(self):
        return int(self.y2 - self.y1)


class RedColorBlockDetector:
    def __init__(
        self,
        min_depth_mm=100,
        max_depth_mm=700,
        min_area=250,
        min_area_ratio=0.0,
        max_area_ratio=0.35,
        min_width=8,
        min_height=8,
        border_margin_px=3,
        aspect_min=0.35,
        aspect_max=3.0,
        extent_min=0.25,
        solidity_min=0.50,
        circularity_min=0.0,
        morph_kernel_size=5,
        erode_kernel_size=5,
        hsv_h1_min=0,
        hsv_h1_max=10,
        hsv_h2_min=170,
        hsv_h2_max=180,
        hsv_s_min=70,
        hsv_v_min=45,
        lab_a_min=145,
        bgr_r_min=80,
        bgr_rg_delta=35,
        bgr_rb_delta=25,
        bgr_b_max=210,
        min_valid_depth_count=8,
        max_targets=4,
    ):
        self.min_depth_mm = float(min_depth_mm)
        self.max_depth_mm = float(max_depth_mm)
        self.min_area = float(min_area)
        self.min_area_ratio = float(min_area_ratio)
        self.max_area_ratio = float(max_area_ratio)
        self.min_width = int(min_width)
        self.min_height = int(min_height)
        self.border_margin_px = int(border_margin_px)
        self.aspect_min = float(aspect_min)
        self.aspect_max = float(aspect_max)
        self.extent_min = float(extent_min)
        self.solidity_min = float(solidity_min)
        self.circularity_min = float(circularity_min)
        self.morph_kernel_size = int(max(3, morph_kernel_size))
        self.erode_kernel_size = int(max(3, erode_kernel_size))
        self.hsv_h1_min = int(hsv_h1_min)
        self.hsv_h1_max = int(hsv_h1_max)
        self.hsv_h2_min = int(hsv_h2_min)
        self.hsv_h2_max = int(hsv_h2_max)
        self.hsv_s_min = int(hsv_s_min)
        self.hsv_v_min = int(hsv_v_min)
        self.lab_a_min = int(lab_a_min)
        self.bgr_r_min = int(bgr_r_min)
        self.bgr_rg_delta = int(bgr_rg_delta)
        self.bgr_rb_delta = int(bgr_rb_delta)
        self.bgr_b_max = int(bgr_b_max)
        self.min_valid_depth_count = int(max(1, min_valid_depth_count))
        self.max_targets = int(max_targets)

    @staticmethod
    def _clamp01(value):
        return max(0.0, min(1.0, float(value)))

    @staticmethod
    def _odd(value):
        value = int(max(1, value))
        return value if value % 2 == 1 else value + 1

    def remove_small_components(self, binary, min_area):
        # 参考 AIRBOT 的做法，形态学后再清理小连通域，避免碎红点进入候选。
        num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(binary, connectivity=8)
        cleaned = np.zeros(binary.shape, dtype=np.uint8)
        min_area_int = max(1, int(round(float(min_area))))
        for label in range(1, num_labels):
            area = int(stats[label, cv2.CC_STAT_AREA])
            if area >= min_area_int:
                cleaned[labels == label] = 255
        return cleaned

    def build_mask(self, bgr_image):
        blurred = cv2.GaussianBlur(bgr_image, (5, 5), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        lab = cv2.cvtColor(blurred, cv2.COLOR_BGR2LAB)

        hsv_low = cv2.inRange(
            hsv,
            np.array([self.hsv_h1_min, self.hsv_s_min, self.hsv_v_min], dtype=np.uint8),
            np.array([self.hsv_h1_max, 255, 255], dtype=np.uint8),
        )
        hsv_high = cv2.inRange(
            hsv,
            np.array([self.hsv_h2_min, self.hsv_s_min, self.hsv_v_min], dtype=np.uint8),
            np.array([self.hsv_h2_max, 255, 255], dtype=np.uint8),
        )
        hsv_mask = cv2.bitwise_or(hsv_low, hsv_high)

        lab_a = lab[:, :, 1]
        lab_mask = (lab_a >= self.lab_a_min).astype(np.uint8) * 255

        b = blurred[:, :, 0].astype(np.int16)
        g = blurred[:, :, 1].astype(np.int16)
        r = blurred[:, :, 2].astype(np.int16)
        bgr_rule = (
            (r >= self.bgr_r_min)
            & (r >= g + self.bgr_rg_delta)
            & (r >= b + self.bgr_rb_delta)
            & (b <= self.bgr_b_max)
        )
        bgr_mask = bgr_rule.astype(np.uint8) * 255

        aux_mask = cv2.bitwise_or(lab_mask, bgr_mask)
        mask = cv2.bitwise_and(hsv_mask, aux_mask)
        mask = cv2.medianBlur(mask, self._odd(self.morph_kernel_size))

        kernel_open = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE,
            (self._odd(self.morph_kernel_size), self._odd(self.morph_kernel_size)),
        )
        close_size = self._odd(max(self.morph_kernel_size + 2, int(round(self.morph_kernel_size * 1.5))))
        kernel_close = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE,
            (close_size, close_size),
        )
        kernel_dilate = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))

        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel_open)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel_close)
        mask = cv2.dilate(mask, kernel_dilate, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel_open)
        return mask

    def merge_nearby_contours(self, contours, image_shape, depth_mm):
        # 只合并“膨胀后轻微接触”的近邻碎块，尽量把同一红块局部拼回整体。
        if len(contours) <= 1:
            return contours

        image_h, image_w = image_shape[:2]
        merge_size = self._odd(max(5, self.morph_kernel_size + 4))
        merge_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (merge_size, merge_size))
        masks = []
        dilated_masks = []
        centers = []
        depths = []

        for contour in contours:
            candidate_mask = np.zeros((image_h, image_w), dtype=np.uint8)
            cv2.drawContours(candidate_mask, [contour], -1, 255, thickness=-1)
            masks.append(candidate_mask)
            dilated_masks.append(cv2.dilate(candidate_mask, merge_kernel, iterations=1))
            moments = cv2.moments(contour)
            if abs(moments["m00"]) > 1e-6:
                center = (int(round(moments["m10"] / moments["m00"])), int(round(moments["m01"] / moments["m00"])))
            else:
                x, y, w, h = cv2.boundingRect(contour)
                center = (int(x + w / 2), int(y + h / 2))
            centers.append(center)
            depth, valid_depth, _ = self.robust_depth_from_mask(depth_mm, candidate_mask, center, image_shape)
            depths.append(depth if valid_depth else None)

        parents = list(range(len(contours)))

        def find(idx):
            while parents[idx] != idx:
                parents[idx] = parents[parents[idx]]
                idx = parents[idx]
            return idx

        def union(a, b):
            ra = find(a)
            rb = find(b)
            if ra != rb:
                parents[rb] = ra

        for i in range(len(contours)):
            for j in range(i + 1, len(contours)):
                if cv2.countNonZero(cv2.bitwise_and(dilated_masks[i], dilated_masks[j])) <= 0:
                    continue
                depth_i = depths[i]
                depth_j = depths[j]
                if depth_i is not None and depth_j is not None and abs(float(depth_i) - float(depth_j)) > 80.0:
                    continue
                if self.pixel_distance(centers[i], centers[j]) > max(40.0, 2.2 * self.morph_kernel_size + 22.0):
                    continue
                union(i, j)

        grouped_masks = {}
        for idx, mask in enumerate(masks):
            root = find(idx)
            if root not in grouped_masks:
                grouped_masks[root] = np.zeros((image_h, image_w), dtype=np.uint8)
            grouped_masks[root] = cv2.bitwise_or(grouped_masks[root], mask)

        merged = []
        for merged_mask in grouped_masks.values():
            merged_contours, _ = cv2.findContours(merged_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            merged.extend(merged_contours)
        return merged

    @staticmethod
    def pixel_distance(p1, p2):
        return float(np.hypot(float(p1[0]) - float(p2[0]), float(p1[1]) - float(p2[1])))

    def _scale_mask_to_depth(self, mask, depth_shape):
        depth_h, depth_w = depth_shape[:2]
        mask_h, mask_w = mask.shape[:2]
        if (depth_h, depth_w) == (mask_h, mask_w):
            return mask
        return cv2.resize(mask, (depth_w, depth_h), interpolation=cv2.INTER_NEAREST)

    def _valid_depths(self, values):
        values = values[(values > self.min_depth_mm) & (values < self.max_depth_mm)]
        if values.size == 0:
            return values
        median = float(np.median(values))
        gate = max(80.0, median * 0.10)
        return values[np.abs(values.astype(np.float32) - median) <= gate]

    def _window_depth(self, depth_mm, center, image_shape, radius):
        color_h, color_w = image_shape[:2]
        depth_h, depth_w = depth_mm.shape[:2]
        u, v = center
        u_depth = int(round(float(u) * depth_w / max(color_w, 1)))
        v_depth = int(round(float(v) * depth_h / max(color_h, 1)))

        u0 = max(0, u_depth - radius)
        u1 = min(depth_w, u_depth + radius + 1)
        v0 = max(0, v_depth - radius)
        v1 = min(depth_h, v_depth + radius + 1)
        if u1 <= u0 or v1 <= v0:
            return None

        valid = self._valid_depths(depth_mm[v0:v1, u0:u1])
        if valid.size == 0:
            return None, 0
        return float(np.median(valid)), int(valid.size)

    def robust_depth_from_mask(self, depth_mm, candidate_mask, center, image_shape):
        if depth_mm is None:
            return None, False, {"depth_source": "none", "valid_depth_count": 0}

        scaled_mask = self._scale_mask_to_depth(candidate_mask, depth_mm.shape)
        kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE,
            (self._odd(self.erode_kernel_size), self._odd(self.erode_kernel_size)),
        )
        eroded = cv2.erode(scaled_mask, kernel, iterations=1)
        if int(np.count_nonzero(eroded)) < 8:
            eroded = scaled_mask

        values = depth_mm[eroded > 0]
        valid = self._valid_depths(values)
        if valid.size >= max(12, self.min_valid_depth_count):
            return float(np.median(valid)), True, {
                "depth_source": "eroded_mask_median",
                "valid_depth_count": int(valid.size),
            }
        if valid.size >= self.min_valid_depth_count:
            return float(np.median(valid)), True, {
                "depth_source": "mask_median",
                "valid_depth_count": int(valid.size),
            }

        for radius in (3, 5, 7):
            depth, valid_count = self._window_depth(depth_mm, center, image_shape, radius)
            if depth is not None and valid_count >= self.min_valid_depth_count:
                return depth, True, {
                    "depth_source": f"center_window_{radius}",
                    "valid_depth_count": int(valid_count),
                }

        return None, False, {
            "depth_source": "invalid",
            "valid_depth_count": int(valid.size),
        }

    def validate_color_means(self, mean_bgr, mean_hsv, mean_lab):
        mean_b = float(mean_bgr[0])
        mean_g = float(mean_bgr[1])
        mean_r = float(mean_bgr[2])
        mean_s = float(mean_hsv[1])
        mean_v = float(mean_hsv[2])
        mean_a = float(mean_lab[1])
        rg_delta = mean_r - mean_g
        rb_delta = mean_r - mean_b

        checks = {
            "mean_r_ok": mean_r >= max(70.0, float(self.bgr_r_min) * 0.95),
            "mean_s_ok": mean_s >= max(45.0, float(self.hsv_s_min) * 0.70),
            "mean_v_ok": mean_v >= max(45.0, float(self.hsv_v_min) * 0.85),
            "mean_rg_ok": rg_delta >= max(18.0, float(self.bgr_rg_delta) * 0.75),
            "mean_rb_ok": rb_delta >= max(15.0, float(self.bgr_rb_delta) * 0.75),
            "mean_lab_a_ok": mean_a >= max(140.0, float(self.lab_a_min) - 8.0),
            "reject_green_bias": mean_g <= mean_r - 8.0,
        }
        return all(checks.values()), {
            "mean_b": mean_b,
            "mean_g": mean_g,
            "mean_r": mean_r,
            "mean_s": mean_s,
            "mean_v": mean_v,
            "mean_lab_a": mean_a,
            "mean_rg_delta": rg_delta,
            "mean_rb_delta": rb_delta,
            **checks,
        }

    def _candidate_score(self, area, min_area, max_area, color_ratio, extent, solidity, rr_extent, rr_ratio, depth_valid, color_stats):
        area_floor = max(float(min_area), 1.0)
        area_mid = max(area_floor * 3.0, float(max_area) * 0.10)
        area_score = self._clamp01((float(area) - area_floor) / max(area_mid - area_floor, 1.0))
        if float(area) < area_floor * 1.6:
            area_score *= 0.70

        color_score = np.mean(
            [
                self._clamp01((float(color_stats["mean_r"]) - max(70.0, float(self.bgr_r_min) * 0.95)) / 90.0),
                self._clamp01((float(color_stats["mean_s"]) - max(45.0, float(self.hsv_s_min) * 0.70)) / 80.0),
                self._clamp01((float(color_stats["mean_v"]) - max(45.0, float(self.hsv_v_min) * 0.85)) / 80.0),
                self._clamp01((float(color_stats["mean_rg_delta"]) - max(18.0, float(self.bgr_rg_delta) * 0.75)) / 70.0),
                self._clamp01((float(color_stats["mean_rb_delta"]) - max(15.0, float(self.bgr_rb_delta) * 0.75)) / 70.0),
                self._clamp01((float(color_stats["mean_lab_a"]) - max(140.0, float(self.lab_a_min) - 8.0)) / 40.0),
                self._clamp01(color_ratio),
            ]
        )
        rr_ratio_score = self._clamp01((2.6 - float(rr_ratio)) / 1.6)
        shape_score = np.mean(
            [
                self._clamp01(extent),
                self._clamp01(solidity),
                self._clamp01(rr_extent),
                rr_ratio_score,
            ]
        )
        depth_score = 1.0 if depth_valid else 0.15
        total = 0.26 * area_score + 0.30 * color_score + 0.32 * shape_score + 0.12 * depth_score
        return float(total), {
            "area_score": float(area_score),
            "color_score": float(color_score),
            "shape_score": float(shape_score),
            "depth_score": float(depth_score),
        }

    def detect(self, bgr_image, depth_mm=None, camera_matrix=None):
        del camera_matrix
        if bgr_image is None:
            return []

        image_h, image_w = bgr_image.shape[:2]
        image_area = max(1, image_w * image_h)
        min_area = max(self.min_area, image_area * self.min_area_ratio)
        max_area = image_area * self.max_area_ratio
        mask = self.build_mask(bgr_image)
        mask = self.remove_small_components(mask, min_area)
        blur = cv2.GaussianBlur(bgr_image, (5, 5), 0)
        hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
        lab = cv2.cvtColor(blur, cv2.COLOR_BGR2LAB)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = self.merge_nearby_contours(contours, bgr_image.shape, depth_mm)

        detections = []
        for contour in contours:
            area = float(cv2.contourArea(contour))
            if area < min_area or area > max_area:
                continue

            x, y, w, h = cv2.boundingRect(contour)
            if w < self.min_width or h < self.min_height:
                continue
            if (
                x <= self.border_margin_px
                or y <= self.border_margin_px
                or x + w >= image_w - self.border_margin_px
                or y + h >= image_h - self.border_margin_px
            ):
                continue

            aspect = float(w) / max(float(h), 1.0)
            if aspect < self.aspect_min or aspect > self.aspect_max:
                continue

            bbox_area = float(w * h)
            extent = area / max(bbox_area, 1.0)
            if extent < self.extent_min:
                continue

            hull = cv2.convexHull(contour)
            hull_area = float(cv2.contourArea(hull))
            solidity = area / max(hull_area, 1.0)
            if solidity < self.solidity_min:
                continue

            rr = cv2.minAreaRect(contour)
            rr_w = max(float(rr[1][0]), float(rr[1][1]))
            rr_h = min(float(rr[1][0]), float(rr[1][1]))
            if rr_w <= 1.0 or rr_h <= 1.0:
                continue
            rr_ratio = rr_w / rr_h
            rr_ratio_min = max(1.0, min(self.aspect_min, 1.0))
            rr_ratio_max = max(2.30, min(3.2, self.aspect_max + 0.35))
            if rr_ratio < rr_ratio_min or rr_ratio > rr_ratio_max:
                continue
            rr_area = rr_w * rr_h
            rr_extent = area / max(rr_area, 1.0)
            if rr_extent < max(0.45, self.extent_min + 0.18):
                continue

            perimeter = float(cv2.arcLength(contour, True))
            circularity = 0.0
            if perimeter > 1e-6:
                circularity = 4.0 * np.pi * area / (perimeter * perimeter)
            if self.circularity_min > 0.0 and circularity < self.circularity_min:
                continue

            candidate_mask = np.zeros((image_h, image_w), dtype=np.uint8)
            cv2.drawContours(candidate_mask, [contour], -1, 255, thickness=-1)
            color_ratio = float(np.count_nonzero(cv2.bitwise_and(mask, candidate_mask))) / max(area, 1.0)
            moments = cv2.moments(contour)
            if abs(moments["m00"]) > 1e-6:
                center = (int(round(moments["m10"] / moments["m00"])), int(round(moments["m01"] / moments["m00"])))
            else:
                center = (int(round(rr[0][0])), int(round(rr[0][1])))
            center = (
                max(0, min(center[0], image_w - 1)),
                max(0, min(center[1], image_h - 1)),
            )

            mean_bgr = cv2.mean(blur, candidate_mask)
            mean_hsv = cv2.mean(hsv, candidate_mask)
            mean_lab = cv2.mean(lab, candidate_mask)
            color_valid, color_stats = self.validate_color_means(mean_bgr, mean_hsv, mean_lab)
            if not color_valid:
                continue

            depth, valid_depth, depth_info = self.robust_depth_from_mask(
                depth_mm=depth_mm,
                candidate_mask=candidate_mask,
                center=center,
                image_shape=bgr_image.shape,
            )

            score, score_parts = self._candidate_score(
                area=area,
                min_area=min_area,
                max_area=max_area,
                color_ratio=color_ratio,
                extent=extent,
                solidity=solidity,
                rr_extent=rr_extent,
                rr_ratio=rr_ratio,
                depth_valid=valid_depth,
                color_stats=color_stats,
            )
            visibility_score = self._clamp01(area / max(max_area, 1.0))
            detections.append(
                RedColorDetection(
                    class_id=0,
                    class_name="red_block",
                    conf=float(score),
                    x1=int(x),
                    y1=int(y),
                    x2=int(x + w),
                    y2=int(y + h),
                    contour=contour,
                    mask=candidate_mask,
                    depth_mm=depth,
                    valid_depth=bool(valid_depth),
                    score=float(score),
                    debug_info={
                        "area": area,
                        "bbox_area": bbox_area,
                        "aspect": aspect,
                        "extent": extent,
                        "solidity": solidity,
                        "rr_width": rr_w,
                        "rr_height": rr_h,
                        "rr_ratio": rr_ratio,
                        "rr_extent": rr_extent,
                        "circularity": circularity,
                        "color_ratio": color_ratio,
                        "mask_area": area,
                        "visibility_score": visibility_score,
                        **color_stats,
                        **score_parts,
                        **depth_info,
                    },
                )
            )

        detections.sort(key=lambda item: item.score, reverse=True)
        return detections[: self.max_targets]
