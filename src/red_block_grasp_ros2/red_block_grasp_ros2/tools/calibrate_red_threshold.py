#!/usr/bin/env python3
import argparse
import sys
import time
from pathlib import Path

import cv2
import numpy as np

from red_block_grasp_ros2.core.camera_rgbd_orbbec import OrbbecRgbdCamera


DEFAULT_OUTPUT = "/home/sunrise/dog/ros2_red_block_ws/src/red_block_grasp_ros2/config/red_color_calib.yaml"


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description="Calibrate red block color thresholds for the current lighting.")
    parser.add_argument("--output", default=DEFAULT_OUTPUT)
    parser.add_argument("--frames", type=int, default=40)
    parser.add_argument("--show", action="store_true")
    parser.add_argument("--min-depth-mm", type=float, default=100.0)
    parser.add_argument("--max-depth-mm", type=float, default=700.0)
    parser.add_argument("--center-roi-ratio", type=float, default=0.55)
    return parser.parse_args(argv)


def odd(value):
    value = int(max(1, value))
    return value if value % 2 == 1 else value + 1


def build_loose_mask(bgr):
    blurred = cv2.GaussianBlur(bgr, (5, 5), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    hsv_low = cv2.inRange(
        hsv,
        np.array([0, 45, 35], dtype=np.uint8),
        np.array([15, 255, 255], dtype=np.uint8),
    )
    hsv_high = cv2.inRange(
        hsv,
        np.array([165, 45, 35], dtype=np.uint8),
        np.array([180, 255, 255], dtype=np.uint8),
    )
    hsv_mask = cv2.bitwise_or(hsv_low, hsv_high)

    b = blurred[:, :, 0].astype(np.int16)
    g = blurred[:, :, 1].astype(np.int16)
    r = blurred[:, :, 2].astype(np.int16)
    bgr_rule = (r > 50) & (r > g + 10) & (r > b + 10)
    bgr_mask = bgr_rule.astype(np.uint8) * 255

    mask = cv2.bitwise_and(hsv_mask, bgr_mask)
    mask = cv2.medianBlur(mask, 5)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    return mask


def center_roi_rect(width, height, ratio):
    ratio = max(0.1, min(1.0, float(ratio)))
    roi_w = int(width * ratio)
    roi_h = int(height * ratio)
    x0 = max(0, int((width - roi_w) / 2))
    y0 = max(0, int((height - roi_h) / 2))
    return x0, y0, x0 + roi_w, y0 + roi_h


def contour_depth_median(depth_mm, candidate_mask, min_depth_mm, max_depth_mm):
    if depth_mm is None:
        return None
    depth_h, depth_w = depth_mm.shape[:2]
    mask_h, mask_w = candidate_mask.shape[:2]
    if (depth_h, depth_w) != (mask_h, mask_w):
        candidate_mask = cv2.resize(candidate_mask, (depth_w, depth_h), interpolation=cv2.INTER_NEAREST)

    values = depth_mm[candidate_mask > 0]
    valid = values[(values > min_depth_mm) & (values < max_depth_mm)]
    if valid.size < 8:
        return None
    return float(np.median(valid))


def choose_candidate(mask, depth_mm, image_shape, roi_rect, min_depth_mm, max_depth_mm):
    height, width = image_shape[:2]
    image_area = max(1, width * height)
    rx1, ry1, rx2, ry2 = roi_rect
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    best = None
    best_score = -1.0

    for contour in contours:
        area = float(cv2.contourArea(contour))
        if area < 120.0 or area > image_area * 0.45:
            continue

        x, y, w, h = cv2.boundingRect(contour)
        if w < 8 or h < 8:
            continue
        if x <= 3 or y <= 3 or x + w >= width - 3 or y + h >= height - 3:
            continue

        cx = x + w * 0.5
        cy = y + h * 0.5
        in_center = rx1 <= cx <= rx2 and ry1 <= cy <= ry2

        candidate_mask = np.zeros((height, width), dtype=np.uint8)
        cv2.drawContours(candidate_mask, [contour], -1, 255, thickness=-1)
        depth = contour_depth_median(depth_mm, candidate_mask, min_depth_mm, max_depth_mm)
        if depth is None:
            continue

        center_bonus = 1.0 if in_center else 0.0
        score = area + image_area * 0.05 * center_bonus
        if score > best_score:
            best_score = score
            best = {
                "contour": contour,
                "bbox": (x, y, w, h),
                "area": area,
                "mask": candidate_mask,
                "depth": depth,
            }

    return best


def collect_candidate_pixels(bgr, candidate_mask):
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    eroded = cv2.erode(candidate_mask, kernel, iterations=1)
    if int(np.count_nonzero(eroded)) < 20:
        eroded = candidate_mask

    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    lab = cv2.cvtColor(bgr, cv2.COLOR_BGR2LAB)
    idx = eroded > 0
    bgr_pixels = bgr[idx]
    hsv_pixels = hsv[idx]
    lab_pixels = lab[idx]
    return bgr_pixels, hsv_pixels, lab_pixels


def percentile(values, q):
    return float(np.percentile(np.asarray(values, dtype=np.float32), q))


def clamp_int(value, low, high):
    return int(max(low, min(high, round(float(value)))))


def build_config(bgr_pixels, hsv_pixels, lab_pixels, areas, min_depth_mm, max_depth_mm):
    b = bgr_pixels[:, 0].astype(np.float32)
    g = bgr_pixels[:, 1].astype(np.float32)
    r = bgr_pixels[:, 2].astype(np.float32)
    s = hsv_pixels[:, 1].astype(np.float32)
    v = hsv_pixels[:, 2].astype(np.float32)
    lab_a = lab_pixels[:, 1].astype(np.float32)
    rg = r - g
    rb = r - b

    return {
        "color_hsv_h1_min": 0,
        "color_hsv_h1_max": 15,
        "color_hsv_h2_min": 165,
        "color_hsv_h2_max": 180,
        "color_hsv_s_min": clamp_int(percentile(s, 5) - 10, 30, 255),
        "color_hsv_v_min": clamp_int(percentile(v, 5) - 10, 25, 255),
        "color_lab_a_min": clamp_int(percentile(lab_a, 5) - 8, 120, 255),
        "color_bgr_r_min": clamp_int(percentile(r, 5) - 10, 40, 255),
        "color_bgr_rg_delta": clamp_int(percentile(rg, 5) - 8, 5, 255),
        "color_bgr_rb_delta": clamp_int(percentile(rb, 5) - 8, 5, 255),
        "color_bgr_b_max": clamp_int(percentile(b, 95) + 20, 0, 255),
        "color_min_area": float(max(120.0, percentile(areas, 10) * 0.45)),
        "color_min_area_ratio": 0.0,
        "color_max_area_ratio": 0.35,
        "color_aspect_min": 0.35,
        "color_aspect_max": 3.0,
        "color_extent_min": 0.25,
        "color_solidity_min": 0.50,
        "color_morph_kernel_size": 5,
        "color_erode_kernel_size": 5,
        "color_min_depth_mm": float(min_depth_mm),
        "color_max_depth_mm": float(max_depth_mm),
    }


def write_config(path, config):
    output_path = Path(path)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    try:
        import yaml
        with output_path.open("w", encoding="utf-8") as f:
            yaml.safe_dump(config, f, sort_keys=False, allow_unicode=False)
    except ImportError:
        with output_path.open("w", encoding="utf-8") as f:
            for key, value in config.items():
                f.write(f"{key}: {value}\n")


def draw_debug(bgr, mask, candidate, roi_rect, accepted, required):
    display = bgr.copy()
    rx1, ry1, rx2, ry2 = roi_rect
    cv2.rectangle(display, (rx1, ry1), (rx2, ry2), (255, 255, 0), 2)
    if candidate is not None:
        x, y, w, h = candidate["bbox"]
        cv2.rectangle(display, (x, y), (x + w, y + h), (0, 255, 255), 2)
    cv2.putText(
        display,
        f"samples {accepted}/{required}",
        (20, 35),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (0, 255, 255),
        2,
    )
    mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
    return np.hstack((display, mask_bgr))


def main(argv=None):
    args = parse_args(argv)
    camera = OrbbecRgbdCamera()
    accepted = 0
    bgr_samples = []
    hsv_samples = []
    lab_samples = []
    areas = []

    try:
        camera.start()
        time.sleep(0.5)
        while accepted < args.frames:
            bgr, depth_mm, _ = camera.read(timeout_ms=100)
            if bgr is None or depth_mm is None:
                continue

            height, width = bgr.shape[:2]
            roi_rect = center_roi_rect(width, height, args.center_roi_ratio)
            mask = build_loose_mask(bgr)
            candidate = choose_candidate(
                mask=mask,
                depth_mm=depth_mm,
                image_shape=bgr.shape,
                roi_rect=roi_rect,
                min_depth_mm=args.min_depth_mm,
                max_depth_mm=args.max_depth_mm,
            )

            if candidate is not None:
                bgr_pixels, hsv_pixels, lab_pixels = collect_candidate_pixels(bgr, candidate["mask"])
                if bgr_pixels.shape[0] >= 20:
                    bgr_samples.append(bgr_pixels)
                    hsv_samples.append(hsv_pixels)
                    lab_samples.append(lab_pixels)
                    areas.append(candidate["area"])
                    accepted += 1

            if args.show:
                debug = draw_debug(bgr, mask, candidate, roi_rect, accepted, args.frames)
                cv2.imshow("red threshold calibration", debug)
                key = cv2.waitKey(1) & 0xFF
                if key in (ord("q"), 27):
                    break

        if accepted < args.frames:
            print(f"ERROR: collected {accepted}/{args.frames} valid samples. Existing calibration was not overwritten.")
            return 1

        config = build_config(
            bgr_pixels=np.concatenate(bgr_samples, axis=0),
            hsv_pixels=np.concatenate(hsv_samples, axis=0),
            lab_pixels=np.concatenate(lab_samples, axis=0),
            areas=np.asarray(areas, dtype=np.float32),
            min_depth_mm=args.min_depth_mm,
            max_depth_mm=args.max_depth_mm,
        )
        write_config(args.output, config)
        print(f"Saved red color calibration: {args.output}")
        for key, value in config.items():
            print(f"{key}: {value}")
        return 0

    finally:
        try:
            camera.stop()
        except Exception:
            pass
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass


if __name__ == "__main__":
    raise SystemExit(main())
