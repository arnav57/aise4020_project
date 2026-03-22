from dataclasses import dataclass
from typing import Optional

import cv2
import numpy as np
from scipy.signal import find_peaks

@dataclass
class LaneResult:
    found: bool
    lane_x: int
    debug_frame: Optional[np.ndarray] = None


@dataclass
class LaneConfig:
    roi_start_frac: float = 0.70
    peak_min_height: int = 1000
    peak_min_distance: int = 80
    peak_min_width: int = 5        # minimum tape width in pixels
    flicker_limit: int = 100
    adaptive_block: int = 31       # neighbourhood size for adaptive threshold
    adaptive_c: int = 8            # how much darker than neighbourhood = tape


def detect_lane(
    frame: np.ndarray,
    config: LaneConfig,
    last_lane_x: int,
) -> LaneResult:
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (9, 9), 0)

    h, w = blur.shape
    roi_y = int(h * config.roi_start_frac)
    roi = blur[roi_y:h, :]

    binary = cv2.adaptiveThreshold(
        roi,
        255,
        cv2.ADAPTIVE_THRESH_MEAN_C,
        cv2.THRESH_BINARY_INV,
        config.adaptive_block,
        config.adaptive_c,
    )

    kernel = np.ones((3, 3), np.uint8)
    binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)

    histogram = np.sum(binary, axis=0)

    peaks, properties = find_peaks(
        histogram,
        height=config.peak_min_height,
        distance=config.peak_min_distance,
        width=config.peak_min_width,
    )

    debug = _build_debug_frame(frame, binary, roi_y)

    if len(peaks) == 0:
        return LaneResult(found=False, lane_x=last_lane_x, debug_frame=debug)

    best_peak = int(min(peaks, key=lambda x: abs(x - last_lane_x)))

    if abs(best_peak - last_lane_x) > config.flicker_limit:
        return LaneResult(found=False, lane_x=last_lane_x, debug_frame=debug)

    _draw_detection(debug, best_peak, roi_y, h)
    return LaneResult(found=True, lane_x=best_peak, debug_frame=debug)


def build_debug_overlay(
    frame: np.ndarray,
    lane_x: int,
    screen_center: int,
    height: int = 160,
) -> np.ndarray:
    if frame is None or frame.size == 0:
        return np.zeros((height, 640, 3), dtype=np.uint8)
    return cv2.resize(frame, (640, height))


def _build_debug_frame(
    original: np.ndarray,
    binary_roi: np.ndarray,
    roi_y: int,
) -> np.ndarray:
    h = original.shape[0]
    raw_crop = original[roi_y:h, :].copy()
    binary_bgr = cv2.cvtColor(binary_roi, cv2.COLOR_GRAY2BGR)
    return np.hstack((raw_crop, binary_bgr))


def _draw_detection(
    debug: np.ndarray,
    lane_x: int,
    roi_y: int,
    frame_h: int,
) -> None:
    roi_h = frame_h - roi_y
    cv2.line(debug, (lane_x, 0), (lane_x, roi_h), (0, 255, 0), 2)
    w = debug.shape[1] // 2
    cv2.line(debug, (w + lane_x, 0), (w + lane_x, roi_h), (0, 0, 255), 2)