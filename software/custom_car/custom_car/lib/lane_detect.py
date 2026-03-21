### FILE: custom_car/custom_car/lib/lane_detect.py
###
### DESCRIPTION: Pure lane detection functions — no ROS dependencies.
###              All functions are stateless transforms: frame in, result out.
###              State (last_lane_x, threshold) lives in the calling node.
### USAGE:
###     result = detect_lane(frame, threshold=80, last_lane_x=320)
###     if result.found:
###         error = result.lane_x - screen_center


from dataclasses import dataclass
from typing import Optional
import cv2
import numpy as np
from scipy.signal import find_peaks


# ------------------------------------------------------------------
# Public types
# ------------------------------------------------------------------

@dataclass
class LaneResult:
    """Output of a single lane detection pass."""
    found:  bool
    lane_x: int                               # X-pixel of detected lane (or last known if not found)
    debug_frame: Optional[np.ndarray] = None  # Side-by-side ROI + binary, for streaming


@dataclass
class LaneConfig:
    """Tunable parameters for lane detection."""
    threshold: int = 80             # Brightness cutoff; tape is dark, floor is light
    roi_start_frac: float = 0.66    # Bottom fraction of frame used as ROI
    peak_min_height: int = 1000     # Minimum histogram column sum to count as a peak
    peak_min_distance: int = 80     # Minimum horizontal distance between peaks (px)
    flicker_limit: int = 100        # Max frame-to-frame lane jump before it's rejected


# ------------------------------------------------------------------
# Public API
# ------------------------------------------------------------------

def detect_lane(
    frame: np.ndarray,
    config: LaneConfig,
    last_lane_x: int,
) -> LaneResult:
    """
    Detect the nearest lane line in the bottom ROI of a BGR frame.

    Strategy:
        1. Grayscale + Gaussian blur to suppress floor texture.
        2. Binary threshold (inverted) to isolate dark tape.
        3. Column histogram of the ROI to find bright vertical bands.
        4. Peak detection; pick the peak closest to last_lane_x (memory filter).
        5. Reject jumps larger than config.flicker_limit (anti-flicker).

    Args:
        frame:       BGR image from cv2.VideoCapture.
        config:      Tunable detection parameters.
        last_lane_x: X position of the lane from the previous frame.

    Returns:
        LaneResult with found=True and the new lane_x if a lane was detected,
        or found=False and last_lane_x as a fallback (memory mode).
    """
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (9, 9), 0)

    h, w = blur.shape
    roi_y = int(h * config.roi_start_frac)
    roi = blur[roi_y:h, :]

    _, binary = cv2.threshold(roi, config.threshold, 255, cv2.THRESH_BINARY_INV)

    histogram = np.sum(binary, axis=0)
    peaks, _ = find_peaks(
        histogram,
        height=config.peak_min_height,
        distance=config.peak_min_distance,
    )

    debug = _build_debug_frame(frame, binary, roi_y)

    if len(peaks) == 0:
        return LaneResult(found=False, lane_x=last_lane_x, debug_frame=debug)

    # Pick the peak closest to where we last saw the lane
    best_peak = int(min(peaks, key=lambda x: abs(x - last_lane_x)))

    # Reject if the jump is implausibly large (flicker suppression)
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
    """
    Resize and annotate the debug frame for the web dashboard feed.

    Returns a (height x 640) BGR image ready to encode as JPEG.
    """
    if frame is None or frame.size == 0:
        return np.zeros((height, 640, 3), dtype=np.uint8)
    return cv2.resize(frame, (640, height))


# ------------------------------------------------------------------
# Internal helpers
# ------------------------------------------------------------------

def _build_debug_frame(
    original: np.ndarray,
    binary_roi: np.ndarray,
    roi_y: int,
) -> np.ndarray:
    """Side-by-side: raw ROI crop (left) | binary mask (right)."""
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
    """Draw the detected lane line onto the debug frame in-place."""
    roi_h = frame_h - roi_y
    # Green line on the raw crop (left half)
    cv2.line(debug, (lane_x, 0), (lane_x, roi_h), (0, 255, 0), 2)
    # Red line on the binary mask (right half)
    w = debug.shape[1] // 2
    cv2.line(debug, (w + lane_x, 0), (w + lane_x, roi_h), (0, 0, 255), 2)