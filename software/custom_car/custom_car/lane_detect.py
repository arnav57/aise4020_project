import cv2
import numpy as np
from scipy.signal import find_peaks

# def find_lanes(frame, threshold_val=60, last_lane_x=320, expected_lanes=1):
#     # 1. AGGRESSIVE Pre-processing (Kill the wood grain)
#     # Use a massive 15x15 kernel to melt the floor texture into a solid color
#     blur = cv2.GaussianBlur(frame, (15, 15), 0)

#     # 2. HSV Color Space
#     hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
    
#     # 3. Extract the 'Value' (brightness) channel
#     v_channel = hsv[:, :, 2]

#     # Set the Region of Interest (Bottom third)
#     h, w = v_channel.shape
#     start_y = int(h * 0.66)
#     roi = v_channel[start_y:h, :]

#     # 4. Binary Thresholding on the smoothed Value channel
#     # Look for very dark pixels (Value < threshold_val)
#     _, binary = cv2.threshold(roi, threshold_val, 255, cv2.THRESH_BINARY_INV)

#     # 5. Morphological Sandpaper (Clean up the edges)
#     # A 7x7 kernel will destroy any lingering wood knots or floor scuffs
#     kernel = np.ones((7,7), np.uint8)
#     # 'Open' removes white noise specs on the black background
#     binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)
#     # 'Close' fills in small black holes inside our white tape line (like glare)
#     binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)

#     # 6. Histogram and Peak Detection
#     histogram = np.sum(binary, axis=0)
    
#     # We can increase the height requirement now because the signal is so clean
#     peaks, _ = find_peaks(histogram, height=1500, distance=80) 

#     if len(peaks) > 0:
#         # Sort by proximity to kill the bistable flicker
#         peaks_sorted = sorted(peaks, key=lambda x: abs(x - last_lane_x))
#         detected_lanes = sorted(peaks_sorted[:expected_lanes])
#     else:
#         detected_lanes = []

#     # 7. Visual Debugging
#     raw_roi = frame[start_y:h, :].copy()
#     debug_roi = cv2.cvtColor(binary, cv2.COLOR_GRAY2BGR)
    
#     for x in detected_lanes:
#         cv2.line(raw_roi, (x, 0), (x, h - start_y), (0, 255, 0), 3)
#         cv2.line(debug_roi, (x, 0), (x, h - start_y), (0, 0, 255), 3)

#     combined_img = np.hstack((raw_roi, debug_roi))

#     return combined_img, detected_lanes

import cv2
import numpy as np
from scipy.signal import find_peaks

def find_lanes(frame, threshold_val=80, last_lane_x=320, expected_lanes=1):
    # 1. Grayscale is all we need (Black tape vs White/Gray floor is a pure intensity difference)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # 2. Gaussian Blur to smooth out any rough concrete texture or dust
    blur = cv2.GaussianBlur(gray, (9, 9), 0)

    # 3. Region of Interest (Bottom third of the camera)
    h, w = blur.shape
    start_y = int(h * 0.66)
    roi = blur[start_y:h, :]

    # 4. Simple Thresholding
    # Tape is usually ~0-50 brightness. Concrete is ~100-150. White table is ~200-255.
    # Setting your web UI slider to ~70 or 80 will perfectly isolate the tape.
    _, binary = cv2.threshold(roi, threshold_val, 255, cv2.THRESH_BINARY_INV)

    # 5. Histogram and Peak Detection
    histogram = np.sum(binary, axis=0)
    
    peaks, _ = find_peaks(histogram, height=1000, distance=80) 

    if len(peaks) > 0:
        # Sort by proximity to the memory variable (This makes your lane switching work!)
        peaks_sorted = sorted(peaks, key=lambda x: abs(x - last_lane_x))
        detected_lanes = sorted(peaks_sorted[:expected_lanes])
    else:
        detected_lanes = []

    # 6. Visual Debugging
    raw_roi = frame[start_y:h, :].copy()
    debug_roi = cv2.cvtColor(binary, cv2.COLOR_GRAY2BGR)
    
    for x in detected_lanes:
        cv2.line(raw_roi, (x, 0), (x, h - start_y), (0, 255, 0), 3)
        cv2.line(debug_roi, (x, 0), (x, h - start_y), (0, 0, 255), 3)

    combined_img = np.hstack((raw_roi, debug_roi))

    return combined_img, detected_lanes