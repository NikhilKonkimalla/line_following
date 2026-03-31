#!/usr/bin/env python3
"""Detect which way a blue arrow is pointing in an image."""
import sys
import numpy as np
import cv2

MIN_CONTOUR_AREA = 80  # reject tiny noise contours


def detect_direction(contour) -> str:
    """
    Given a validated arrow contour, return 'LEFT' or 'RIGHT'.
    Returns 'UNCERTAIN' if direction cannot be determined.

    Primary: approxPolyDP tip-vertex method.
    Fallback: area-split at centroid.
    """
    if contour is None or len(contour) < 5:
        return "UNCERTAIN"

    area = cv2.contourArea(contour)
    if area < MIN_CONTOUR_AREA:
        return "UNCERTAIN"

    # Centroid via moments
    M = cv2.moments(contour)
    if M["m00"] == 0:
        return "UNCERTAIN"
    cx = M["m10"] / M["m00"]
    cy = M["m01"] / M["m00"]

    # --- Primary: approxPolyDP tip-vertex method ---
    perim = cv2.arcLength(contour, True)
    approx = cv2.approxPolyDP(contour, 0.025 * perim, True)

    if len(approx) >= 3:
        # For each vertex, measure signed horizontal distance from centroid.
        # The tip is the vertex furthest from centroid in X.
        pts = approx.reshape(-1, 2)
        x_dists = pts[:, 0] - cx
        abs_x_dists = np.abs(x_dists)

        # Find the vertex with max |x - cx|
        tip_idx = np.argmax(abs_x_dists)
        tip_x_offset = x_dists[tip_idx]

        # Require the tip to be meaningfully off-center (>5% of bounding width)
        x, y, w, h = cv2.boundingRect(contour)
        if w == 0:
            return "UNCERTAIN"
        relative_offset = abs(tip_x_offset) / w

        if relative_offset > 0.05:
            return "LEFT" if tip_x_offset < 0 else "RIGHT"
        # else fall through to area-split

    # --- Fallback: area-split at centroid X ---
    x, y, w, h = cv2.boundingRect(contour)
    if w == 0 or h == 0:
        return "UNCERTAIN"

    # Draw the contour into a tight mask
    mask = np.zeros((h, w), dtype=np.uint8)
    shifted = contour.copy()
    shifted[:, :, 0] -= x
    shifted[:, :, 1] -= y
    cv2.drawContours(mask, [shifted], -1, 255, thickness=cv2.FILLED)

    # Split at centroid X (relative to bounding box)
    split_col = int(round(cx - x))
    split_col = max(1, min(split_col, w - 1))

    left_pixels = np.count_nonzero(mask[:, :split_col])
    right_pixels = np.count_nonzero(mask[:, split_col:])
    total = left_pixels + right_pixels

    if total == 0:
        return "UNCERTAIN"

    ratio = abs(left_pixels - right_pixels) / total
    # Need at least 8% asymmetry to call a direction
    if ratio < 0.08:
        return "UNCERTAIN"

    # Fewer pixels on a side = that's the tip = arrow points that way
    if left_pixels < right_pixels:
        return "LEFT"
    else:
        return "RIGHT"


def detect_arrow(image_path: str) -> str:
    frame = cv2.imread(image_path)
    if frame is None:
        return "ERROR: could not read image"

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_blue = np.array([90, 30, 200])
    upper_blue = np.array([135, 255, 255])
    mask = cv2.inRange(hsv, lower_blue, upper_blue)

    blue_pixels = np.where(mask > 0)
    if len(blue_pixels[1]) > 50:
        xs = blue_pixels[1]
        x_min, x_max = xs.min(), xs.max()
        centroid_x = (x_min + x_max) / 2.0

        left_count = np.sum(xs < centroid_x)
        right_count = np.sum(xs >= centroid_x)

        if left_count > right_count:
            return "< LEFT"
        elif right_count > left_count:
            return "RIGHT >"
        else:
            return "UNCERTAIN"
    else:
        return "NO ARROW"


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print(f"Usage: python {sys.argv[0]} <image_path>")
        sys.exit(1)
    result = detect_arrow(sys.argv[1])
    print(f"Arrow: {result}")
