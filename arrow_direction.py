#!/usr/bin/env python3
"""Detect which way a blue arrow is pointing in an image."""
import sys
import numpy as np
import cv2


def detect_arrow(image_path: str) -> str:
    frame = cv2.imread(image_path)
    if frame is None:
        return "ERROR: could not read image"

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_blue = np.array([90, 30, 10])
    upper_blue = np.array([135, 255, 150])
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
