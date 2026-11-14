import cv2 as cv
import numpy as np


# Medians measured in four gray patches of the user's chart, away from glare.
# Green is the reference channel. Keep black and white endpoints fixed.
_GRAY_PATCHES_BGR = np.array([
    [116, 81, 77],
    [130, 98, 94],
    [140, 112, 107],
    [162, 139, 134],
])
_LEVELS = np.arange(256)
_NEUTRAL_LEVELS = np.r_[0, _GRAY_PATCHES_BGR[:, 1], 255]
_COLOR_TABLE = np.column_stack([
    np.interp(_LEVELS, np.r_[0, channel, 255], _NEUTRAL_LEVELS)
    for channel in _GRAY_PATCHES_BGR.T
]).round().astype(np.uint8).reshape(256, 1, 3)


def balance_colors(image):
    return cv.LUT(image, _COLOR_TABLE)


def encode_camera_frame(frame, correct_colors):
    # Arduino's camera API returns BGR, BGRA, or grayscale arrays.
    image = cv.cvtColor(frame, cv.COLOR_BGRA2BGR) if frame.ndim == 3 and frame.shape[2] == 4 else frame
    if correct_colors and image.ndim == 3 and image.shape[2] == 3:
        image = balance_colors(image)
    ok, jpeg = cv.imencode(".jpg", image, [cv.IMWRITE_JPEG_QUALITY, 92])
    if not ok:
        raise RuntimeError("Impossibile codificare il fotogramma della camera.")
    return jpeg.tobytes()
