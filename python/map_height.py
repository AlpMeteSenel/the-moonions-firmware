import numpy as np


def estimate_height(image_to_map, meters_per_pixel, intrinsics, image_shape, capture_shape):
    """Return distance from the camera center to a metric map plane."""
    fx, fy, cx, cy = intrinsics
    sx = image_shape[1] / capture_shape[1]
    sy = image_shape[0] / capture_shape[0]
    camera = np.array([
        [sx * fx, 0, sx * (cx + 0.5) - 0.5],
        [0, sy * fy, sy * (cy + 0.5) - 0.5],
        [0, 0, 1],
    ])
    if not np.isfinite(camera).all() or fx <= 0 or fy <= 0:
        return None
    if not np.isfinite(meters_per_pixel) or meters_per_pixel <= 0:
        return None
    try:
        metric_to_map = np.diag([1 / meters_per_pixel, 1 / meters_per_pixel, 1])
        normalized = np.linalg.solve(camera, np.linalg.solve(image_to_map, metric_to_map))
    except np.linalg.LinAlgError:
        return None
    axis_x, axis_y, translation = normalized.T
    length_x, length_y = np.linalg.norm(axis_x), np.linalg.norm(axis_y)
    if not np.isfinite(normalized).all() or min(length_x, length_y) == 0:
        return None
    # Reject transforms far from the geometry of a calibrated rigid camera.
    if max(length_x, length_y) > 1.25 * min(length_x, length_y):
        return None
    if abs(np.dot(axis_x / length_x, axis_y / length_y)) > 0.2:
        return None
    normal = np.cross(axis_x, axis_y)
    normal /= np.linalg.norm(normal)
    height = abs(np.dot(normal, translation)) / ((length_x + length_y) / 2)
    return round(float(height), 3) if np.isfinite(height) and height > 0 else None
