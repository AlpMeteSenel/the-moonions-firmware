import cv2 as cv
import numpy as np


class LandingZone:
    """Compare the visible, surveyed landing area with its clear reference."""

    def __init__(self, reference, settings):
        if not isinstance(settings, dict):
            raise ValueError("Invalid landing-zone settings.")
        try:
            left, top, right, bottom = map(float, settings["area"])
            diameter = float(settings["footprint"])
        except (KeyError, TypeError, ValueError, OverflowError):
            raise ValueError("Select a clear landing area and enter a footprint size.") from None
        if not (0 <= left < right <= 1 and 0 <= top < bottom <= 1 and 0.02 <= diameter <= 0.5):
            raise ValueError("Use a landing footprint between 2% and 50% of the map width.")

        height, width = reference.shape[:2]
        scale = min(1, 640 / max(width, height))
        reference_view = cv.resize(reference, None, fx=scale, fy=scale, interpolation=cv.INTER_AREA)
        self.height, self.width = reference_view.shape[:2]
        self.map_scale = np.diag([self.width / width, self.height / height, 1])
        self.reference_lab = self._lab(reference_view)
        self.area = (left, top, right, bottom)
        self.footprint = diameter
        self.radius = int(np.ceil(diameter * self.width / 2))
        # Keep a few pixels beyond the footprint clear of obstacles and the observed edge.
        self.clearance = self.radius + 3
        self.surveyed = np.zeros((self.height, self.width), np.uint8)
        self.surveyed[int(np.ceil(top * self.height)):int(bottom * self.height),
                      int(np.ceil(left * self.width)):int(right * self.width)] = 255
        self.kernel = cv.getStructuringElement(cv.MORPH_ELLIPSE, (5, 5))
        if self._distance(self.surveyed).max() < self.clearance:
            raise ValueError("The marked landing area is too small for the footprint.")

    @staticmethod
    def _lab(image):
        return cv.cvtColor(cv.GaussianBlur(image, (5, 5), 0), cv.COLOR_BGR2LAB).astype(np.int16)

    @staticmethod
    def _distance(mask):
        # Treat the edge of the map as unobserved.
        padded = cv.copyMakeBorder(mask, 1, 1, 1, 1, cv.BORDER_CONSTANT, value=0)
        return cv.distanceTransform(padded, cv.DIST_L2, 5)[1:-1, 1:-1]

    def inspect(self, frame, frame_to_map, x_norm, y_norm):
        target = {"x_norm": x_norm, "y_norm": y_norm}
        result = {"status": "unknown", "target": target, "alternative": None}
        size = (self.width, self.height)
        warp = self.map_scale @ frame_to_map
        visible = cv.warpPerspective(np.full(frame.shape[:2], 255, np.uint8), warp, size,
                                     flags=cv.INTER_NEAREST)
        visible = cv.erode(visible, self.kernel, borderType=cv.BORDER_CONSTANT, borderValue=0)
        observed = cv.bitwise_and(visible, self.surveyed)
        x, y = int(x_norm * self.width), int(y_norm * self.height)
        if (not 0 <= x < self.width or not 0 <= y < self.height
                or self._distance(observed)[y, x] < self.clearance):
            return {**result, "reason": "The full footprint is outside the visible, marked area."}

        aligned = cv.warpPerspective(frame, warp, size)
        difference = self._lab(aligned) - self.reference_lab
        # Correct small global lighting shifts. Large local changes remain obstacles.
        sampled = difference[::4, ::4][observed[::4, ::4] != 0]
        offsets = np.clip(np.median(sampled, axis=0), [-16, -8, -8], [16, 8, 8])
        change = np.abs(difference - offsets)
        foreground = ((change[..., 0] >= 26) | (np.max(change[..., 1:], axis=2) >= 20))
        foreground = np.uint8(foreground & (observed != 0)) * 255
        foreground = cv.morphologyEx(foreground, cv.MORPH_OPEN, self.kernel)
        clearance = self._distance(cv.bitwise_and(observed, cv.bitwise_not(foreground)))
        if clearance[y, x] >= self.clearance:
            return {**result, "status": "clear"}

        candidates_y, candidates_x = np.nonzero(clearance >= self.clearance)
        if candidates_x.size:
            nearest = np.argmin((candidates_x - x) ** 2 + (candidates_y - y) ** 2)
            result["alternative"] = {
                "x_norm": (int(candidates_x[nearest]) + 0.5) / self.width,
                "y_norm": (int(candidates_y[nearest]) + 0.5) / self.height,
            }
        return {**result, "status": "blocked"}
