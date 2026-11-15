import cv2 as cv
import numpy as np

from hazard import red_mask

# Red hazard marker (printed red piece on the foam Moon): confirmed after this many consecutive
# matched frames, then latched on the map so the spot stays NOT LANDABLE when out of view.
RED_CONFIRM = 2
RED_MIN_FRAC = 0.005       # of the visible map area


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
        self.hazard_mask = np.zeros((self.height, self.width), np.uint8)   # latched red hazards (map view)
        self.red_hits = 0
        self.hazard = None
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
        self._update_red(frame, warp, size, visible)
        if self.hazard is not None:
            result["hazard"] = self.hazard
        x, y = int(x_norm * self.width), int(y_norm * self.height)
        in_map = 0 <= x < self.width and 0 <= y < self.height
        red_blocks_target = (in_map and self.hazard is not None
                             and self._distance(cv.bitwise_not(self.hazard_mask))[y, x] < self.clearance)
        if red_blocks_target:
            result["reason"] = "Red hazard on the landing site: NOT LANDABLE"
        if not in_map or self._distance(observed)[y, x] < self.clearance:
            if red_blocks_target:
                return {**result, "status": "blocked"}
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
        foreground = cv.bitwise_or(foreground, self.hazard_mask)
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

    def _update_red(self, frame, warp, size, visible):
        """Detect the red hazard marker, project it onto the map and latch it once confirmed."""
        red = cv.warpPerspective(red_mask(frame), warp, size, flags=cv.INTER_NEAREST)
        red = cv.bitwise_and(red, visible)
        if cv.countNonZero(red) < RED_MIN_FRAC * max(1, cv.countNonZero(visible)):
            self.red_hits = 0
            return
        self.red_hits += 1
        if self.red_hits < RED_CONFIRM:
            return
        grown = cv.dilate(red, cv.getStructuringElement(cv.MORPH_ELLIPSE, (9, 9)))
        self.hazard_mask = cv.bitwise_or(self.hazard_mask, grown)
        m = cv.moments(self.hazard_mask, binaryImage=True)
        if m["m00"]:
            self.hazard = {"type": "red_marker", "x_norm": m["m10"] / m["m00"] / self.width,
                           "y_norm": m["m01"] / m["m00"] / self.height,
                           "r_norm": float(np.sqrt(m["m00"] / np.pi)) / self.width,
                           "inside_landing_area": bool(cv.countNonZero(cv.bitwise_and(
                               self.hazard_mask, self.surveyed)))}

    def reset_hazards(self):
        self.hazard_mask[:] = 0
        self.red_hits = 0
        self.hazard = None
