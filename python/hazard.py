"""Red hazard marker detection for the LUNA-GAP tabletop demo.

A red object (the red printed rover lid, or any saturated red/orange piece) placed on the foam Moon
stands for a terrain hazard (boulder field, crater rim). The onboard camera looks for it, the hit is
projected onto the reference map with the same frame->map homography the position tracker uses, and
any landing zone it falls inside is marked NOT LANDABLE. The target then moves to the next clear zone.

Pure OpenCV/numpy, no App Lab imports, so it can be tested on a laptop:
    python hazard.py photo.jpg [out.jpg]
"""
from dataclasses import dataclass, field
from time import monotonic

import cv2 as cv
import numpy as np

# HSV thresholds (OpenCV hue 0..180). Red wraps around 0, so two bands.
# Tuned on the onboard IMX219 in two lightings. Dim: lid H 0-4/165-179, S 106-129, Lab a* ~17.
# Bright: lid turns orange, H 7-19, S 125-153, a* ~17. Desk/wall/ruler/paper/mirror stay below on at
# least one of: S < 100, a* < 13, or hue outside red-orange. A pixel must pass all three.
RED_BANDS = (((0, 100, 75), (16, 255, 255)), ((170, 100, 75), (180, 255, 255)))
DIM_RED_BAND = ((163, 105, 65), (179, 255, 255))
MIN_RED_BLUE_GAP = 12
MIN_A_STAR = 13                # Lab a* (redness) above neutral, 0..127 scale
MIN_AREA_FRAC = 0.005          # ignore blobs smaller than 0.5 % of the frame
CONFIRM_FRAMES = 3             # consecutive hits needed before a zone is declared blocked
MISS_FRAMES = 6                # consecutive misses before an unconfirmed hit is dropped


def red_mask(frame_bgr):
    """Binary mask (uint8 0/255) of saturated red pixels, cleaned with open/close."""
    h, w = frame_bgr.shape[:2]
    blur = cv.GaussianBlur(frame_bgr, (5, 5), 0)
    hsv = cv.cvtColor(blur, cv.COLOR_BGR2HSV)
    mask = np.zeros((h, w), np.uint8)
    for lo, hi in RED_BANDS:
        mask |= cv.inRange(hsv, np.array(lo), np.array(hi))
    # In dim frames the block shifts toward magenta. Require red to exceed
    # blue in this extended range to reject the dark purple surroundings.
    lo, hi = DIM_RED_BAND
    dim = cv.inRange(hsv, np.array(lo), np.array(hi))
    red_blue_gap = cv.subtract(blur[..., 2], blur[..., 0])
    mask |= cv.bitwise_and(dim, cv.inRange(red_blue_gap, MIN_RED_BLUE_GAP, 255))
    a_star = cv.cvtColor(blur, cv.COLOR_BGR2LAB)[..., 1]
    mask &= np.where(a_star >= 128 + MIN_A_STAR, 255, 0).astype(np.uint8)
    k = cv.getStructuringElement(cv.MORPH_ELLIPSE, (5, 5))
    mask = cv.morphologyEx(mask, cv.MORPH_OPEN, k)
    return cv.morphologyEx(mask, cv.MORPH_CLOSE, k, iterations=2)


def find_red(frame_bgr):
    """Return the largest red blob as dict(cx, cy, area, box, contour) in frame pixels, or None."""
    h, w = frame_bgr.shape[:2]
    mask = red_mask(frame_bgr)
    contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    c = max(contours, key=cv.contourArea)
    area = cv.contourArea(c)
    if area < MIN_AREA_FRAC * w * h:
        return None
    m = cv.moments(c)
    cx, cy = m["m10"] / m["m00"], m["m01"] / m["m00"]
    return {"cx": float(cx), "cy": float(cy), "area": float(area), "box": cv.boundingRect(c), "contour": c}


def to_map(H, pts):
    """Project frame pixel points (N,2) to map pixels with a 3x3 homography (frame -> map)."""
    p = np.asarray(pts, np.float32).reshape(-1, 1, 2)
    return cv.perspectiveTransform(p, np.asarray(H, np.float64)).reshape(-1, 2)


@dataclass
class Zone:
    name: str
    x: float                    # map pixels
    y: float
    r: float                    # radius in map pixels
    blocked: bool = False
    blocked_at: float = 0.0


@dataclass
class HazardMonitor:
    """Latches a zone as blocked after CONFIRM_FRAMES consecutive red hits inside it."""
    zones: list = field(default_factory=list)       # priority order: [Z1 primary, Z2 backup, ...]
    hits: int = 0
    misses: int = 0
    last: dict = None

    def set_zones(self, zones):
        self.zones = [Zone(z["name"], float(z["x"]), float(z["y"]), float(z.get("r", 40))) for z in zones]
        self.reset()

    def reset(self):
        for z in self.zones:
            z.blocked, z.blocked_at = False, 0.0
        self.hits = self.misses = 0
        self.last = None

    def update(self, frame_bgr, H=None):
        """Feed one camera frame (+ frame->map homography when the tracker has a fix)."""
        blob = find_red(frame_bgr)
        info = {"detected": blob is not None}
        if blob is None:
            self.misses += 1
            if self.misses >= MISS_FRAMES:
                self.hits = 0
        else:
            self.misses = 0
            self.hits += 1
            info.update(frame_xy=[round(blob["cx"], 1), round(blob["cy"], 1)], area_px=round(blob["area"]))
            if H is not None:
                x, y, w, h = blob["box"]
                corners = [[x, y], [x + w, y], [x + w, y + h], [x, y + h]]
                (mx, my), *box_map = to_map(H, [[blob["cx"], blob["cy"]], *corners])
                info["map_xy"] = [round(float(mx), 1), round(float(my), 1)]
                info["map_box"] = [[round(float(a), 1), round(float(b), 1)] for a, b in box_map]
                if self.hits >= CONFIRM_FRAMES:
                    for z in self.zones:
                        if not z.blocked and np.hypot(mx - z.x, my - z.y) <= z.r:
                            z.blocked, z.blocked_at = True, monotonic()
        info["confirming"] = min(self.hits, CONFIRM_FRAMES)
        self.last = info
        return self.state()

    def target(self):
        for z in self.zones:
            if not z.blocked:
                return z.name
        return None

    def state(self):
        tgt = self.target()
        if not self.zones:
            decision = "No landing zones set"
        elif tgt is None:
            decision = "NO SAFE SITE: all zones blocked"
        elif tgt == self.zones[0].name:
            decision = f"Primary {tgt} clear: continue"
        else:
            blocked = ", ".join(z.name for z in self.zones if z.blocked)
            decision = f"{blocked} NOT LANDABLE: divert to {tgt}"
        return {"hazard": self.last or {"detected": False},
                "zones": [{"name": z.name, "x": z.x, "y": z.y, "r": z.r, "blocked": z.blocked} for z in self.zones],
                "target": tgt, "decision": decision}


def draw(frame_bgr, blob):
    out = frame_bgr.copy()
    if blob:
        cv.drawContours(out, [blob["contour"]], -1, (0, 255, 255), 4)
        cv.drawMarker(out, (int(blob["cx"]), int(blob["cy"])), (0, 255, 255), cv.MARKER_CROSS, 40, 4)
        cv.putText(out, "HAZARD", (blob["box"][0], max(30, blob["box"][1] - 12)), cv.FONT_HERSHEY_SIMPLEX, 1.4, (0, 255, 255), 4)
    return out


if __name__ == "__main__":
    import sys
    img = cv.imread(sys.argv[1])
    b = find_red(img)
    print("detected:", bool(b), {k: v for k, v in (b or {}).items() if k != "contour"})
    if len(sys.argv) > 2:
        cv.imwrite(sys.argv[2], draw(img, b))
    # zone logic smoke test with an identity homography
    mon = HazardMonitor(); mon.set_zones([{"name": "Z1", "x": b["cx"], "y": b["cy"], "r": 80}, {"name": "Z2", "x": 0, "y": 0, "r": 80}] if b else [])
    for _ in range(CONFIRM_FRAMES):
        st = mon.update(img, np.eye(3))
    print(st["decision"])
