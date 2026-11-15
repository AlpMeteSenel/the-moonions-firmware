from dataclasses import dataclass
from threading import Lock
from time import monotonic


FRESH_SECONDS = 1.25
HEADING_HOLD_SECONDS = 3.0
HEIGHT_CAMERA_FRESH_SECONDS = 0.35
HEIGHT_IMU_HOLD_SECONDS = 4.0
IMU_HEADING_WEIGHT = 0.7
CAMERA_HEIGHT_WEIGHT = 0.7
# Map angles increase clockwise from image-up. Rover yaw is positive around +Z.
IMU_TO_MAP_YAW = -1


def wrap_angle(angle):
    return (angle + 180) % 360 - 180


@dataclass
class HeightAnchor:
    time: float
    height: float
    velocity: float
    epoch: int | None
    motion: tuple | None
    source: str
    camera_height: float | None = None


class MapPose:
    """Camera position with short inertial propagation of heading and height."""

    def __init__(self):
        self.lock = Lock()
        self.fix = None
        self.tentative = None
        self.height_fix = None
        self.last_imu_height = None

    def seed_height(self, height, imu, now=None):
        if imu["status"] != "ready" or imu.get("vertical_motion") is None:
            return False
        with self.lock:
            self.height_fix = HeightAnchor(
                monotonic() if now is None else now, height, 0.0,
                imu["epoch"], imu["vertical_motion"], "initial",
            )
            self.last_imu_height = None
        return True

    def update(self, visual, captured_at, imu):
        with self.lock:
            if visual["tentative"]:
                self.tentative = (captured_at, visual)
                return
            heading, source = self._heading(imu, captured_at)
            if source == "camera_imu":
                heading = wrap_angle(heading + (1 - IMU_HEADING_WEIGHT) *
                                     wrap_angle(visual["angle"] - heading))
            else:
                heading = visual["angle"]
            anchor = (imu["epoch"], imu["yaw"], imu["sampled_at"]) if imu["status"] == "ready" else None
            self.fix = (captured_at, visual, heading, anchor)
            self.tentative = None
            height = visual.get("z_m")
            if height is not None:
                velocity = self._vertical_speed(height, captured_at)
                motion = imu.get("vertical_motion") if imu["status"] == "ready" else None
                predicted = self._predict_height(imu, captured_at)
                corrected = height if predicted is None else (
                    predicted + CAMERA_HEIGHT_WEIGHT * (height - predicted)
                )
                self.height_fix = HeightAnchor(
                    captured_at, corrected, velocity, imu.get("epoch"), motion, "camera", height,
                )
                self.last_imu_height = None

    def _vertical_speed(self, height, captured_at):
        if self.height_fix and self.height_fix.camera_height is not None:
            dt = captured_at - self.height_fix.time
            if 0.08 <= dt <= 2:
                speed = (height - self.height_fix.camera_height) / dt
                if abs(speed) <= 2:
                    return speed
        return 0.0

    def update_imu(self, imu, now=None):
        if now is None:
            now = monotonic()
        with self.lock:
            if self.height_fix is None or imu["status"] != "ready":
                return
            if imu.get("lander_status") == "landed" and self.height_fix.source != "initial":
                return
            height = self._predict_height(imu, now)
            if height is not None:
                self.last_imu_height = (height, imu["sampled_at"])

    def _predict_height(self, imu, now):
        fix = self.height_fix
        if (fix is None or imu["status"] != "ready" or fix.motion is None
                or not 0 <= now - fix.time <= HEIGHT_IMU_HOLD_SECONDS or imu["epoch"] != fix.epoch):
            return None
        anchor = fix.motion
        current = imu.get("vertical_motion")
        if current is None:
            return None
        dt = current[0] - anchor[0]
        if not 0 < dt <= HEIGHT_IMU_HOLD_SECONDS:
            return None
        displacement = current[2] - anchor[2] + (fix.velocity - anchor[1]) * dt
        height = fix.height + displacement
        return round(height, 3) if height >= 0 else None

    def _height(self, imu, now):
        if self.height_fix is None:
            return None, None, None, None
        fix = self.height_fix
        age = max(0, now - fix.time)
        age_ms = round(age * 1000)
        if age <= HEIGHT_CAMERA_FRESH_SECONDS:
            return round(fix.height, 3), fix.source, age_ms, age_ms
        if self.last_imu_height is not None:
            estimate, sampled_at = self.last_imu_height
            estimate_age_ms = round(max(0, now - sampled_at) * 1000)
            fresh = (age <= HEIGHT_IMU_HOLD_SECONDS and estimate_age_ms <= 250
                     and imu["status"] == "ready" and imu["epoch"] == fix.epoch
                     and (imu.get("lander_status") != "landed" or fix.source == "initial"))
            return estimate, "imu" if fresh else "last_imu", age_ms, estimate_age_ms
        return round(fix.height, 3), "last_" + fix.source, age_ms, age_ms

    def _heading(self, imu, now):
        if not self.fix or now - self.fix[0] > HEADING_HOLD_SECONDS:
            return None, None
        _, _, heading, anchor = self.fix
        if anchor and imu["status"] == "ready" and imu["epoch"] == anchor[0]:
            yaw_delta = wrap_angle(imu["yaw"] - anchor[1])
            if imu["sampled_at"] > anchor[2]:
                return wrap_angle(heading + IMU_TO_MAP_YAW * yaw_delta), "camera_imu"
        if now - self.fix[0] <= FRESH_SECONDS:
            return heading, "camera"
        return None, None

    def snapshot(self, imu, now=None):
        if now is None:
            now = monotonic()
        with self.lock:
            height, height_source, height_age_ms, estimate_age_ms = self._height(imu, now)
            height_report = {
                "z_m": height,
                "height_source": height_source,
                "height_age_ms": height_age_ms,
                "height_estimate_age_ms": estimate_age_ms,
            }
            if self.fix and now - self.fix[0] <= FRESH_SECONDS:
                captured_at, visual, _, _ = self.fix
                status = "valid"
            elif self.tentative and now - self.tentative[0] <= FRESH_SECONDS:
                captured_at, visual = self.tentative
                status = "tentative"
            elif self.fix:
                captured_at, visual, _, _ = self.fix
                status = "stale"
            else:
                return {"tracking_status": "waiting", **height_report}
            heading, source = self._heading(imu, now)
            if status == "tentative":
                heading, source = visual["angle"], "camera"
            return {
                "tracking_status": status,
                "x_px": visual["x"],
                "y_px": visual["y"],
                **height_report,
                "frame_age_ms": round(max(0, now - captured_at) * 1000),
                "heading_deg": None if heading is None else round(heading, 1),
                "heading_source": source,
            }
