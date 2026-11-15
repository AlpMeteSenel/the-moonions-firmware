from threading import Lock
from time import monotonic


FRESH_SECONDS = 1.25
HEADING_HOLD_SECONDS = 3.0
IMU_HEADING_WEIGHT = 0.7
# Map angles increase clockwise from image-up. Rover yaw is positive around +Z.
IMU_TO_MAP_YAW = -1


def wrap_angle(angle):
    return (angle + 180) % 360 - 180


class MapPose:
    """Camera position with camera-corrected inertial heading."""

    def __init__(self):
        self.lock = Lock()
        self.fix = None
        self.tentative = None

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
            anchor = (imu["epoch"], imu["yaw"]) if imu["status"] == "ready" else None
            self.fix = (captured_at, visual, heading, anchor)
            self.tentative = None

    def _heading(self, imu, now):
        if not self.fix or now - self.fix[0] > HEADING_HOLD_SECONDS:
            return None, None
        _, _, heading, anchor = self.fix
        if anchor and imu["status"] == "ready" and imu["epoch"] == anchor[0]:
            yaw_delta = wrap_angle(imu["yaw"] - anchor[1])
            return wrap_angle(heading + IMU_TO_MAP_YAW * yaw_delta), "camera_imu"
        if now - self.fix[0] <= FRESH_SECONDS:
            return heading, "camera"
        return None, None

    def snapshot(self, imu, now=None):
        if now is None:
            now = monotonic()
        with self.lock:
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
                return {"tracking_status": "waiting"}
            heading, source = self._heading(imu, now)
            if status == "tentative":
                heading, source = visual["angle"], "camera"
            return {
                "tracking_status": status,
                "x_px": visual["x"],
                "y_px": visual["y"],
                "frame_age_ms": round(max(0, now - captured_at) * 1000),
                "heading_deg": None if heading is None else round(heading, 1),
                "heading_source": source,
            }
