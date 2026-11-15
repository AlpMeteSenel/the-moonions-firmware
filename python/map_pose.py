from threading import Lock
from time import monotonic


FRESH_SECONDS = 1.25
HEADING_HOLD_SECONDS = 3.0
HEIGHT_CAMERA_FRESH_SECONDS = 0.35
HEIGHT_IMU_HOLD_SECONDS = 2.0
IMU_HEADING_WEIGHT = 0.7
# Map angles increase clockwise from image-up. Rover yaw is positive around +Z.
IMU_TO_MAP_YAW = -1


def wrap_angle(angle):
    return (angle + 180) % 360 - 180


class MapPose:
    """Camera position with short inertial propagation of heading and height."""

    def __init__(self):
        self.lock = Lock()
        self.fix = None
        self.tentative = None
        self.height_fix = None

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
                self.height_fix = (captured_at, height, velocity, imu.get("epoch"), motion)

    def _vertical_speed(self, height, captured_at):
        if self.height_fix:
            previous_at, previous_height, _, _, _ = self.height_fix
            dt = captured_at - previous_at
            if 0.08 <= dt <= 2:
                speed = (height - previous_height) / dt
                if abs(speed) <= 2:
                    return speed
        return 0.0

    def _height(self, imu, now):
        if not self.height_fix:
            return None, None, None
        captured_at, height, velocity, epoch, anchor = self.height_fix
        age = max(0, now - captured_at)
        age_ms = round(age * 1000)
        if age <= HEIGHT_CAMERA_FRESH_SECONDS:
            return height, "camera", age_ms
        if (age <= HEIGHT_IMU_HOLD_SECONDS and anchor is not None
                and imu["status"] == "ready" and imu["epoch"] == epoch):
            current = imu.get("vertical_motion")
            if current is not None:
                dt = current[0] - anchor[0]
                if 0 < dt <= HEIGHT_IMU_HOLD_SECONDS:
                    displacement = current[2] - anchor[2] + (velocity - anchor[1]) * dt
                    predicted_height = height + displacement
                    if predicted_height >= 0:
                        return round(predicted_height, 3), "imu", age_ms
        return height, "last_camera", age_ms

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
            height, height_source, height_age_ms = self._height(imu, now)
            return {
                "tracking_status": status,
                "x_px": visual["x"],
                "y_px": visual["y"],
                "z_m": height,
                "height_source": height_source,
                "height_age_ms": height_age_ms,
                "frame_age_ms": round(max(0, now - captured_at) * 1000),
                "heading_deg": None if heading is None else round(heading, 1),
                "heading_source": source,
            }
