import base64
import math
from hashlib import sha256
from threading import Event, Lock, Thread
from time import monotonic, sleep
from urllib.request import urlopen

import cv2 as cv
import numpy as np
from fastapi.responses import JSONResponse

from arduino.app_bricks.web_ui import WebUI
from arduino.app_peripherals.camera import Camera
from arduino.app_utils import App, Bridge
from camera_color import encode_camera_frame
from imu import Attitude
from map_height import estimate_height
from map_pose import MapPose
from matching import MapMatcher, draw_projection

# Optional. Edit in App Lab, then Stop and Run again.
MAP_URL = ""
# Bench map download (7).jpg. The upper-right 30 mm wheel spans about 97 px.
# The reference was captured at roughly 0.75 m, assuming a near-overhead view.
BENCH_MAP_METERS_PER_PIXEL = 0.030 / 97
BENCH_FOCAL_LENGTH_PX = 0.75 / BENCH_MAP_METERS_PER_PIXEL
HEIGHT_CALIBRATION = (
    "678543fb811057ac309efc4dd657305943b579091c46f165649e5126ccbe424b",
    BENCH_MAP_METERS_PER_PIXEL,
    (BENCH_FOCAL_LENGTH_PX, BENCH_FOCAL_LENGTH_PX, 639.5, 479.5),
)
cv.setNumThreads(2)
CAPTURE_RESOLUTION = (1280, 960)
MAX_IMAGE_BYTES = 12 * 1024 * 1024
ui = WebUI()
camera_lock = Lock()
attitude = Attitude()
shown_lander_status = None


def decode_image(data):
    if not data or len(data) > MAX_IMAGE_BYTES:
        raise ValueError("Usa una foto JPG o PNG entro 12 MB.")
    image = cv.imdecode(np.frombuffer(data, np.uint8), cv.IMREAD_COLOR)
    if image is None:
        raise ValueError("Immagine illeggibile. Usa JPG o PNG.")
    return image


def jpeg_url(data):
    return "data:image/jpeg;base64," + base64.b64encode(data).decode("ascii")


def capture_frame(correct_colors):
    with camera_lock:
        camera = Camera(resolution=CAPTURE_RESOLUTION)
        try:
            camera.start()
            start = monotonic()
            while monotonic() - start < 5:
                frame = camera.capture()
                if frame is not None and monotonic() - start >= 2.0:
                    return encode_camera_frame(frame, correct_colors)
                sleep(0.03)
            raise RuntimeError("La camera non ha restituito un fotogramma.")
        finally:
            camera.stop()


def capture(correct_colors: bool = True):
    try:
        return {"frame": jpeg_url(capture_frame(correct_colors))}
    except Exception as error:
        return JSONResponse({"error": str(error)}, status_code=503)


def capture_landing(payload: dict, correct_colors: bool = True):
    try:
        reference = decode_image(base64.b64decode(payload["map"], validate=True))
        frames = []
        with camera_lock:
            camera = Camera(resolution=CAPTURE_RESOLUTION)
            try:
                camera.start()
                start = monotonic()
                for index in range(8):
                    sleep(max(0, start + 0.6 + index * 0.55 - monotonic()))
                    frame = camera.capture()
                    if frame is not None:
                        timestamp = monotonic() - start
                        frames.append((timestamp, encode_camera_frame(frame, correct_colors)))
            finally:
                camera.stop()

        if not frames:
            raise ValueError("La camera non ha restituito fotogrammi.")
        matcher = MapMatcher(reference)
        candidates = []
        previews = []
        diagnostics = []
        for timestamp, data in frames:
            if timestamp < 2.7:
                continue
            try:
                overlay, estimate = matcher.locate(decode_image(data), allow_tentative=True)
                result = (timestamp, data, overlay, estimate)
                previews.append(result)
                if not estimate["tentative"]:
                    candidates.append(result)
            except ValueError as error:
                diagnostics.append(f"{timestamp:.1f}s: {error}")
        tolerance = max(12, 0.025 * math.hypot(*reference.shape[:2]))
        for index in range(len(candidates) - 1, 0, -1):
            timestamp, data, overlay, last = candidates[index]
            if frames[-1][0] - timestamp > 0.8:
                break
            previous = candidates[index - 1][3]
            distance = math.hypot(last["x"] - previous["x"], last["y"] - previous["y"])
            angle_difference = abs((last["angle"] - previous["angle"] + 180) % 360 - 180)
            if timestamp - candidates[index - 1][0] <= 1.3 and distance <= tolerance and angle_difference <= 10:
                ok, jpeg = cv.imencode(".jpg", overlay)
                if not ok:
                    raise ValueError("Impossibile creare la mappa del risultato.")
                return {**last, "overlay": jpeg_url(jpeg.tobytes()), "frame": jpeg_url(data),
                        "valid": len(candidates), "total": len(frames), "confirmed": True}
        if previews:
            # Favor evidence close to the end over an older view from the descent.
            recent = [result for result in previews if result[0] >= previews[-1][0] - 0.8]
            timestamp, data, overlay, estimate = max(recent, key=lambda result: (
                not result[3]["tentative"], result[3]["inliers"], result[0]))
            if not estimate["tentative"]:
                # The geometry passed, but the sequence did not confirm it.
                green = np.all(overlay == (70, 255, 50), axis=2)
                overlay[green] = (0, 185, 255)
            ok, jpeg = cv.imencode(".jpg", overlay)
            if not ok:
                raise ValueError("Impossibile creare la mappa del risultato.")
            return {**estimate, "tentative": True, "confirmed": False,
                    "overlay": jpeg_url(jpeg.tobytes()), "frame": jpeg_url(data),
                    "timestamp": round(timestamp, 1), "valid": len(candidates), "total": len(frames)}
        return {"confirmed": False, "valid": len(candidates), "total": len(frames),
                "frame": jpeg_url(frames[-1][1]), "diagnostics": diagnostics}
    except (ValueError, KeyError, TypeError, RuntimeError, cv.error) as error:
        return JSONResponse({"error": str(error)}, status_code=422)


def downloaded_map():
    if not MAP_URL:
        return JSONResponse({"error": "Imposta MAP_URL in python/main.py, oppure carica una foto."}, 400)
    try:
        with urlopen(MAP_URL, timeout=8) as response:
            data = response.read(MAX_IMAGE_BYTES + 1)
        image = decode_image(data)
        ok, jpeg = cv.imencode(".jpg", image)
        if not ok:
            raise RuntimeError("Impossibile ricodificare la mappa.")
        return {"map": jpeg_url(jpeg.tobytes())}
    except Exception as error:
        return JSONResponse({"error": str(error)}, status_code=503)


def match(payload: dict):
    try:
        reference = decode_image(base64.b64decode(payload["map"], validate=True))
        frame = decode_image(base64.b64decode(payload["frame"], validate=True))
        overlay, estimate = MapMatcher(reference).locate(frame, allow_tentative=True)
        ok, jpeg = cv.imencode(".jpg", overlay)
        if not ok:
            raise RuntimeError("Impossibile creare l'immagine del risultato.")
        return {**estimate, "overlay": jpeg_url(jpeg.tobytes())}
    except (ValueError, KeyError, TypeError, cv.error) as error:
        return JSONResponse({"error": str(error)}, status_code=422)


class LiveSession:
    def __init__(self, reference, correct_colors, map_id):
        self.matcher = MapMatcher(reference, live=True)
        height, width = reference.shape[:2]
        self.map_info = {"id": map_id, "width_px": width, "height_px": height}
        self.height_calibration = (
            HEIGHT_CALIBRATION if HEIGHT_CALIBRATION and HEIGHT_CALIBRATION[0] == map_id else None
        )
        self.correct_colors = correct_colors
        self.stop = Event()
        self.frame = None
        self.error = None
        self.last_seen = monotonic()
        self.match_lock = Lock()
        self.last_matched = None
        self.tracking = None
        self.last_fix = 0
        self.pose = MapPose()

    def pose_snapshot(self):
        pose = self.pose.snapshot(attitude.snapshot())
        if pose["tracking_status"] != "waiting":
            pose["x_norm"] = pose["x_px"] / self.map_info["width_px"]
            pose["y_norm"] = pose["y_px"] / self.map_info["height_px"]
        pose["height_calibrated"] = self.height_calibration is not None
        return pose

    def locate(self, frame, timestamp):
        # Keep the full field of view and bound the per-frame workload for live mode.
        height, width = frame.shape[:2]
        scale = min(1, 640 / max(height, width))
        view = cv.resize(frame, None, fx=scale, fy=scale, interpolation=cv.INTER_AREA)
        gray = cv.cvtColor(view, cv.COLOR_BGR2GRAY)
        if np.percentile(gray[::4, ::4], 99) < 12:
            raise ValueError("Camera view is too dark for map matching.")
        if self.tracking is not None and timestamp - self.last_fix < 5:
            previous_time, previous, points, transform, tentative = self.tracking
            if timestamp - previous_time < 1.5:
                result = self.track(gray, previous, points, transform, tentative, timestamp)
                if result is not None:
                    overlay, estimate = result
                    return overlay, self.add_height(estimate, gray.shape, frame.shape)
        self.tracking = None
        overlay, estimate = self.matcher.locate(view, allow_tentative=True, include_transform=True)
        transform = estimate["transform"]
        points = cv.goodFeaturesToTrack(gray, 180, 0.015, 9)
        if points is not None and len(points) >= 12:
            self.tracking = (timestamp, gray, points, transform, estimate["tentative"])
            self.last_fix = timestamp
        return overlay, self.add_height({**estimate, "mode": "map"}, gray.shape, frame.shape)

    def add_height(self, estimate, image_shape, capture_shape):
        transform = estimate.pop("transform")
        estimate["z_m"] = None
        if self.height_calibration and not estimate["tentative"] and capture_shape[:2] == CAPTURE_RESOLUTION[::-1]:
            _, meters_per_pixel, intrinsics = self.height_calibration
            estimate["z_m"] = estimate_height(transform, meters_per_pixel, intrinsics, image_shape, capture_shape)
        return estimate

    def track(self, gray, previous, points, transform, tentative, timestamp):
        moved, found, _ = cv.calcOpticalFlowPyrLK(previous, gray, points, None)
        if moved is None or found is None:
            return
        back, returned, _ = cv.calcOpticalFlowPyrLK(gray, previous, moved, None)
        if back is None or returned is None:
            return
        good = (found.ravel() == 1) & (returned.ravel() == 1) & (
            np.linalg.norm((back - points).reshape(-1, 2), axis=1) < 1.2)
        if good.sum() < 12:
            return
        motion, mask = cv.findHomography(moved[good], points[good], cv.RANSAC, 2.0)
        if motion is None or mask is None or mask.sum() < max(12, good.sum() * 0.60):
            return
        tracked = moved[good][mask.ravel().astype(bool)]
        if cv.contourArea(cv.convexHull(tracked)) < 0.05 * gray.size:
            return
        mapped = transform @ motion
        try:
            overlay, estimate = draw_projection(self.matcher.reference, gray.shape, mapped,
                                                len(tracked), len(points), tentative)
        except ValueError:
            return
        self.tracking = (timestamp, gray, tracked, mapped, tentative)
        return overlay, {**estimate, "mode": "tracking"}


    def run(self):
        try:
            camera = Camera(resolution=CAPTURE_RESOLUTION)
            try:
                camera.start()
                ready_at = monotonic() + 2
                if not self.stop.is_set():
                    Thread(target=self.prepare_map, daemon=True).start()
                while not self.stop.is_set() and monotonic() - self.last_seen < 60:
                    frame = camera.capture()
                    if frame is not None and monotonic() >= ready_at:
                        captured_at = monotonic()
                        imu = attitude.snapshot()
                        self.frame = (captured_at, encode_camera_frame(frame, self.correct_colors), imu)
                    self.stop.wait(0.18)
            finally:
                camera.stop()
        except Exception as error:
            self.error = str(error)
        finally:
            self.stop.set()
            camera_lock.release()

    def prepare_map(self):
        with self.match_lock:
            if not self.stop.is_set():
                self.matcher.prepare()


live_session = None


def start_live(payload: dict, correct_colors: bool = True):
    global live_session
    try:
        map_bytes = base64.b64decode(payload["map"], validate=True)
        reference = decode_image(map_bytes)
    except (ValueError, KeyError, TypeError, cv.error) as error:
        return JSONResponse({"error": str(error)}, 422)
    if not camera_lock.acquire(blocking=False):
        return JSONResponse({"error": "La camera ? occupata."}, 409)
    live_session = LiveSession(reference, correct_colors, sha256(map_bytes).hexdigest())
    Thread(target=live_session.run, daemon=True).start()
    return {"started": True}


def end_live():
    if live_session is not None:
        live_session.stop.set()
    return {"stopped": True}


def active_live():
    session = live_session
    if session is not None and not session.stop.is_set():
        session.last_seen = monotonic()
        return session


def live_preview():
    session = active_live()
    if session is None:
        error = live_session.error if live_session is not None else None
        return JSONResponse({"error": error or "Sessione live terminata."}, 409)
    snapshot = session.frame
    return {
        "frame": jpeg_url(snapshot[1]) if snapshot else None,
        "pose": session.pose_snapshot(),
    }


def get_map_pose():
    session = live_session
    active = session is not None and not session.stop.is_set()
    pose = session.pose_snapshot() if active else {"tracking_status": "inactive"}
    source = pose.get("heading_source")
    height_source = pose.get("height_source")
    uses_imu = source == "camera_imu" or height_source in {"imu", "last_imu"}
    return JSONResponse({
        "map": session.map_info if active else None,
        "tracking_status": pose["tracking_status"],
        "x_px": pose.get("x_px"),
        "y_px": pose.get("y_px"),
        "x_norm": pose.get("x_norm"),
        "y_norm": pose.get("y_norm"),
        "z_m": pose.get("z_m"),
        "height_source": height_source,
        "height_age_ms": pose.get("height_age_ms"),
        "height_estimate_age_ms": pose.get("height_estimate_age_ms"),
        "height_calibrated": pose.get("height_calibrated"),
        "frame_age_ms": pose.get("frame_age_ms"),
        "heading_deg": pose.get("heading_deg"),
        "heading_source": source,
        "imu_estimated": None if source is None and height_source is None else int(uses_imu),
    }, headers={"Cache-Control": "no-store"})


def live_match():
    session = active_live()
    if session is None:
        return JSONResponse({"error": "Sessione live terminata."}, 409)
    if not session.match_lock.acquire(blocking=False):
        return {"waiting": True}
    try:
        snapshot = session.frame
        if snapshot is None or snapshot[0] == session.last_matched:
            return {"waiting": True}
        session.last_matched = snapshot[0]
        started = monotonic()
        try:
            overlay, estimate = session.locate(decode_image(snapshot[1]), snapshot[0])
            session.pose.update(estimate, snapshot[0], snapshot[2])
            ok, jpeg = cv.imencode(".jpg", overlay)
            if not ok:
                raise ValueError("Impossibile creare la mappa del risultato.")
            result = {**estimate, "overlay": jpeg_url(jpeg.tobytes())}
        except (ValueError, cv.error, np.linalg.LinAlgError) as error:
            result = {"diagnostic": str(error)}
        return {**result, "seconds": round(monotonic() - started, 2)}
    finally:
        session.match_lock.release()


ui.expose_api("POST", "/live/start", start_live)
ui.expose_api("POST", "/live/stop", end_live)
ui.expose_api("POST", "/live/preview", live_preview)
ui.expose_api("POST", "/live/match", live_match)
ui.expose_api("GET", "/api/pose", get_map_pose)
ui.expose_api("POST", "/capture", capture)
ui.expose_api("POST", "/map", downloaded_map)
ui.expose_api("POST", "/match", match)
ui.expose_api("POST", "/landing", capture_landing)


def read_attitude():
    global shown_lander_status
    sleep(0.04)
    try:
        attitude.update(Bridge.call("rover_read_imu"))
    except Exception:
        attitude.update("")
        sleep(0.5)

    reading = attitude.snapshot()
    session = live_session
    if session is not None and not session.stop.is_set():
        session.pose.update_imu(reading)
    if reading["status"] == "ready":
        display_status = reading["lander_status"].replace("_", " ").upper()
    elif reading["status"] in {"unavailable", "stale"}:
        display_status = "IMU UNAVAILABLE"
    else:
        display_status = "CALIBRATING"
    if display_status != shown_lander_status:
        try:
            Bridge.call("rover_lander_status", display_status)
        except Exception:
            return
        shown_lander_status = display_status


def arm_landing():
    if not attitude.arm_landing():
        return JSONResponse({"error": "Wait for the IMU to calibrate before arming."}, 409)
    return {"lander_status": "in_air"}


def reset_attitude():
    attitude.reset()
    return {"reset": True}


ui.expose_api("POST", "/imu", attitude.snapshot)
ui.expose_api("POST", "/imu/calibrate", reset_attitude)
ui.expose_api("POST", "/lander/arm", arm_landing)
App.run(user_loop=read_attitude)
