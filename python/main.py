import base64
import binascii
import json
import math
import os
from pathlib import Path
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
from camera_color import balance_colors, encode_camera_frame
from imu import Attitude
from landing_zone import LandingZone
from map_height import estimate_height
from map_pose import MapPose
from matching import MapMatcher, draw_projection

# Optional. Edit in App Lab, then Stop and Run again.
MAP_URL = ""
# Map SHA-256 -> (meters per pixel, (fx, fy, cx, cy) at CAPTURE_RESOLUTION).
# Bench profiles use a 30 mm wheel and an approximate 0.75 m reference height.
HEIGHT_CALIBRATIONS = {
    # download (7).jpg, wheel diameter about 97 px
    "678543fb811057ac309efc4dd657305943b579091c46f165649e5126ccbe424b": (
        0.030 / 97, (2425, 2425, 639.5, 479.5),
    ),
    # download (8).jpg, wheel diameter about 55 px
    "b6eb6d036ddbc43d716e6e8ed1d8ae750328038fbcd5303d74f4747a927a128b": (
        0.030 / 55, (1375, 1375, 639.5, 479.5),
    ),
}
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
        self.height_calibration = HEIGHT_CALIBRATIONS.get(map_id)
        self.correct_colors = correct_colors
        self.stop = Event()
        self.frame = None
        self.error = None
        self.last_seen = monotonic()
        self.match_lock = Lock()
        self.last_matched = None
        self.tracking = None
        self.last_fix = 0
        self.last_map_attempt = 0
        self.pose = MapPose()
        self.landing_zone = None
        self.zone_reading = None

    def landing_report(self):
        if self.landing_zone is None:
            return None
        reading = self.zone_reading
        # A match takes ~0.7 s on the Uno Q plus HTTP latency, so 1.25 s made the zone flicker.
        if reading is None or monotonic() - reading[0] > ZONE_FRESH_S:
            result = {"status": "unknown", "target": None, "alternative": None,
                      "reason": "Waiting for a fresh camera match."}
        else:
            result = reading[1]
        return {**result, "hazard": self.landing_zone.hazard,
                "area": self.landing_zone.area, "footprint": self.landing_zone.footprint}

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
        tracked = None
        if self.tracking is not None and timestamp - self.last_fix < 12:
            previous_time, previous, points, transform, tentative = self.tracking
            if timestamp - previous_time < 1.5:
                tracked = self.track(gray, previous, points, transform, tentative, timestamp)
                if tracked is not None and not tentative and (
                    timestamp - self.last_fix < 5 or timestamp - self.last_map_attempt < 2
                ):
                    overlay, estimate = tracked
                    return overlay, self.finish_match(estimate, view, frame.shape, timestamp)
        self.last_map_attempt = timestamp
        try:
            overlay, estimate = self.matcher.locate(view, allow_tentative=True, include_transform=True)
        except ValueError:
            if tracked is None or tracked[1]["tentative"]:
                raise
            overlay, estimate = tracked
            return overlay, self.finish_match(estimate, view, frame.shape, timestamp)
        if estimate["tentative"] and tracked is not None and not tracked[1]["tentative"]:
            overlay, estimate = tracked
            return overlay, self.finish_match(estimate, view, frame.shape, timestamp)
        self.tracking = None
        transform = estimate["transform"]
        if not estimate["tentative"]:
            self.last_fix = timestamp
        points = cv.goodFeaturesToTrack(gray, 180, 0.01, 7)
        if points is not None and len(points) >= 12:
            self.tracking = (timestamp, gray, points, transform, estimate["tentative"])
            self.last_fix = timestamp
        return overlay, self.finish_match({**estimate, "mode": "map"}, view, frame.shape, timestamp)

    def finish_match(self, estimate, view, capture_shape, timestamp):
        if self.landing_zone is not None and not estimate["tentative"] and timestamp - self.last_fix < 5:
            result = self.landing_zone.inspect(
                view, estimate["transform"],
                estimate["x"] / self.map_info["width_px"],
                estimate["y"] / self.map_info["height_px"],
            )
            self.zone_reading = (timestamp, result)
        else:
            self.zone_reading = None
        return self.add_height(estimate, view.shape, capture_shape)

    def add_height(self, estimate, image_shape, capture_shape):
        transform = estimate.pop("transform")
        estimate["z_m"] = None
        if self.height_calibration and not estimate["tentative"] and capture_shape[:2] == CAPTURE_RESOLUTION[::-1]:
            meters_per_pixel, intrinsics = self.height_calibration
            estimate["z_m"] = estimate_height(transform, meters_per_pixel, intrinsics, image_shape, capture_shape)
        return estimate

    def track(self, gray, previous, points, transform, tentative, timestamp):
        moved, found, _ = cv.calcOpticalFlowPyrLK(previous, gray, points, None, winSize=(31, 31))
        if moved is None or found is None:
            return
        back, returned, _ = cv.calcOpticalFlowPyrLK(gray, previous, moved, None, winSize=(31, 31))
        if back is None or returned is None:
            return
        good = (found.ravel() == 1) & (returned.ravel() == 1) & (
            np.linalg.norm((back - points).reshape(-1, 2), axis=1) < 1.6)
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
        if len(tracked) < 100:
            mask = np.zeros_like(gray)
            hull = np.rint(cv.convexHull(tracked)).astype(np.int32)
            cv.fillConvexPoly(mask, hull, 255)
            for x, y in np.rint(tracked.reshape(-1, 2)).astype(int):
                cv.circle(mask, (x, y), 7, 0, -1)
            extra = cv.goodFeaturesToTrack(gray, 180 - len(tracked), 0.01, 7, mask=mask)
            if extra is not None:
                tracked = np.concatenate((tracked, extra))
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


# ---------------------------------------------------------------- saved setup (persists across reloads/restarts)
# The reference map and the landing site live on the board until the user replaces them.
DATA_DIR = Path(__file__).resolve().parent.parent / "data"
ZONE_FRESH_S = 4.0                         # how long a landing-zone reading stays valid
MAP_FILE = DATA_DIR / "map.img"            # original bytes (keeps the map hash used by HEIGHT_CALIBRATIONS)
ZONE_FILE = DATA_DIR / "landing_zone.json"
CROPPED_FLAG = DATA_DIR / "map.cropped"       # present once the saved map was trimmed to the board
REGION_FILE = DATA_DIR / "region.json"       # Plan Banana region (Z1/Z2) laid over the board


def _write_atomic(path, data):
    DATA_DIR.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_bytes(data)
    os.replace(tmp, path)


def saved_map_bytes():
    try:
        return MAP_FILE.read_bytes()
    except OSError:
        return None


def saved_zone():
    try:
        return json.loads(ZONE_FILE.read_text())
    except (OSError, ValueError):
        return None


def saved_region():
    try:
        return json.loads(REGION_FILE.read_text())
    except (OSError, ValueError):
        return None


def detect_board(image):
    """Normalised [left, top, right, bottom] of the bright foam board on a darker background."""
    gray = cv.GaussianBlur(cv.cvtColor(image, cv.COLOR_BGR2GRAY), (9, 9), 0)
    _, mask = cv.threshold(gray, 0, 255, cv.THRESH_BINARY + cv.THRESH_OTSU)
    mask = cv.morphologyEx(mask, cv.MORPH_OPEN, cv.getStructuringElement(cv.MORPH_ELLIPSE, (15, 15)))
    contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    height, width = gray.shape
    if contours:
        x, y, w, h = cv.boundingRect(max(contours, key=cv.contourArea))
        if w * h > 0.15 * width * height:
            return [x / width, y / height, (x + w) / width, (y + h) / height]
    return [0.0, 0.0, 1.0, 1.0]


def region_to_map(region, board, shape, rotation_deg=0.0):
    """Place zones on the board. The region square IS the board (axis-aligned); rotation_deg turns only the
    north direction (clockwise from up). Zones are clamped onto the board; a zone the user dragged keeps
    its dragged position ("override")."""
    height, width = shape[:2]
    size_m = float(region.get("region_size_km", 15.0)) * 1000.0
    left, top, right, bottom = board
    bw, bh = (right - left) * width, (bottom - top) * height
    cx, cy = left * width + bw / 2, top * height + bh / 2
    theta = math.radians(rotation_deg)
    north = (math.sin(theta), -math.cos(theta))                  # image x right, y down
    east = (math.cos(theta), math.sin(theta))
    inset = 0.04
    zones = []
    for zone in region.get("zones", []):
        e, n = float(zone["east_m"]), float(zone["north_m"])
        u = 0.5 + (e * east[0] + n * north[0]) / size_m          # 0..1 across the board
        v = 0.5 + (e * east[1] + n * north[1]) / size_m
        u, v = min(1 - inset, max(inset, u)), min(1 - inset, max(inset, v))
        x_norm, y_norm = left + u * (right - left), top + v * (bottom - top)
        if zone.get("override"):
            x_norm, y_norm = map(float, zone["override"])
        zones.append({**zone, "x_norm": x_norm, "y_norm": y_norm,
                      "r_norm": min(0.12, float(zone.get("margin_m", 500.0)) / size_m * bw / width)})
    square = {"x_norm": cx / width, "y_norm": cy / height, "w_norm": bw / width, "h_norm": bh / height, "angle": 0.0}
    return zones, square


def save_region(payload: dict):
    """Store a Plan Banana region summary and make its rank-1 zone the planned landing site."""
    try:
        payload = payload if isinstance(payload, dict) else {}
        if "region" not in payload and ("rotation_deg" in payload or "move" in payload or "replace" in payload):
            region = saved_region()                             # re-place the saved region (rotate / drag / new map)
            if region is None:
                raise ValueError("Import a region first.")
            if "rotation_deg" in payload:
                for z in region["zones"]:
                    z.pop("override", None)                     # a new north direction re-places every zone
            move = payload.get("move")
            if move:
                x, y = float(move["x_norm"]), float(move["y_norm"])
                if not (0 <= x <= 1 and 0 <= y <= 1):
                    raise ValueError("Drop the zone on the map.")
                for z in region["zones"]:
                    if z.get("id") == move.get("id"):
                        z["override"] = [round(x, 5), round(y, 5)]
        else:
            region = payload.get("region")
        if region is None:
            REGION_FILE.unlink(missing_ok=True)
            return {"saved": True, "region": None}
        if isinstance(region, str):
            region = json.loads(region)
        zones = region["zones"]
        if not isinstance(zones, list) or not zones:
            raise ValueError("The region has no landing zones.")
        for zone in zones:
            float(zone["east_m"]), float(zone["north_m"])
        data = saved_map_bytes()
        if data is None:
            raise ValueError("Save a map first.")
        image = decode_image(data)
        board = detect_board(image)
        rotation = float(payload.get("rotation_deg", region.get("rotation_deg", 0.0))) % 360
        placed, square = region_to_map(region, board, image.shape, rotation)
        primary = min(placed, key=lambda z: z.get("rank", 99))
        footprint = float(payload.get("footprint", (saved_zone() or {}).get("footprint", 0.10)))
        half = min(0.2, max(primary["r_norm"], footprint * 0.9))
        area = [max(0.0, primary["x_norm"] - half), max(0.0, primary["y_norm"] - half),
                min(1.0, primary["x_norm"] + half), min(1.0, primary["y_norm"] + half)]
        zone_settings = {"area": [round(v, 5) for v in area], "footprint": round(footprint, 5)}
        LandingZone(image, zone_settings)                     # validate like a live start
        stored = {"name": region.get("name"), "id": region.get("id"), "region_size_km": region.get("region_size_km", 15.0),
                  "optimal_touchdown_utc": region.get("optimal_touchdown_utc"), "board": [round(v, 5) for v in board],
                  "rotation_deg": round(rotation, 2), "square": square,
                  "zones": [{k: z.get(k) for k in ("id", "rank", "east_m", "north_m", "margin_m", "mean_success",
                                                   "target_success", "from_primary", "x_norm", "y_norm", "r_norm",
                                                   "override")}
                            for z in placed]}
        _write_atomic(REGION_FILE, json.dumps(stored).encode())
        _write_atomic(ZONE_FILE, json.dumps(zone_settings).encode())
        return {"saved": True, "region": stored, "landing_zone": zone_settings}
    except (ValueError, KeyError, TypeError, OSError, cv.error) as error:
        return JSONResponse({"error": f"Could not use this region: {error}"}, 422)


def get_setup():
    data = saved_map_bytes()
    mime = "image/png" if data and data[:4] == b"\x89PNG" else "image/jpeg"
    return JSONResponse({
        "map": f"data:{mime};base64," + base64.b64encode(data).decode("ascii") if data else None,
        "map_saved_at": MAP_FILE.stat().st_mtime if data else None,
        "landing_zone": saved_zone(),
        "region": saved_region(),
        "cropped": CROPPED_FLAG.exists(),
    }, headers={"Cache-Control": "no-store"})


def _store_map(data):
    decode_image(data)                       # validates JPG/PNG and size
    _write_atomic(MAP_FILE, data)
    for stale in (ZONE_FILE, REGION_FILE, CROPPED_FLAG):   # the old landing site / region placement belong to the old map
        try:
            stale.unlink()
        except OSError:
            pass
    return {"saved": True, "map": jpeg_url(data) if data[:4] != b"\x89PNG" else
            "data:image/png;base64," + base64.b64encode(data).decode("ascii")}


def save_map(payload: dict):
    try:
        return _store_map(base64.b64decode(payload["map"], validate=True))
    except (KeyError, TypeError, ValueError, binascii.Error, OSError) as error:
        return JSONResponse({"error": str(error)}, 422)


class MapPreview:
    """Lightweight camera stream for lining up the map photo (no map matching)."""

    def __init__(self):
        self.stop = Event()
        self.raw = None
        self.error = None
        self.last_seen = monotonic()

    def run(self):
        try:
            camera = Camera(resolution=CAPTURE_RESOLUTION)
            try:
                camera.start()
                ready_at = monotonic() + 1.0
                while not self.stop.is_set() and monotonic() - self.last_seen < 20:
                    frame = camera.capture()
                    if frame is not None and monotonic() >= ready_at:
                        self.raw = frame
                    self.stop.wait(0.12)
            finally:
                camera.stop()
        except Exception as error:
            self.error = str(error)
        finally:
            self.stop.set()
            camera_lock.release()


map_preview = None


def _preview_active():
    return map_preview is not None and not map_preview.stop.is_set()


def _stop_preview_and_wait(timeout=4.0):
    """Stop the map preview (if any) and take the camera lock. Returns True when the lock is held."""
    if _preview_active():
        map_preview.stop.set()
    return camera_lock.acquire(timeout=timeout)


def preview_start():
    global map_preview
    if live_session is not None and not live_session.stop.is_set():
        return JSONResponse({"error": "Stop the mission before opening the live view."}, 409)
    if _preview_active():
        map_preview.last_seen = monotonic()
        return {"started": True}
    if not camera_lock.acquire(timeout=3):
        return JSONResponse({"error": "The camera is busy. Try again in a moment."}, 409)
    map_preview = MapPreview()
    Thread(target=map_preview.run, daemon=True).start()
    return {"started": True}


def preview_frame(correct_colors: bool = True):
    session = map_preview
    if session is None or session.stop.is_set():
        return JSONResponse({"error": (session.error if session else None) or "Live view stopped."}, 409)
    session.last_seen = monotonic()
    raw = session.raw
    if raw is None:
        return {"frame": None}
    image = cv.cvtColor(raw, cv.COLOR_BGRA2BGR) if raw.ndim == 3 and raw.shape[2] == 4 else raw
    small = cv.resize(image, None, fx=0.5, fy=0.5, interpolation=cv.INTER_AREA)
    if correct_colors and small.ndim == 3:
        small = balance_colors(small)
    ok, jpeg = cv.imencode(".jpg", small, [cv.IMWRITE_JPEG_QUALITY, 78])
    return {"frame": jpeg_url(jpeg.tobytes())} if ok else {"frame": None}


def preview_stop():
    if map_preview is not None:
        map_preview.stop.set()
    return {"stopped": True}


def crop_map_to_board():
    """Trim the saved map to the detected board so the surroundings play no part; keep the region."""
    try:
        data = saved_map_bytes()
        if data is None:
            raise ValueError("Save a map first.")
        image = decode_image(data)
        height, width = image.shape[:2]
        left, top, right, bottom = detect_board(image)
        if CROPPED_FLAG.exists() or (right - left > 0.97 and bottom - top > 0.97):
            raise ValueError("The map already shows only the board.")
        pad = 0.01
        x0, y0 = int(max(0, left - pad) * width), int(max(0, top - pad) * height)
        x1, y1 = int(min(1, right + pad) * width), int(min(1, bottom + pad) * height)
        ok, jpeg = cv.imencode(".jpg", image[y0:y1, x0:x1], [cv.IMWRITE_JPEG_QUALITY, 95])
        if not ok:
            raise ValueError("Could not encode the cropped map.")
        region = saved_region()
        footprint = (saved_zone() or {}).get("footprint", 0.10)
        result = _store_map(jpeg.tobytes())                      # clears zone + region files
        _write_atomic(CROPPED_FLAG, b"1")
        if region is not None:
            for z in region["zones"]:
                if z.get("override"):                            # move dragged positions into the cropped frame
                    ox, oy = z["override"]
                    z["override"] = [round((ox * width - x0) / (x1 - x0), 5), round((oy * height - y0) / (y1 - y0), 5)]
            placed = save_region({"region": region, "rotation_deg": region.get("rotation_deg", 0.0), "footprint": footprint})
            if isinstance(placed, dict):
                result.update(region=placed["region"], landing_zone=placed["landing_zone"])
        return result
    except (ValueError, OSError, cv.error) as error:
        return JSONResponse({"error": str(error)}, 422)


def save_map_from_camera(correct_colors: bool = True):
    if live_session is not None and not live_session.stop.is_set():
        return JSONResponse({"error": "Stop the mission before taking a new map photo."}, 409)
    try:
        if _preview_active() and map_preview.raw is not None:
            data = encode_camera_frame(map_preview.raw, correct_colors)   # exactly the frame on screen
            map_preview.stop.set()
            return _store_map(data)
        if _preview_active():
            map_preview.stop.set()
        return _store_map(capture_frame(correct_colors))
    except Exception as error:
        return JSONResponse({"error": str(error)}, 503)


def save_zone(payload: dict):
    zone = payload.get("landing_zone") if isinstance(payload, dict) else None
    try:
        if zone is None:
            ZONE_FILE.unlink(missing_ok=True)
            return {"saved": True, "landing_zone": None}
        data = saved_map_bytes()
        if data is None:
            raise ValueError("Save a map first.")
        LandingZone(decode_image(data), zone)          # same validation as a live start
        clean = {"area": [round(float(v), 5) for v in zone["area"]], "footprint": round(float(zone["footprint"]), 5)}
        _write_atomic(ZONE_FILE, json.dumps(clean).encode())
        return {"saved": True, "landing_zone": clean}
    except (ValueError, KeyError, TypeError, OSError, cv.error) as error:
        return JSONResponse({"error": str(error)}, 422)


live_session = None


def start_live(payload: dict, correct_colors: bool = True):
    global live_session
    try:
        payload = payload if isinstance(payload, dict) else {}
        if payload.get("map"):
            map_bytes = base64.b64decode(payload["map"], validate=True)
        else:
            map_bytes = saved_map_bytes()
            if map_bytes is None:
                raise ValueError("No saved map. Take or upload a map photo first.")
        reference = decode_image(map_bytes)
        zone_settings = payload["landing_zone"] if "landing_zone" in payload else saved_zone()
        landing_zone = LandingZone(reference, zone_settings) if zone_settings is not None else None
    except (ValueError, KeyError, TypeError, cv.error) as error:
        return JSONResponse({"error": str(error)}, 422)
    if not _stop_preview_and_wait():
        return JSONResponse({"error": "La camera ? occupata."}, 409)
    live_session = LiveSession(reference, correct_colors, sha256(map_bytes).hexdigest())
    live_session.landing_zone = landing_zone
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
        "landing_zone": session.landing_report(),
    }


def seed_live_height(payload: dict):
    session = active_live()
    if session is None:
        return JSONResponse({"error": "Start live tracking before setting height."}, 409)
    try:
        height = float(payload["height_m"])
        if not math.isfinite(height) or height <= 0:
            raise ValueError()
    except (TypeError, ValueError, KeyError, OverflowError):
        return JSONResponse({"error": "Enter a positive height in meters."}, 422)
    if not session.pose.seed_height(height, attitude.snapshot()):
        return JSONResponse({"error": "Wait for the IMU to calibrate before setting height."}, 409)
    return {"z_m": height, "height_source": "initial"}


def get_map_pose():
    session = live_session
    active = session is not None and not session.stop.is_set()
    pose = session.pose_snapshot() if active else {"tracking_status": "inactive"}
    source = pose.get("heading_source")
    height_source = pose.get("height_source")
    uses_imu = source == "camera_imu" or height_source in {"imu", "last_imu"}
    return JSONResponse({
        "map": session.map_info if active else None,
        "landing_zone": session.landing_report() if active else None,
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
            session.zone_reading = None
            result = {"diagnostic": str(error)}
        return {**result, "seconds": round(monotonic() - started, 2)}
    finally:
        session.match_lock.release()


ui.expose_api("POST", "/live/start", start_live)
ui.expose_api("POST", "/live/stop", end_live)
ui.expose_api("POST", "/live/preview", live_preview)
ui.expose_api("POST", "/live/match", live_match)
ui.expose_api("POST", "/live/height", seed_live_height)
ui.expose_api("GET", "/api/pose", get_map_pose)
ui.expose_api("POST", "/capture", capture)
ui.expose_api("POST", "/map", downloaded_map)
ui.expose_api("POST", "/match", match)
ui.expose_api("POST", "/landing", capture_landing)
ui.expose_api("GET", "/setup", get_setup)
ui.expose_api("POST", "/setup/map", save_map)
ui.expose_api("POST", "/setup/map/camera", save_map_from_camera)
ui.expose_api("POST", "/setup/zone", save_zone)
ui.expose_api("POST", "/setup/region", save_region)
ui.expose_api("POST", "/setup/map/crop", crop_map_to_board)
ui.expose_api("POST", "/setup/preview/start", preview_start)
ui.expose_api("POST", "/setup/preview", preview_frame)
ui.expose_api("POST", "/setup/preview/stop", preview_stop)


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
