import math
from collections import deque
from threading import Lock
from time import monotonic

import cv2 as cv
import numpy as np

STANDARD_G = 9.80665
LOCAL_GRAVITY = 9.81
SETTLED_SECONDS = 2.0
# Sensor axes to rover axes, right-handed. Set this for the actual mounting.
SENSOR_TO_ROVER = np.eye(3)


class Attitude:
    def __init__(self):
        self.lock = Lock()
        self.epoch = 0
        self.lander_status = "ready"
        self.reset()

    def reset(self):
        with self.lock:
            self.epoch += 1
            self.samples = deque(maxlen=40)
            self.rotation = None
            self.timestamp = None
            self.sampled_at = None
            self.result = {"status": "waiting"}
            self.observed_motion = False
            self.settled_seconds = 0.0

    def arm_landing(self):
        with self.lock:
            if self.result["status"] != "ready" or monotonic() - self.sampled_at > 0.25:
                return False
            self.lander_status = "in_air"
            self.observed_motion = False
            self.settled_seconds = 0.0
            return True

    def _update_landing(self, dt, magnitude, rotation_rate):
        if self.lander_status != "in_air":
            return
        gravity_error = abs(magnitude - LOCAL_GRAVITY) / LOCAL_GRAVITY
        if gravity_error >= 0.18 or rotation_rate >= 15:
            self.observed_motion = True
        if self.observed_motion and 0 < dt < 0.25 and gravity_error <= 0.08 and rotation_rate <= 5:
            self.settled_seconds += dt
            if self.settled_seconds >= SETTLED_SECONDS:
                self.lander_status = "landed"
        else:
            self.settled_seconds = 0.0

    def update(self, packet):
        with self.lock:
            if not packet:
                if self.timestamp is not None:
                    self.epoch += 1
                self.result = {"status": "unavailable"}
                self.timestamp = None
                self.samples.clear()
                self.settled_seconds = 0.0
                return
            tick, *data = map(float, packet.split(","))
            acceleration = SENSOR_TO_ROVER @ np.array(data[:3]) * STANDARD_G
            rates = SENSOR_TO_ROVER @ np.array(data[3:])
            magnitude = np.linalg.norm(acceleration)
            if self.rotation is None:
                if magnitude < 0.65 * LOCAL_GRAVITY or magnitude > 1.35 * LOCAL_GRAVITY or np.linalg.norm(rates) > 5:
                    self.samples.clear()
                else:
                    self.samples.append((acceleration, rates))
                self.result = {"status": "calibrating", "samples": len(self.samples)}
                if len(self.samples) < self.samples.maxlen:
                    return
                acc, gyro = np.asarray(self.samples).transpose(1, 0, 2)
                if np.max(acc.std(axis=0)) > 0.18 or np.max(gyro.std(axis=0)) > 0.65:
                    self.samples.clear()
                    return
                mean_acc = acc.mean(axis=0)
                self.scale = LOCAL_GRAVITY / np.linalg.norm(mean_acc)
                self.bias = gyro.mean(axis=0)
                x, y, z = mean_acc
                roll = math.atan2(y, z)
                pitch = math.atan2(-x, math.hypot(y, z))
                cr, sr, cp, sp = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch)
                self.rotation = np.array([[cp, sp * sr, sp * cr], [0, cr, -sr], [-sp, cp * sr, cp * cr]])
            dt = ((tick - self.timestamp) % (2 ** 32)) / 1000 if self.timestamp is not None else 0
            self.timestamp = tick
            self.sampled_at = monotonic()
            if dt >= 0.25:
                self.epoch += 1
            if 0 < dt < 0.25:
                omega = np.deg2rad(rates - self.bias)
                if abs(magnitude * self.scale - LOCAL_GRAVITY) < 0.12 * LOCAL_GRAVITY:
                    omega += 1.5 * np.cross(acceleration / magnitude, self.rotation[2])
                self.rotation = self.rotation @ cv.Rodrigues(omega * dt)[0]
            self._update_landing(dt, magnitude * self.scale, np.linalg.norm(rates - self.bias))
            r = self.rotation
            self.result = {
                "status": "ready",
                "epoch": self.epoch,
                "sampled_at": self.sampled_at,
                "roll": round(math.degrees(math.atan2(r[2, 1], r[2, 2])), 1),
                "pitch": round(math.degrees(math.atan2(-r[2, 0], math.hypot(r[2, 1], r[2, 2]))), 1),
                "yaw": round(math.degrees(math.atan2(r[1, 0], r[0, 0])), 1),
                "acceleration": round(float(magnitude * self.scale), 2),
            }

    def snapshot(self):
        with self.lock:
            if self.result["status"] == "ready" and monotonic() - self.sampled_at > 0.25:
                return {"status": "stale", "lander_status": self.lander_status}
            return {**self.result, "lander_status": self.lander_status}
