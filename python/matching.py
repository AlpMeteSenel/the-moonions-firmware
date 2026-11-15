import math

import cv2 as cv
import numpy as np


def extract_features(image, enhance=False, max_features=8000):
    gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
    if enhance:
        gray = cv.createCLAHE(clipLimit=2.5, tileGridSize=(8, 8)).apply(gray)
    detector = cv.SIFT_create(nfeatures=max_features, contrastThreshold=0.025 if enhance else 0.04)
    return detector.detectAndCompute(gray, None)


class MapMatcher:
    def __init__(self, reference, live=False):
        self.reference = reference
        self.live = live
        self.features = {}

    def prepare(self, enhance=False):
        if enhance not in self.features:
            keys, descriptors = extract_features(self.reference, enhance)
            index = cv.FlannBasedMatcher(dict(algorithm=1, trees=4),
                                         dict(checks=32 if self.live else 64))
            if descriptors is not None and len(descriptors) >= 2:
                index.add([descriptors])
                index.train()
            self.features[enhance] = keys, index

    def locate(self, frame, allow_tentative=False, include_transform=False):
        errors = []
        tentative = None
        for enhance in (False, True):
            self.prepare(enhance)
            try:
                result = localize(self.reference, frame, self.features[enhance], enhance, allow_tentative, self.live)
                if not include_transform:
                    result[1].pop("transform")
                if not result[1]["tentative"]:
                    return result
                if tentative is None or result[1]["inliers"] > tentative[1]["inliers"]:
                    tentative = result
            except ValueError as error:
                errors.append(str(error))
        if tentative is not None:
            return tentative
        if errors[0] == errors[1]:
            raise ValueError(errors[0])
        raise ValueError(f"Standard: {errors[0]} Contrasto locale: {errors[1]}")


def localize(canvas, view, map_features, enhance, allow_tentative, live):
    frame_keys, frame_descriptors = extract_features(view, enhance, 650 if live else 8000)
    map_keys, matcher = map_features
    detail = f"Punti rilevati: fotogramma {len(frame_keys)}, mappa {len(map_keys)}."
    minimum = 4 if allow_tentative else 12
    if len(frame_keys) < minimum or len(map_keys) < 2:
        raise ValueError(f"Dettagli insufficienti. {detail}")
    pairs = matcher.knnMatch(frame_descriptors, k=2)
    matches = sorted(
        (pair[0] for pair in pairs if len(pair) == 2 and pair[0].distance < (0.80 if enhance else 0.75) * pair[1].distance),
        key=lambda match: match.distance,
    )
    unique = {}
    for match in matches:
        unique.setdefault(match.trainIdx, match)
    matches = list(unique.values())
    if len(matches) < minimum:
        raise ValueError(f"Solo {len(matches)}/{minimum} corrispondenze distinte. {detail}")
    source = np.float32([frame_keys[m.queryIdx].pt for m in matches])
    target = np.float32([map_keys[m.trainIdx].pt for m in matches])
    threshold = max(3.0, max(canvas.shape[:2]) / 600)
    transform, mask = cv.findHomography(source, target, cv.USAC_MAGSAC, threshold,
                                       maxIters=8000, confidence=0.999)
    if transform is None or mask is None:
        raise ValueError("Nessuna trasformazione coerente.")
    inliers = mask.ravel().astype(bool)
    count = int(inliers.sum())
    height, width = view.shape[:2]
    ratio = count / len(matches)
    coverage = cv.contourArea(cv.convexHull(source[inliers])) / (height * width) if count >= 4 else 0
    strict = count >= 12 and ratio >= 0.35 and coverage >= 0.04
    if not strict and count >= 16 and ratio >= 0.20 and coverage >= 0.08:
        recovered = cv.perspectiveTransform(target[inliers][None], np.linalg.inv(transform))[0]
        backward_error = np.linalg.norm(recovered - source[inliers], axis=1)
        strict = np.median(backward_error) <= max(3, max(height, width) / 375)
    tentative = not strict
    if tentative and (not allow_tentative or count < 4):
        raise ValueError(f"Match incerto ({count}/{len(matches)} punti coerenti).")

    return draw_projection(canvas, (height, width), transform, count, len(matches), tentative)


def draw_projection(canvas, shape, transform, count, matches, tentative):
    height, width = shape
    anchors = np.float32([
        [0, 0], [width - 1, 0], [width - 1, height - 1], [0, height - 1],
        [width / 2, height / 2], [width / 2, height / 4],
    ])
    denominator = anchors @ transform[2, :2] + transform[2, 2]
    if np.any(abs(denominator) < 1e-8) or np.min(denominator) * np.max(denominator) <= 0:
        raise ValueError("Proiezione non valida.")
    projected = cv.perspectiveTransform(anchors[None, :, :], transform)[0]
    outline = projected[:4]
    if (not np.isfinite(projected).all() or np.max(abs(projected)) > 4 * max(canvas.shape[:2])
            or not cv.isContourConvex(outline) or cv.contourArea(outline, oriented=True) < 100):
        raise ValueError("Il contorno stimato non è plausibile.")
    center, top = projected[4:]
    if not (0 <= center[0] < canvas.shape[1] and 0 <= center[1] < canvas.shape[0]):
        raise ValueError("Il centro stimato è fuori dalla mappa.")
    dx, dy = top - center
    color = (0, 185, 255) if tentative else (70, 255, 50)
    cv.polylines(canvas := canvas.copy(), [np.rint(outline).astype(np.int32)], True, color, 3)
    cv.arrowedLine(canvas, tuple(np.rint(center).astype(int)), tuple(np.rint(top).astype(int)),
                   color, 3)
    return canvas, {
        "x": round(float(center[0]), 1),
        "y": round(float(center[1]), 1),
        "angle": round(math.degrees(math.atan2(dx, -dy)), 1),
        "inliers": count,
        "matches": matches,
        "transform": transform,
        "tentative": tentative,
    }
