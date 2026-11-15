"""Vignette + fine film grain for the hero render.  python post_grade.py in.png out.png"""
import sys, numpy as np
from PIL import Image
im = np.asarray(Image.open(sys.argv[1]).convert("RGB")).astype(np.float32) / 255
h, w, _ = im.shape
yy, xx = np.mgrid[0:h, 0:w]
r = np.sqrt(((xx - w / 2) / (w / 2)) ** 2 + ((yy - h / 2) / (h / 2)) ** 2)
im *= (1 - 0.32 * np.clip(r - 0.45, 0, 1) ** 1.6)[..., None]
rng = np.random.default_rng(3)
im += rng.normal(0, 0.012, (h, w, 1))
Image.fromarray((np.clip(im, 0, 1) * 255).astype(np.uint8)).save(sys.argv[2], optimize=True)
