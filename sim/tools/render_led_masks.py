#!/usr/bin/env python3
"""Render before/after mask pictures from tri_led_photo_tb dumps.

Usage (from the repository root, after the bench has run with +DUMP_DIR):
    python3 sim/tools/render_led_masks.py build/image_processing_tests/dumps \\
        build/image_processing_tests/tri_led_photo.log docs/img

For each dumped scene <scene>.txt (one line per pixel: RGB565 hex, raw mask
bits RGB, filtered mask bits RGB, all straight from the RTL simulation) it
writes <out>/<scene>_masking.png: camera input | raw colour mask | mask after
the run filter with the RTL centroids (read from the bench log), plus zooms
around each LED listed in sim/data/scenes.json. Needs numpy and matplotlib.
"""
import json
import re
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

W, H = 640, 480
CLASS_RGB = np.array([[255, 40, 40], [40, 230, 60], [60, 120, 255]], np.uint8)
NAMES = ["Red", "Green", "Blue"]


def load(path):
    rgb = np.zeros((H * W, 3), np.uint8)
    raw = np.zeros((H * W, 3), bool)
    filt = np.zeros((H * W, 3), bool)
    with open(path) as f:
        for i, line in enumerate(f):
            p, m, q = line.split()
            v = int(p, 16)
            rgb[i] = (((v >> 11) & 31) * 255 // 31, ((v >> 5) & 63) * 255 // 63, (v & 31) * 255 // 31)
            raw[i] = [c == "1" for c in m]
            filt[i] = [c == "1" for c in q]
    return rgb.reshape(H, W, 3), raw.reshape(H, W, 3), filt.reshape(H, W, 3)


def paint(mask):
    img = np.zeros((H, W, 3), np.uint8)
    for c in range(3):
        img[mask[..., c]] = CLASS_RGB[c]
    return img


def rtl_results(log, scene):
    """{'Red': (x, y), ...} for the colours the RTL reported valid."""
    out = {}
    for line in open(log):
        if line.startswith(scene + ":"):
            for name, v, x, y in re.findall(r"(\w) valid=(\d) \((\d+),(\d+)\)", line):
                if v == "1":
                    out[{"R": "Red", "G": "Green", "B": "Blue"}[name]] = (int(x), int(y))
    return out


def render(dump_dir, log, out_dir, scene, leds):
    rgb, raw, filt = load(f"{dump_dir}/{scene}.txt")
    res = rtl_results(log, scene)
    rc, fc = raw.sum((0, 1)), filt.sum((0, 1))
    rows = 1 + len(leds)
    fig = plt.figure(figsize=(18, 5.6 + 2.6 * len(leds)), facecolor="white")
    gs = fig.add_gridspec(rows, 3, height_ratios=[2.2] + [1] * len(leds))
    heads = ["Camera input (RGB565)",
             f"Raw colour mask  (R {rc[0]}, G {rc[1]}, B {rc[2]} px)",
             f"After run filter + RTL centroid  (R {fc[0]}, G {fc[1]}, B {fc[2]} px)"]
    for j, (img, h) in enumerate(zip([rgb, paint(raw), paint(filt)], heads)):
        ax = fig.add_subplot(gs[0, j])
        ax.imshow(img, interpolation="nearest")
        ax.set_title(h, fontsize=12)
        ax.set_xticks([]); ax.set_yticks([])
        if j == 1:   # isolated false pixels vanish at this scale: ring them
            for c in range(3):
                ys, xs = np.nonzero(raw[..., c] & ~filt[..., c])
                ax.scatter(xs, ys, s=30, facecolors="none", edgecolors=CLASS_RGB[c] / 255, linewidths=0.8)
        if j == 2:
            for name, (cx, cy) in res.items():
                ax.plot(cx, cy, "+", color="white", ms=22, mew=2)
                ax.annotate(f"{name} ({cx},{cy})", (cx, cy), xytext=(12, -14),
                            textcoords="offset points", color="white", fontsize=10)
            if not res:
                ax.text(320, 240, "no target in any colour", color="white", ha="center", fontsize=12)
    for i, led in enumerate(leds):
        cx, cy, name = led["cx"], led["cy"], led["colour"].capitalize()
        x0, x1, y0, y1 = cx - 24, cx + 24, cy - 16, cy + 16
        for j, img in enumerate([rgb, paint(raw), paint(filt)]):
            ax = fig.add_subplot(gs[1 + i, j])
            ax.imshow(img[y0:y1, x0:x1], interpolation="nearest",
                      extent=(x0 - .5, x1 - .5, y1 - .5, y0 - .5))
            ax.tick_params(labelsize=7)
            if j == 0:
                ax.set_ylabel(f"{name} LED", fontsize=10)
            if j == 2:
                ax.plot(cx, cy, "x", color="yellow", ms=8, mew=1.4)
                if name in res:
                    ax.plot(*res[name], "+", color="white", ms=20, mew=1.8)
    fig.suptitle(f"{scene}: zoom rows mark the RTL centroid (white +) and the true LED centre (yellow x)",
                 fontsize=13)
    fig.tight_layout()
    path = f"{out_dir}/{scene}_masking.png"
    fig.savefig(path, dpi=90, bbox_inches="tight")
    print("wrote", path)


def main():
    if len(sys.argv) != 4:
        sys.exit(__doc__)
    dump_dir, log, out_dir = sys.argv[1:]
    scenes = json.load(open("sim/data/scenes.json"))
    for scene in ("photo_only", "photo_leds"):
        render(dump_dir, log, out_dir, scene, scenes[scene])


if __name__ == "__main__":
    main()
