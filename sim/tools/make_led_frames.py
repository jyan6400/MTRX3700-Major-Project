#!/usr/bin/env python3
"""Build 640x480 RGB565 test frames from a real lab photo with LED dots added.

Usage (from the repository root):
    python3 sim/tools/make_led_frames.py

Reads  sim/data/lab_photo.jpg
Writes sim/data/<scene>.hex        640x480 RGB565 words, row-major ($readmemh),
                                   what the pixel stream carries
       sim/data/<scene>.png        preview exactly as that RGB565 frame looks
       sim/data/<scene>_scene.hex  656x488 RGB888 words for Ed's ov7670_model
                                   (SCENE_FILE): the same frame, turned through
                                   180 degrees because the course register list
                                   mirrors and flips the sensor (MVFP), and set
                                   in the model's array margins, so the camera's
                                   OUTPUT is the frame above
       sim/data/scenes.json        true LED centres for every scene

LEDs are drawn the way a camera sees them: a saturated white core, a
coloured body and a soft glow blended into the background. Mild sensor
noise (fixed seed, so the frames are reproducible) is added everywhere.
Requires numpy and Pillow.
"""
import json
import numpy as np
from PIL import Image

W, H = 640, 480
DATA = "sim/data"

LED_COL = {"red": (255, 35, 25), "green": (35, 255, 90), "blue": (35, 95, 255)}

# Each placement is chosen on the hardest background for that colour.
RED_ON_PHONE = ("red", 430, 205, 9)      # pink phone: red-ish distractor
RED_LEFT     = ("red", 150, 330, 9)      # left of centre, on the window
GREEN        = ("green", 165, 300, 8)    # purple jacaranda through the glass
BLUE         = ("blue", 205, 190, 10)    # blue sky: blue distractor

SCENES = {
    "photo_only":       [],
    "photo_leds":       [RED_ON_PHONE, GREEN, BLUE],
    "photo_red_left":   [RED_LEFT, BLUE],
    "photo_green_blue": [GREEN, BLUE],
}


def load_base():
    rng = np.random.default_rng(26)
    src = Image.open(f"{DATA}/lab_photo.jpg").convert("RGB")
    sw, sh = src.size
    cw = sh * 4 // 3                        # centre-crop to 4:3
    x0 = (sw - cw) // 2
    img = np.asarray(src.crop((x0, 0, x0 + cw, sh)).resize((W, H), Image.BILINEAR),
                     dtype=np.float32)
    return np.clip(img + rng.normal(0, 3.0, img.shape), 0, 255)


def draw_led(img, name, cx, cy, rad, core=3, glow=4):
    col = np.array(LED_COL[name], np.float32)
    yy, xx = np.mgrid[0:H, 0:W]
    d = np.sqrt((xx - cx) ** 2 + (yy - cy) ** 2)
    a = np.clip(1.0 - (d - rad) / glow, 0.0, 1.0)[..., None]
    shade = np.clip(1.0 - 0.15 * np.clip(d - core, 0, None) / rad, 0.8, 1.0)[..., None]
    body = np.where((d <= core)[..., None], 255.0, col * shade)
    return img * (1 - a) + body * a


def to565(img):
    q = np.clip(np.rint(img), 0, 255).astype(np.uint16)
    return ((q[..., 0] >> 3) << 11) | ((q[..., 1] >> 2) << 5) | (q[..., 2] >> 3)


def save(name, img):
    w = to565(img)
    with open(f"{DATA}/{name}.hex", "w") as f:
        f.write("\n".join(f"{v:04x}" for v in w.reshape(-1)) + "\n")
    r = ((w >> 11) & 31) * 255 // 31
    g = ((w >> 5) & 63) * 255 // 63
    b = (w & 31) * 255 // 31
    Image.fromarray(np.stack([r, g, b], -1).astype(np.uint8)).save(f"{DATA}/{name}.png")


ARRAY_W, ARRAY_H = 656, 488
MARGIN_X, MARGIN_Y = 8, 0      # where ov7670_model reads the 640x480 window from,
                               # with the course list (VSTRT/VREF, mirror + flip);
                               # measured with top_led_photo_tb


def save_scene(name, img):
    """RGB888 sensor-array scene whose mirrored + flipped readout is img."""
    q = np.clip(np.rint(img), 0, 255).astype(np.uint8)
    rot = q[::-1, ::-1]                                    # mirror + flip
    arr = np.pad(rot, ((MARGIN_Y, ARRAY_H - H - MARGIN_Y),
                       (MARGIN_X, ARRAY_W - W - MARGIN_X), (0, 0)), mode="edge")
    with open(f"{DATA}/{name}_scene.hex", "w") as f:
        f.write("\n".join(f"{r:02x}{g:02x}{b:02x}" for r, g, b in arr.reshape(-1, 3)) + "\n")


def main():
    base = load_base()
    meta = {}
    for name, leds in SCENES.items():
        img = base.copy()
        for led in leds:
            img = draw_led(img, *led)
        save(name, img)
        save_scene(name, img)
        meta[name] = [{"colour": c, "cx": x, "cy": y, "rad": r} for c, x, y, r in leds]
        print(f"wrote {DATA}/{name}.hex")
    with open(f"{DATA}/scenes.json", "w") as f:
        json.dump(meta, f, indent=2)


if __name__ == "__main__":
    main()
