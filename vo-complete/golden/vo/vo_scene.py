"""Scenes for the beacon VO: the three LEDs drawn onto the lab photo from a known robot pose.

The world (mm): the LED line is along X at Y = 0, height H_LED; red at X = -300, green 0, blue +300
(SPACING apart). The robot's camera is at (X, Y) with Y < 0 (in front of the LEDs), height H_CAM,
heading psi (0 = square-on to the LED line, positive = turned left), tilted up by PITCH.

An LED is drawn as a solid disc of its colour, radius f * R_GLOW / depth pixels (a lit LED blooms
far wider than its 5 mm body), in exact integer form so the SystemVerilog bench draws the very same
pixels: centre (U, V) and radius R in 1/16 pixel, pixel (x, y) is lit when
(16x - U)^2 + (16y - V)^2 <= R^2. Ground truth (forward, lateral of each LED in the robot frame)
comes from the pose, not from the pixels.

Scene file (sim/vo benches read it):  one block per frame
    F <sel> <good> <en> <appr> <target> <n>
                              selected colour, frame_good (0 = the frame is cut short: the adapter
                              closes it empty), the stand-off inputs, discs
    D <colour> <rgb565> <U> <V> <R>
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import sys

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
sys.path.insert(0, str(HERE))

import stimulus           # noqa: E402
import vo_model as vm     # noqa: E402

W, H = 640, 480
LED_RGB565 = (int(stimulus.rgb565(255, 40, 30)), int(stimulus.rgb565(30, 230, 40)), int(stimulus.rgb565(30, 60, 255)))


@dataclass(frozen=True)
class Rig:
    H_CAM: float = 300.0
    H_LED: float = 450.0
    SPACING: float = 300.0
    PITCH_DEG: float = 12.0
    F_PX: float = 560.0
    CX: float = 319.5
    CY: float = 239.5
    R_GLOW: float = 15.0       # apparent LED radius, mm

    def led_world(self, k):
        return np.array([(k - 1) * self.SPACING, 0.0, self.H_LED])

    def geom_params(self, **kw):
        return vm.geom_params(self.F_PX, self.CX, self.CY, self.PITCH_DEG, round(self.H_LED - self.H_CAM), **kw)


def robot_frame(rig, X, Y, psi_deg, k):
    """True (forward, lateral, up) of LED k from the camera, mm, robot frame."""
    p = np.deg2rad(psi_deg)
    rel = rig.led_world(k)[:2] - np.array([X, Y])
    fwd_axis = np.array([-np.sin(p), np.cos(p)])
    right_axis = np.array([np.cos(p), np.sin(p)])
    return float(rel @ fwd_axis), float(rel @ right_axis), rig.H_LED - rig.H_CAM


def project(rig, forward, lateral, up):
    """Pixel (u, v) and depth of a point in the robot frame (camera tilted up by PITCH)."""
    a = np.deg2rad(rig.PITCH_DEG)
    z = forward * np.cos(a) + up * np.sin(a)
    yc = up * np.cos(a) - forward * np.sin(a)
    if z <= 1.0:
        return None
    return rig.CX + rig.F_PX * lateral / z, rig.CY - rig.F_PX * yc / z, z


def frame_discs(rig, X, Y, psi_deg, leds_on):
    """[(colour, rgb565, U, V, R)] of the LEDs lit and in front of the camera, and the truth."""
    discs, truth = [], []
    for k in range(3):
        f, l, u = robot_frame(rig, X, Y, psi_deg, k)
        truth.append(dict(forward=f, lateral=l))
        if not leds_on[k]:
            continue
        pr = project(rig, f, l, u)
        if pr is None:
            continue
        px, py, z = pr
        r = rig.F_PX * rig.R_GLOW / z
        U, V, R = int(round(px * 16)), int(round(py * 16)), int(round(r * 16))
        if -R < U < 16 * W + R and -R < V < 16 * H + R:
            discs.append((k, LED_RGB565[k], U, V, R))
    return discs, truth


def render(background, discs):
    f = background.copy()
    for (k, rgb, U, V, R) in discs:
        x0, x1 = max(0, (U - R) // 16 - 1), min(W - 1, (U + R) // 16 + 1)
        y0, y1 = max(0, (V - R) // 16 - 1), min(H - 1, (V + R) // 16 + 1)
        if x1 < x0 or y1 < y0:
            continue
        xs = np.arange(x0, x1 + 1)
        ys = np.arange(y0, y1 + 1)
        dx = 16 * xs[None, :] - U
        dy = 16 * ys[:, None] - V
        inside = dx * dx + dy * dy <= R * R
        sub = f[y0:y1 + 1, x0:x1 + 1]
        sub[inside] = rgb
    return f


def background():
    return stimulus.photo_frame([])


# ---------------------------------------------------------------- trajectories

@dataclass
class Scenario:
    name: str
    poses: list          # (X, Y, psi_deg) per frame
    leds_on: list        # (r, g, b) per frame
    sel: list            # selected colour per frame
    good: list           # frame_good per frame
    note: str = ""
    standoff: list = None    # (enable, approaching, target_mm) per frame, or None
    extra: list = None       # extra discs per frame [(colour, rgb565, U, V, R)]: distractors


def _line(a, b, n):
    return [a + (b - a) * t for t in np.linspace(0.0, 1.0, n)]


def scenarios(quick=False):
    """The bench's drives. Distances are forward ranges from the camera to the LED line."""
    S = []
    n = 45 if quick else 90
    # 1. the V4 reverse: docked square-on in front of green, back away to 1.2 m at 0.5 m/s
    ys = _line(-300.0, -1200.0, n)
    S.append(Scenario("reverse_green", [(0.0, y, 0.0) for y in ys], [(0, 1, 0)] * n, [1] * n, [1] * n,
                      "square-on reverse from 0.3 m to 1.2 m, only the selected LED lit; V4 stand-off at 1000 mm",
                      standoff=[(int(i >= 3), 0, 1000) for i in range(n)]))
    # 2. approach with an offset and a heading, all three lit, red selected
    m = 40 if quick else 80
    xs = _line(-150.0, -250.0, m)
    ys2 = _line(-1500.0, -450.0, m)
    ps = _line(8.0, 3.0, m)
    S.append(Scenario("approach_red_offset", [(xs[i], ys2[i], ps[i]) for i in range(m)], [(1, 1, 1)] * m,
                      [0] * m, [1] * m, "approach 1.5 m to 0.45 m, 150-250 mm off-centre, heading 8 to 3 deg, all lit"))
    # 3. dropouts, a bad frame and a bay change: blue selected, LED flickers off, then red selected
    q = 40
    ys3 = _line(-600.0, -900.0, q)
    on = [(1, 0, 1)] * q
    for i in (10, 11, 25, 26, 27, 28, 29):
        on[i] = (1, 0, 0)
    good = [1] * q
    good[18] = 0
    sel = [2] * 30 + [0] * 10
    S.append(Scenario("dropout_switch", [(100.0, y, 4.0) for y in ys3], on, sel, good,
                      "blue selected, off for 2 then 5 frames (coast, then drop), one rejected frame, switch to red"))
    # 4. the gates: distractor discs (a red one at the frame edge, a stretched blue bar, a far speck,
    #    a second green blob) around a slow approach to green with an approach-direction stand-off
    r = 12
    ys4 = _line(-1100.0, -700.0, r)
    ex = []
    for i in range(r):
        e = []
        if i % 3 == 0:
            e.append((0, LED_RGB565[0], 16 * 3, 16 * 200, 16 * 10))            # red, clipped by the left edge
        if i % 3 == 1:
            e += [(2, LED_RGB565[2], 16 * x, 16 * 420, 16 * 3) for x in range(80, 200, 4)]   # blue bar, aspect
        if i % 4 == 2:
            e.append((1, LED_RGB565[1], 16 * 500, 16 * 380, 16 * 6))           # a second green blob
        if i % 4 == 3:
            e.append((0, LED_RGB565[0], 16 * 100 + 7, 16 * 60 + 3, 16))        # red speck: too few pixels
        ex.append(e)
    S.append(Scenario("gates_distractors", [(0.0, y, 0.0) for y in ys4], [(1, 1, 1)] * r, [1] * r, [1] * r,
                      "distractors: clipped red at the edge, a blue bar, a second green blob, a red speck",
                      standoff=[(1, 1, 800)] * r, extra=ex))
    return S


def scenario_frames(sc, rig=Rig(), bg=None):
    """Discs and truth per frame."""
    out = []
    for i, (X, Y, psi) in enumerate(sc.poses):
        discs, truth = frame_discs(rig, X, Y, psi, sc.leds_on[i])
        if sc.extra:
            discs = discs + list(sc.extra[i])
        out.append((discs, truth))
    return out


def write_scene(path, sc, rig=Rig()):
    fr = scenario_frames(sc, rig)
    with open(path, "w") as f:
        f.write(f"# {sc.name}: {sc.note}\n")
        for i, (discs, _) in enumerate(fr):
            en, ap, tg = sc.standoff[i] if sc.standoff else (0, 0, 0)
            f.write(f"F {sc.sel[i]} {sc.good[i]} {en} {ap} {tg} {len(discs)}\n")
            for (k, rgb, U, V, R) in discs:
                f.write(f"D {k} {rgb} {U} {V} {R}\n")
    return fr


def write_background(path, bg):
    with open(path, "w") as f:
        f.write("\n".join(f"{int(p):04x}" for p in bg.ravel().tolist()) + "\n")
