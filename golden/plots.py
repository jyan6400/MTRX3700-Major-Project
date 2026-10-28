"""Figures for the golden-model notebook."""
from __future__ import annotations

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap

import model
import stimulus

CLASS_RGB = np.array([[0, 0, 0], [235, 45, 45], [40, 200, 70], [60, 120, 255]], np.uint8)


def show_classification_map(p: model.MaskParams = model.MaskParams(), blues=(0, 6, 12, 20, 31)):
    """led_colour_mask over all RGB565 codes: R5 (x) vs G6 (y) for a few B5 slices."""
    fig, axs = plt.subplots(1, len(blues), figsize=(3.2 * len(blues), 3.4), constrained_layout=True)
    r = np.arange(32)[None, :]
    g6 = np.arange(64)[:, None]
    for ax, b in zip(axs, blues):
        codes = (r << 11) | (g6 << 5) | b
        R, G, Bl = model.colour_classes(codes, p)
        cls = np.where(R, 1, np.where(G, 2, np.where(Bl, 3, 0)))
        ax.imshow(CLASS_RGB[cls], origin="lower", aspect="auto", interpolation="nearest")
        ax.set_title(f"B5 = {b}")
        ax.set_xlabel("R5")
    axs[0].set_ylabel("G6")
    fig.suptitle("led_colour_mask on every RGB565 code (black = no colour; red/green/blue = class)")
    return fig


def show_gate_gallery(results, frames, names, sws):
    """Frame, mask and result for the synthetic gate frames."""
    n = len(names)
    cols = 7
    rows = int(np.ceil(n / cols))
    fig, axs = plt.subplots(rows, cols, figsize=(2.6 * cols, 2.6 * rows), constrained_layout=True)
    axs = np.atleast_2d(axs)
    for i in range(rows * cols):
        ax = axs.flat[i]
        ax.set_xticks([]); ax.set_yticks([])
        if i >= n:
            ax.axis("off"); continue
        f = frames[i]
        m = model.frame_mask(f)
        img = (stimulus.to_rgb888(f) * 0.35).astype(np.uint8)
        img[m] = [255, 60, 60]
        ax.imshow(img, interpolation="nearest")
        r = results[i]
        if r["valid"]:
            ax.plot(r["x"], r["y"], "w+", ms=14, mew=2)
        from replay import reject_reason
        ax.set_title(f"{names[i]}  SW1={sws[i]}\n{'TARGET' if r['valid'] else 'none'} n={r['count']} conf={r['conf']}",
                     fontsize=8)
        ax.set_xlabel(reject_reason(r).replace("rejected: ", "").replace("accepted by SW1 bypass despite: ", "bypass: ")[:38],
                      fontsize=7)
    return fig


def show_capture_frame(cr, k, zoom=None):
    """Board capture frame k: what was recorded, board vs model mask, results."""
    cap = cr.cap
    f = cr.frames[k]
    img = cap.rebuilt_image(f)
    rec = cap.recorded_mask(f)
    mm = model.frame_mask(img)
    shown = stimulus.to_rgb888(img).astype(np.float32)
    shown[rec < 0] = 90                                  # not recorded: grey
    board = np.where(rec < 0, 0, np.where(rec == 1, 1, 2))
    model_img = np.where(rec < 0, 0, np.where(mm, 1, 2))
    disagree = (rec >= 0) & (rec != mm.astype(np.int8))
    cmap = ListedColormap([[0.35, 0.35, 0.35], [0.95, 0.2, 0.2], [0.1, 0.1, 0.1]])
    fig, axs = plt.subplots(1, 3, figsize=(17, 4.6), constrained_layout=True)
    axs[0].imshow(shown.astype(np.uint8), interpolation="nearest")
    axs[0].set_title(f"frame {f.index}: recorded pixels (grey = not recorded: provably not red)")
    axs[1].imshow(board, cmap=cmap, vmin=0, vmax=2, interpolation="nearest")
    axs[1].set_title("board red_mask (red), board not-red (black)")
    axs[2].imshow(model_img, cmap=cmap, vmin=0, vmax=2, interpolation="nearest")
    ys, xs = np.nonzero(disagree)
    axs[2].scatter(xs, ys, s=40, facecolors="none", edgecolors="yellow")
    axs[2].set_title(f"golden model on the same pixels: {int(disagree.sum())} disagree (ringed)")
    b, m = f.result, cr.model.results[k]
    for ax in axs:
        ax.set_xticks([]); ax.set_yticks([])
        if b["valid"]:
            ax.plot(b["x"], b["y"], "+", color="cyan", ms=18, mew=2, label="board target")
        if m["valid"]:
            ax.plot(m["x"], m["y"], "x", color="yellow", ms=10, mew=1.5, label="model target")
        if m["count"]:
            ax.add_patch(plt.Rectangle((m["bx0"] - .5, m["by0"] - .5), m["bx1"] - m["bx0"] + 1, m["by1"] - m["by0"] + 1,
                                       fill=False, ec="orange", lw=1, ls="--"))
        if zoom:
            x0, y0, w, h = zoom
            ax.set_xlim(x0, x0 + w); ax.set_ylim(y0 + h, y0)
    if axs[0].get_legend_handles_labels()[0]:
        axs[0].legend(loc="lower left", fontsize=8)
    return fig


def show_timeline(mrec, title, rrec=None):
    """Adapter closes and centroid results over time (model; RTL overlaid)."""
    fig, ax = plt.subplots(figsize=(14, 2.6), constrained_layout=True)
    P = mrec["P"]
    if len(P):
        ax.plot(P[:, 0], np.full(len(P), 0), "|", color="0.7", ms=6, label="accepted pixels")
    for c in mrec["S"][:, 0]:
        ax.plot(c, 1, "v", color="tab:blue", ms=9)
    for c, g in mrec["E"]:
        ax.plot(c, 2, "o" if g else "x", color="tab:green" if g else "tab:red", ms=9, mew=2)
    for row in mrec["R"]:
        ax.plot(row[0], 3, "s", color="tab:purple" if row[1] else "0.4", ms=9)
    if rrec is not None:
        for row in rrec["R"]:
            ax.plot(row[0], 3.25, "s", mfc="none", mec="k", ms=9)
    ax.set_yticks([0, 1, 2, 3])
    ax.set_yticklabels(["pixels", "frame_start", "frame_end (o good, x rejected)", "frame_done (filled: target)"])
    ax.set_xlabel("PCLK cycle")
    ax.set_title(title + ("   (open squares: RTL)" if rrec is not None else ""))
    ax.set_ylim(-0.6, 3.7)
    return fig
