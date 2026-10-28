
import argparse
import csv
from pathlib import Path

import numpy as np
from PIL import Image


def load_rgb565(path):
    lines = Path(path).read_text().split()
    words = np.array([int(x, 16) for x in lines], dtype=np.uint16)
    if len(words) != 320 * 240:
        raise ValueError(f"{path}: expected 76800 pixels, got {len(words)}")
    return words.reshape(240, 320)


def red_mask(words):
    r = (words >> 11) & 31
    g6 = (words >> 5) & 63
    g5 = (words >> 6) & 31
    b = words & 31

    return (
        (r >= 6)
        & (g6 <= 32)
        & (b <= 20)
        & (r >= g5 + 6)
        & (r >= b + 6)
    )


def run_filter(mask, length):
    if length == 1:
        return mask.copy()

    out = mask.copy()

    for offset in range(1, length):
        shifted = np.zeros_like(mask)
        shifted[:, offset:] = mask[:, :-offset]
        out &= shifted

    return out


def assess(mask, length):
    ys, xs = np.nonzero(mask)
    count = len(xs)

    if count == 0:
        return dict(count=0, aspect=0, confidence=0,
                    accepted=False, reasons="empty")

    # The RTL compensates the retained-pixel X coordinate by
    # floor((RUN_LEN-1)/2), saturating at zero.
    xcomp = (length - 1) // 2
    xs = np.maximum(xs - xcomp, 0)

    width = int(xs.max() - xs.min() + 1)
    height = int(ys.max() - ys.min() + 1)

    aspect = max(width / height, height / width)
    confidence = min(255, 256 * count // (width * height))

    reasons = []
    if count < 20:
        reasons.append("too few")
    if count > 40000:
        reasons.append("too many")
    if width > 4 * height or height > 4 * width:
        reasons.append("aspect")
    if confidence < 64:
        reasons.append("low fill")

    return dict(
        count=count,
        aspect=round(aspect, 2),
        confidence=confidence,
        accepted=not reasons,
        reasons="; ".join(reasons) if reasons else "none",
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--capture", action="append", required=True)
    parser.add_argument("--output", default="build/v2_run_filter")
    args = parser.parse_args()

    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=True)

    rows = []

    for path in args.capture:
        name = Path(path).parent.name
        words = load_rgb565(path)
        raw = red_mask(words)

        for length in (1, 2, 3, 4):
            filtered = run_filter(raw, length)
            result = assess(filtered, length)

            rows.append({
                "capture": name,
                "run_len": length,
                **result,
            })

            image = Image.fromarray(
                (filtered.astype(np.uint8) * 255), mode="L"
            )
            image.save(output / f"{name}_run{length}.png")

    with (output / "run_filter_comparison.csv").open(
        "w", newline=""
    ) as f:
        writer = csv.DictWriter(f, fieldnames=rows[0].keys())
        writer.writeheader()
        writer.writerows(rows)

    for row in rows:
        print(
            f"{row['capture']:34s} "
            f"RUN={row['run_len']} "
            f"pixels={row['count']:4d} "
            f"aspect={row['aspect']:5.2f} "
            f"confidence={row['confidence']:3d} "
            f"accepted={row['accepted']} "
            f"reason={row['reasons']}"
        )


if __name__ == "__main__":
    main()
