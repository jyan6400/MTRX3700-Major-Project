
import argparse
import csv
from collections import deque
from pathlib import Path

import numpy as np
from PIL import Image

W, H = 320, 240

def load_frame(path):
    words = np.array(
        [int(s, 16) for s in Path(path).read_text().split()],
        dtype=np.uint16
    )
    if words.size != W * H:
        raise ValueError(f"Expected {W*H} pixels, got {words.size}")
    return words.reshape(H, W)

def red_mask(frame):
    r = (frame >> 11) & 31
    g6 = (frame >> 5) & 63
    g5 = (frame >> 6) & 31
    b = frame & 31
    return (
        (r >= 6) &
        (g6 <= 32) &
        (b <= 20) &
        (r >= g5 + 6) &
        (r >= b + 6)
    )

def find_components(mask):
    visited = np.zeros_like(mask, dtype=bool)
    components = []
    offsets = [
        (-1,-1),(-1,0),(-1,1),
        (0,-1),       (0,1),
        (1,-1), (1,0), (1,1)
    ]

    for sy, sx in zip(*np.nonzero(mask)):
        if visited[sy, sx]:
            continue

        q = deque([(int(sy), int(sx))])
        visited[sy, sx] = True
        pixels = []

        while q:
            y, x = q.popleft()
            pixels.append((y, x))

            for dy, dx in offsets:
                ny, nx = y + dy, x + dx
                if 0 <= ny < H and 0 <= nx < W:
                    if mask[ny, nx] and not visited[ny, nx]:
                        visited[ny, nx] = True
                        q.append((ny, nx))

        components.append(pixels)

    return sorted(components, key=len, reverse=True)

def evaluate(pixels):
    if not pixels:
        return None

    a = np.asarray(pixels)
    ys, xs = a[:, 0], a[:, 1]
    n = len(pixels)
    w = int(xs.max() - xs.min() + 1)
    h = int(ys.max() - ys.min() + 1)
    conf = min(255, 256 * n // (w * h))
    aspect_ok = w <= 4*h and h <= 4*w

    return dict(
        pixels=n,
        cx=int(xs.sum() // n),
        cy=int(ys.sum() // n),
        bbox=f"{xs.min()},{ys.min()},{xs.max()},{ys.max()}",
        confidence=conf,
        accepted=20 <= n <= 40000 and aspect_ok and conf >= 64
    )

def main():
    p = argparse.ArgumentParser()
    p.add_argument("--capture", action="append", required=True)
    p.add_argument("--output", default="build/v2_blob_analysis")
    args = p.parse_args()

    out = Path(args.output)
    out.mkdir(parents=True, exist_ok=True)
    results = []

    for filename in args.capture:
        path = Path(filename)
        mask = red_mask(load_frame(path))
        blobs = find_components(mask)
        largest = blobs[0] if blobs else []
        result = evaluate(largest)

        name = path.parent.name
        row = {
            "capture": name,
            "total_mask_pixels": int(mask.sum()),
            "component_count": len(blobs),
            "largest_blob_pixels": len(largest),
            "cx": result["cx"] if result else "",
            "cy": result["cy"] if result else "",
            "bbox": result["bbox"] if result else "",
            "confidence": result["confidence"] if result else 0,
            "accepted": result["accepted"] if result else False
        }
        results.append(row)

        selected = np.zeros_like(mask, dtype=np.uint8)
        for y, x in largest:
            selected[y, x] = 255
        Image.fromarray(selected).save(out / f"{name}_largest_blob.png")

        print(
            f"{name:34s} blobs={len(blobs):3d} "
            f"largest={len(largest):3d} "
            f"confidence={row['confidence']:3d} "
            f"accepted={row['accepted']}"
        )

    with (out / "blob_results.csv").open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=results[0].keys())
        writer.writeheader()
        writer.writerows(results)

if __name__ == "__main__":
    main()
