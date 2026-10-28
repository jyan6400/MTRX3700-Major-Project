"""Offline 8x6 spatial-bin candidate detector for FrameGrab 320x240 RGB565.
Compare two simple streaming-friendly candidate policies with largest-blob reference.
"""
import argparse
import csv
from pathlib import Path
import numpy as np

W, H, NX, NY = 320, 240, 8, 6
BW, BH = W // NX, H // NY

def load_rgb565(path):
    words = np.array([int(s, 16) for s in path.read_text().split()], dtype=np.uint16)
    if words.size != W * H:
        raise ValueError(f"{path}: expected {W*H} RGB565 words, found {words.size}")
    return words.reshape(H, W)

def classify(words):
    r = (words >> 11) & 31
    g6 = (words >> 5) & 63
    g5 = g6 >> 1
    b = words & 31
    return (r >= 6) & (g6 <= 32) & (b <= 20) & (r >= g5 + 6) & (r >= b + 6)

def metric(mask, min_pixels):
    ys, xs = np.nonzero(mask)
    n = len(xs)
    if not n:
        return dict(pixels=0, cx='', cy='', confidence=0, aspect_ok=False, accepted=False)
    width = int(xs.max() - xs.min() + 1)
    height = int(ys.max() - ys.min() + 1)
    confidence = min(255, 256*n // (width*height))
    aspect_ok = width <= 4*height and height <= 4*width
    return dict(pixels=n, cx=int(xs.sum() // n), cy=int(ys.sum() // n),
                confidence=confidence, aspect_ok=aspect_ok,
                accepted=(min_pixels <= n <= 40000 and confidence >= 64 and aspect_ok))

def group(name):
    for key in ('led_on_center','led_left','led_right','led_far','led_off'):
        if name.startswith(key):
            return key
    return 'other'

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--captures', required=True, type=Path, help='Folder with 35 capture subdirectories')
    ap.add_argument('--blob-csv', required=True, type=Path)
    ap.add_argument('--output', default='build/v2_spatial_bins', type=Path)
    ap.add_argument('--min-pixels', default=5, type=int)
    args = ap.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    reference = {}
    with args.blob_csv.open(newline='') as f:
        for r in csv.DictReader(f):
            reference[r['capture']] = r
    rows = []
    for capture in sorted(args.captures.iterdir()):
        if not capture.is_dir() or not (capture / 'frame.hex').exists():
            continue
        mask = classify(load_rgb565(capture / 'frame.hex'))
        counts = mask.reshape(NY, BH, NX, BW).sum(axis=(1,3))
        iy, ix = np.unravel_index(np.argmax(counts), counts.shape)
        # Compare (1) the strongest bin only and (2) the 3x3 bins centred on it.
        # This is not equivalent to 8-connected component selection.
        for policy in ('single_bin', 'neighbour_3x3'):
            if policy == 'single_bin':
                x0, x1 = ix*BW, (ix+1)*BW
                y0, y1 = iy*BH, (iy+1)*BH
            else:
                x0, x1 = max(0, (ix-1)*BW), min(W, (ix+2)*BW)
                y0, y1 = max(0, (iy-1)*BH), min(H, (iy+2)*BH)
            selected = np.zeros_like(mask)
            selected[y0:y1, x0:x1] = mask[y0:y1, x0:x1]
            result = metric(selected, args.min_pixels)
            ref = reference.get(capture.name, {})
            refx, refy = ref.get('cx',''), ref.get('cy','')
            error = ''
            if refx != '' and refy != '' and result['cx'] != '':
                error = round(float(np.hypot(int(result['cx'])-int(refx), int(result['cy'])-int(refy))), 2)
            rows.append(dict(capture=capture.name, group=group(capture.name), policy=policy,
                             top_bin_x=int(ix), top_bin_y=int(iy), top_bin_pixels=int(counts[iy,ix]),
                             total_red_pixels=int(mask.sum()), **result, ref_cx=refx, ref_cy=refy,
                             centroid_error_vs_blob_px=error))
    out = args.output/'spatial_bin_results.csv'
    with out.open('w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    print('Policy / group          Accepted / frames    Median centroid error vs blob (px)')
    for policy in ('single_bin','neighbour_3x3'):
        for g in ('led_on_center','led_left','led_right','led_far','led_off'):
            sub = [r for r in rows if r['policy']==policy and r['group']==g]
            if not sub: continue
            accepted = sum(r['accepted'] for r in sub)
            errs = [r['centroid_error_vs_blob_px'] for r in sub if r['centroid_error_vs_blob_px'] != '']
            median = f'{np.median(errs):.2f}' if errs else 'n/a'
            print(f'{policy:16s} {g:15s} {accepted:2d}/{len(sub):2d}             {median}')
    print(f'\nDetailed CSV: {out}')
    print('NOTE: Blob centroid is a reference prediction, not manually annotated ground truth.')

if __name__=='__main__':
    main()
