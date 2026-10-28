"""Offline controlled LED ON/OFF change test on corrected 320x240 FrameGrab PNGs.
Outputs pixel-change maps, per-pair metrics and negative OFF/OFF controls.
These are manually switched stills, not a demonstration of live temporal RTL.
"""
import argparse
import csv
from pathlib import Path
import numpy as np
from PIL import Image


def image(path):
    a=np.asarray(Image.open(path).convert('RGB'),dtype=np.float32)
    if a.shape[:2] != (240,320):
        raise ValueError(f'Expected 320x240: {path}, got {a.shape}')
    return a


def score(on,off):
    # Positive red-channel increase is the primary signal.
    # Subtract the common-mode change in green/blue to limit global
    # illumination/exposure differences, without requiring red-mask membership.
    delta=on-off
    return np.maximum(0., delta[:,:,0] - 0.5*(delta[:,:,1]+delta[:,:,2]))


def largest(mask):
    from collections import deque
    visited=np.zeros_like(mask,dtype=bool)
    best=[]
    h,w=mask.shape
    for sy,sx in zip(*np.nonzero(mask)):
        if visited[sy,sx]: continue
        q=deque([(int(sy),int(sx))]); visited[sy,sx]=True; comp=[]
        while q:
            y,x=q.popleft();comp.append((y,x))
            for dy in (-1,0,1):
                for dx in (-1,0,1):
                    if not dy and not dx:continue
                    ny,nx=y+dy,x+dx
                    if 0<=ny<h and 0<=nx<w and mask[ny,nx] and not visited[ny,nx]:
                        visited[ny,nx]=True;q.append((ny,nx))
        if len(comp)>len(best):best=comp
    if not best: return 0,None,None
    pts=np.array(best);return len(best),float(pts[:,1].mean()),float(pts[:,0].mean())


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--captures',required=True,help='Folder with renamed *_frame.png from ZIP')
    p.add_argument('--output',default='build/v2_temporal_led')
    p.add_argument('--threshold',type=float,default=35.)
    p.add_argument('--led-x',type=int,default=154)
    p.add_argument('--led-y',type=int,default=122)
    p.add_argument('--radius',type=int,default=12)
    a=p.parse_args()
    src=Path(a.captures);out=Path(a.output);out.mkdir(parents=True,exist_ok=True)
    ons=[image(src/f'v2_distractor_led_on_{i:02}_frame.png') for i in range(1,6)]
    offs=[image(src/f'v2_distractor_led_off_corrected_{i:02}_frame.png') for i in range(1,6)]
    yy,xx=np.indices((240,320))
    target=(xx-a.led_x)**2+(yy-a.led_y)**2<=a.radius*a.radius
    distractor=(xx-52)**2+(yy-157)**2<=20**2
    rows=[]
    def evaluate(label,index,on,off):
        s=score(on,off)
        m=s>=a.threshold
        n,cx,cy=largest(m)
        ys,xs=np.where(m)
        row={'type':label,'pair':index,'max_score':round(float(s.max()),1),
            'target_peak':round(float(s[target].max()),1),
            'target_above_threshold':int(m[target].sum()),
            'distractor_above_threshold':int(m[distractor].sum()),
            'all_above_threshold':int(m.sum()),
            'largest_component_pixels':n,'largest_cx':round(cx,1) if cx is not None else '',
            'largest_cy':round(cy,1) if cy is not None else ''}
        rows.append(row)
        Image.fromarray(np.uint8(np.clip(s*3,0,255))).save(out/f'{label}_{index:02}_red_change.png')
        Image.fromarray(np.uint8(m)*255).save(out/f'{label}_{index:02}_threshold.png')
        print(f'{label:7s} {index}: LED_ROI={row["target_above_threshold"]:4d} '
              f'distractor_ROI={row["distractor_above_threshold"]:4d} '
              f'largest={n:4d} centroid=({row["largest_cx"]},{row["largest_cy"]})')
    for i in range(5): evaluate('ON_OFF',i+1,ons[i],offs[i])
    # Null comparisons: two distinct OFF frames should not contain an activated LED.
    for i in range(4): evaluate('OFF_OFF',i+1,offs[i+1],offs[i])
    with (out/'temporal_results.csv').open('w',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=rows[0].keys());writer.writeheader();writer.writerows(rows)
    print(f'\nSaved {out / "temporal_results.csv"}')
    print('Interpretation: compare ON_OFF LED_ROI to OFF_OFF LED_ROI; inspect masks before concluding target identification.')

if __name__=='__main__':main()
