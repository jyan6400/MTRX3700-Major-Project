#!/usr/bin/env python3
"""Per-frame V2 RGB565 red-mask + centroid gate diagnostics.
Uses read_hex/mask_for/gates/parse_pos/roi_mask in camera_threshold_sweep.py
in same directory. ROI coordinates: x0,y0,x1,y1 (exclusive max bounds).
"""
import argparse, csv
from pathlib import Path
import numpy as np
from PIL import Image
from camera_threshold_sweep import read_hex,mask_for,gates,parse_pos,roi_mask

p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--positive',action='append',required=True,help='FILE.hex:X0,Y0,X1,Y1')
p.add_argument('--negative',action='append',default=[],help='FILE.hex with LED off')
p.add_argument('--output',default='build/v2_camera_candidate')
p.add_argument('--red-min',type=int,default=6)
p.add_argument('--green6-max',type=int,default=32)
p.add_argument('--blue-max',type=int,default=20)
p.add_argument('--margin',type=int,default=6)
a=p.parse_args()
threshold=(a.red_min,a.green6_max,a.blue_max,a.margin)
out=Path(a.output);out.mkdir(parents=True,exist_ok=True)
rows=[]
for i,item in enumerate(a.positive,1):
    path,roi=parse_pos(item)
    channels=read_hex(path); mask=mask_for(channels,threshold); region=roi_mask(roi)
    info=gates(mask)
    row=dict(capture=path.parent.name,condition='LED_ON',roi=str(roi),in_roi=int((mask & region).sum()),outside_roi=int((mask & ~region).sum()),**info)
    rows.append(row)
    Image.fromarray(np.uint8(mask)*255).save(out/f'{i:02d}_{path.parent.name}_mask.png')
for j,item in enumerate(a.negative,1):
    path=Path(item); mask=mask_for(read_hex(path),threshold);info=gates(mask)
    rows.append(dict(capture=path.parent.name,condition='LED_OFF',roi='',in_roi='',outside_roi=int(mask.sum()),**info))
    Image.fromarray(np.uint8(mask)*255).save(out/f'neg{j:02d}_{path.parent.name}_mask.png')
with (out/'candidate_per_frame.csv').open('w',newline='',encoding='utf8') as f:
    writer=csv.DictWriter(f,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
print('Threshold:',threshold,'| 320x240 decimated FrameGrab frame | V2 gates measured on entire image')
for row in rows:
    print(f"{row['capture']:<33} {row['condition']:<7} ROI={str(row['in_roi']):>4} outside={row['outside_roi']:>4} total={row['count']:>4} bbox={str(row['bbox']):<20} aspect={str(row['aspect']):>5} conf={row['confidence']:>3} accepted={str(row['accepted']):<5} reasons={row['reject']}")
print('Wrote:',out/'candidate_per_frame.csv')
