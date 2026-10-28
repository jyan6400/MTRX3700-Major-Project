#!/usr/bin/env python3
"""RGB565 V2 red-mask parameter sweep against FrameGrab images.

Dependencies: numpy, pillow (optional visualization only).
Positive inputs: --positive PATH:X0,Y0,X1,Y1 (ROI enclosing expected LED).
Negative inputs: --negative PATH (target absent; recommended).
Outputs: baseline.csv, sweep.csv, top masks PNGs, summary.txt.
This models the combinational V2 red mask + frame centroid gates only.
Does not model camera stream timing, CDC, or 640x480 source decimation.
"""
import argparse, csv
from pathlib import Path
import numpy as np
from PIL import Image

W,H=320,240
BASE=(20,24,12,8)

def read_hex(p):
    words=np.fromiter((int(x.strip(),16) for x in Path(p).read_text().splitlines() if x.strip()),dtype=np.uint16)
    if words.size != W*H: raise ValueError(f'{p}: expected {W*H} words, got {words.size}')
    a=words.reshape(H,W)
    return a>>11, (a>>5)&63, a&31, a

def mask_for(ch,t):
    r,g6,b,_=ch
    g5=g6>>1
    rmin,gmax,bmax,margin=t
    return (r>=rmin)&(g6<=gmax)&(b<=bmax)&(r>=g5+margin)&(r>=b+margin)

def gates(m, bypass=False):
    yy,xx=np.nonzero(m)
    n=len(xx)
    if n==0:
        return dict(count=0,x='',y='',bbox='',aspect='',confidence=0,accepted=False,reject='too few; empty')
    xmin,xmax=int(xx.min()),int(xx.max())
    ymin,ymax=int(yy.min()),int(yy.max())
    w=xmax-xmin+1;h=ymax-ymin+1
    conf=min(255,256*n//(w*h))
    reasons=[]
    if n<20: reasons.append('too few')
    if n>40000: reasons.append('too many')
    if w>4*h or h>4*w: reasons.append('aspect')
    if conf<64: reasons.append('low fill')
    accepted=(n>=20) if bypass else not reasons
    return dict(count=n,x=int(xx.sum()//n),y=int(yy.sum()//n),bbox=f'{xmin},{ymin},{xmax},{ymax}',aspect=round(max(w/h,h/w),2),confidence=conf,accepted=accepted,reject='; '.join(reasons) or 'none')

def parse_pos(s):
    path,coords=s.rsplit(':',1)
    x0,y0,x1,y1=map(int,coords.split(','))
    if not(0<=x0<x1<=W and 0<=y0<y1<=H):raise ValueError('ROI must be x0,y0,x1,y1 inside 320x240; x1/y1 exclusive')
    return Path(path),(x0,y0,x1,y1)

def roi_mask(r):
    x0,y0,x1,y1=r
    a=np.zeros((H,W),dtype=bool);a[y0:y1,x0:x1]=True
    return a

def write_csv(path,rows):
    if not rows:return
    with open(path,'w',newline='',encoding='utf8') as f:
        writer=csv.DictWriter(f,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)

def main():
    p=argparse.ArgumentParser(description=__doc__,formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--positive',action='append',required=True,help='PATH:X0,Y0,X1,Y1 enclosing LED (repeatable)')
    p.add_argument('--negative',action='append',default=[],help='PATH image with target OFF (repeatable)')
    p.add_argument('--output',default='camera_sweep_output')
    args=p.parse_args()
    pos=[(str(path),read_hex(path),roi_mask(roi)) for path,roi in map(parse_pos,args.positive)]
    neg=[(str(path),read_hex(path)) for path in map(Path,args.negative)]
    out=Path(args.output);out.mkdir(parents=True,exist_ok=True)
    # Dimensions here represent FrameGrab's 320x240 DECIMATED data.
    thresholds=[(r,g,b,d) for r in (6,10,14,18,20,24) for g in (16,24,32,40,48,56) for b in (8,12,16,20,24,31) for d in (2,4,6,8,10)]
    rows=[]
    for t in thresholds:
        pos_counts=[]; pos_out=[]; pos_gate=[]
        for name,ch,roi in pos:
            m=mask_for(ch,t); within=int((m&roi).sum());outside=int((m&~roi).sum())
            pos_counts.append(within);pos_out.append(outside);pos_gate.append(gates(m)['accepted'])
        false_counts=[int(mask_for(ch,t).sum()) for name,ch in neg]
        row=dict(red_min=t[0],green6_max=t[1],blue_max=t[2],margin=t[3],
                 min_target_pixels=min(pos_counts),total_target_pixels=sum(pos_counts),
                 max_outside_roi=max(pos_out),max_negative_pixels=max(false_counts) if false_counts else '',
                 all_frames_pass_v2=int(all(pos_gate)),
                 captures=len(pos),negative_captures=len(neg))
        rows.append(row)
    # Favor robust target visibility first, then a small false positive burden.
    rows.sort(key=lambda x:(-x['all_frames_pass_v2'],x['max_negative_pixels'] if neg else x['max_outside_roi'],x['max_outside_roi'],-x['min_target_pixels']))
    write_csv(out/'sweep.csv',rows)
    base_rows=[]
    for name,ch,roi in pos:
        m=mask_for(ch,BASE);g=gates(m)
        base_rows.append(dict(file=name,condition='positive',in_roi=int((m&roi).sum()),outside_roi=int((m&~roi).sum()),**g))
    for name,ch in neg:
        m=mask_for(ch,BASE);g=gates(m)
        base_rows.append(dict(file=name,condition='negative',in_roi='',outside_roi=int(m.sum()),**g))
    write_csv(out/'baseline.csv',base_rows)
    for rank,row in enumerate(rows[:3],start=1):
        t=(row['red_min'],row['green6_max'],row['blue_max'],row['margin'])
        for i,(name,ch,roi) in enumerate(pos):
            m=mask_for(ch,t)
            Image.fromarray((m*255).astype('uint8'),'L').save(out/f'top{rank}_capture{i+1}_mask.png')
    lines=['V2 RGB565 sensitivity sweep (320x240 decimated FrameGrab data)',
           f'Current V2 red mask: red>=20; green6<=24; blue<=12; red>=green5+8 and red>=blue+8',
           'V2 centroid gates: pixels 20..40000; aspect <=4:1; fill confidence >=64/256.',
           'No post-classification colour_run_filter is connected in the current V2 top-level.',
           'ROI serves as approximate target label; outside ROI is a proxy, not verified false-positive ground truth.',
           'Do not select thresholds without target-OFF negative captures and multiple lighting conditions.',
           'Do not compare 320x240 count thresholds directly with 640x480 without accounting for sampling.',
           '', 'CURRENT SETTINGS:']
    lines.extend(str(x) for x in base_rows)
    lines.append('\nTOP CANDIDATES (hypotheses for replay, not approved RTL changes):')
    lines.extend(str(x) for x in rows[:10])
    (out/'summary.txt').write_text('\n'.join(lines)+'\n',encoding='utf8')
    print('\n'.join(lines[:11])); print(f'\nWrote {out.resolve()}')

if __name__=='__main__':main()
