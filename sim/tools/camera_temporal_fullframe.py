"""Blind full-frame temporal red-change detection using saved FrameGrab PNGs.
No LED location enters the detection algorithm; expected point is used only to
report localization accuracy after detection.
"""
import argparse,csv
from pathlib import Path
from collections import deque
import numpy as np
from PIL import Image

def read(path):
    arr=np.asarray(Image.open(path).convert('RGB'),dtype=np.float32)
    if arr.shape!=(240,320,3):raise ValueError(f'{path}: expected 240x320 RGB image')
    return arr

def components(mask):
    h,w=mask.shape;seen=np.zeros((h,w),dtype=bool);result=[]
    for sy,sx in zip(*np.nonzero(mask)):
        if seen[sy,sx]:continue
        q=deque([(int(sy),int(sx))]);seen[sy,sx]=True;pix=[]
        while q:
            y,x=q.popleft();pix.append((y,x))
            for dy in (-1,0,1):
                for dx in (-1,0,1):
                    if dx==dy==0:continue
                    yy,xx=y+dy,x+dx
                    if 0<=yy<h and 0<=xx<w and mask[yy,xx] and not seen[yy,xx]:
                        seen[yy,xx]=True;q.append((yy,xx))
        result.append(pix)
    return sorted(result,key=len,reverse=True)

def main():
    p=argparse.ArgumentParser()
    p.add_argument('--captures',required=True)
    p.add_argument('--output',default='build/v2_temporal_fullframe')
    p.add_argument('--threshold',type=float,default=35)
    p.add_argument('--min-pixels',type=int,default=5)
    p.add_argument('--expected-x',type=float,default=154,help='Evaluation only; not used by detector')
    p.add_argument('--expected-y',type=float,default=127,help='Evaluation only; not used by detector')
    args=p.parse_args()
    out=Path(args.output);out.mkdir(parents=True,exist_ok=True)
    src=Path(args.captures)
    ons=[read(src/f'v2_distractor_led_on_{i:02}_frame.png') for i in range(1,6)]
    offs=[read(src/f'v2_distractor_led_off_corrected_{i:02}_frame.png') for i in range(1,6)]
    rows=[]
    def process(kind,i,a,b):
        d=a-b
        score=np.maximum(0,d[:,:,0]-0.5*(d[:,:,1]+d[:,:,2]))
        mask=score>=args.threshold
        comps=components(mask)
        chosen=next((c for c in comps if len(c)>=args.min_pixels),[])
        if chosen:
            points=np.array(chosen)
            cy,cx=points.mean(axis=0)
            error=float(np.hypot(cx-args.expected_x,cy-args.expected_y))
        else:cx=cy=error=None
        row={'type':kind,'pair':i,'total_above_threshold':int(mask.sum()),
             'components':len(comps),'largest_component_pixels':len(comps[0]) if comps else 0,
             'detected':bool(chosen),'cx':round(float(cx),2) if cx is not None else '',
             'cy':round(float(cy),2) if cy is not None else '',
             'error_to_reference_px':round(error,2) if error is not None else ''}
        rows.append(row)
        Image.fromarray((mask.astype('uint8')*255)).save(out/f'{kind}_{i:02}_mask.png')
        print(f'{kind:8} {i}: pixels={row["total_above_threshold"]:3} components={row["components"]:2} '
              f'largest={row["largest_component_pixels"]:2} detect={row["detected"]} '
              f'centroid=({row["cx"]},{row["cy"]}) error={row["error_to_reference_px"]}')
    for i in range(5):process('ON_OFF',i+1,ons[i],offs[i])
    for i in range(4):process('OFF_OFF',i+1,offs[i+1],offs[i])
    with (out/'fullframe_results.csv').open('w',newline='') as f:
        wr=csv.DictWriter(f,fieldnames=rows[0].keys());wr.writeheader();wr.writerows(rows)
    pos=sum(r['detected'] for r in rows if r['type']=='ON_OFF')
    false=sum(r['detected'] for r in rows if r['type']=='OFF_OFF')
    print(f'\nON/OFF detections: {pos}/5; OFF/OFF false detections: {false}/4')
    print('NOTE: stationary snapshots, known switching state; not a live FPGA validation.')
    print(f'Results: {out / "fullframe_results.csv"}')
if __name__=='__main__':main()
