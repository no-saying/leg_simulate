#!/usr/bin/env python3
"""从控制器日志绘制二维平面轨迹。

cyan/blue 是控制里程计 ox/oy；magenta/red 是仿真真值 bx/by，仅用于对照。
"""
import re, sys
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

def load(path):
    rows=[]
    for line in Path(path).read_text(errors='ignore').splitlines():
        if not line.startswith('[t='):
            continue
        d={'t': float(re.search(r'\[t=([0-9.]+)\]', line).group(1))}
        for k in ['bx','by','ox','oy','xh','es','vh','yaw','tp']:
            m=re.search(rf'{k}=([+\-0-9.]+)', line)
            if m: d[k]=float(m.group(1))
        rows.append(d)
    return rows

def main():
    if len(sys.argv) < 2:
        print('usage: plot_odom_plane.py <log> [out.png]'); sys.exit(1)
    path=sys.argv[1]
    out=sys.argv[2] if len(sys.argv)>2 else str(Path(path).with_suffix('')) + '_plane.png'
    rows=load(path)
    if not rows:
        print('no rows'); sys.exit(2)
    ox=[r.get('ox',0) for r in rows]
    oy=[r.get('oy',0) for r in rows]
    bx=[r.get('bx',0) for r in rows]
    by=[r.get('by',0) for r in rows]
    fig, ax = plt.subplots(figsize=(7.5,7.0), dpi=140)
    ax.plot(ox, oy, '-', color='#1aa3ff', lw=1.8, label='odom (ox,oy)')
    if any(abs(y)>1e-6 for y in by) or True:
        ax.plot(bx, by, '-', color='#e23a8c', lw=1.2, alpha=0.8, label='true (bx,by)')
    ax.scatter([ox[0]],[oy[0]], c='#1aa3ff', s=40, zorder=5)
    ax.scatter([ox[-1]],[oy[-1]], c='#0b3d91', s=55, zorder=6, label=f'odom now ({ox[-1]:+.2f},{oy[-1]:+.2f})')
    ax.scatter([bx[-1]],[by[-1]], c='#9b1048', s=55, zorder=6, label=f'true now ({bx[-1]:+.2f},{by[-1]:+.2f})')
    # origin + hold band
    ax.scatter([0],[0], c='gold', s=70, marker='x', zorder=7, label='origin')
    band=plt.Rectangle((-0.1,-0.1),0.2,0.2, fill=False, ec='#22cc66', lw=1.5, label='±0.1 m band')
    ax.add_patch(band)
    ax.set_aspect('equal', adjustable='datalim')
    ax.grid(True, alpha=0.3)
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_title(f'Plane map  |  {Path(path).name}')
    ax.legend(loc='best', fontsize=8)
    fig.tight_layout()
    fig.savefig(out)
    print('wrote', out)
    print(f'odom start=({ox[0]:+.3f},{oy[0]:+.3f}) end=({ox[-1]:+.3f},{oy[-1]:+.3f})')
    print(f'true start=({bx[0]:+.3f},{by[0]:+.3f}) end=({bx[-1]:+.3f},{by[-1]:+.3f})')
    print(f'|odom|max={max((x*x+y*y)**0.5 for x,y in zip(ox,oy)):.3f}  |true|max={max((x*x+y*y)**0.5 for x,y in zip(bx,by)):.3f}')

if __name__ == '__main__':
    main()
