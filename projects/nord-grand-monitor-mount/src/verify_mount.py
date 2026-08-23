#!/usr/bin/env python3
"""Verify bolt access and face relief on the v6 base."""
import numpy as np, trimesh, sys

OUT='../stl/'
PLATE_T, DRIVER_R, DRIVER_L, RELIEF_D, LAND_R = 12.0, 9.0, 26.0, 4.0, 10.5
BOLTS=[(0.0,16.0),(0.0,56.0)]

m=trimesh.load(OUT+'v7_A_base.stl'); m.process(validate=True); m.merge_vertices()
print('faces %d | depth in Y: 0.0 (piano face) .. %.1f\n'%(len(m.faces), m.bounds[1][1]))

def inside(pts, chunk=800):
    out=[]
    for i in range(0,len(pts),chunk):
        out.append(m.contains(pts[i:i+chunk]))
    return np.concatenate(out)

fail=0; rng=np.random.default_rng(0)

ys=np.arange(-2, m.bounds[1][1]+4, 0.5)
for bx,bz in BOLTS:
    p=np.column_stack([np.full_like(ys,bx),ys,np.full_like(ys,bz)])
    h=ys[inside(p)]; ok=not len(h); fail+=(not ok)
    print('bolt (%.0f,%2.0f)  axis clear, full depth      : %s%s'%(bx,bz,'PASS' if ok else 'FAIL',
          '' if ok else '  blocked %.1f..%.1f'%(h.min(),h.max())))

N=1600
for bx,bz in BOLTS:
    t=rng.uniform(0,2*np.pi,N); r=DRIVER_R*np.sqrt(rng.uniform(0,1,N))
    y=rng.uniform(PLATE_T+0.5,PLATE_T+DRIVER_L,N)
    p=np.column_stack([bx+r*np.cos(t),y,bz+r*np.sin(t)])
    n=int(inside(p).sum()); fail+=(n>0)
    print('bolt (%.0f,%2.0f)  driver envelope r9 x 26     : %s (%d/%d in material)'%(
        bx,bz,'PASS' if n==0 else 'FAIL',n,N))

# land proud, recess actually recessed
for bx,bz in BOLTS:
    t=rng.uniform(0,2*np.pi,400); r=rng.uniform(6.0,LAND_R-1.5,400)
    p=np.column_stack([bx+r*np.cos(t),np.full(400,0.6),bz+r*np.sin(t)])
    n=int(inside(p).sum()); ok=n>=392; fail+=(not ok)
    print('bolt (%.0f,%2.0f)  bearing land proud at y=0.6 : %s (%d/400 solid)'%(
        bx,bz,'PASS' if ok else 'FAIL',n))

xs=rng.uniform(12.0,24.5,400); zs=rng.uniform(30.0,42.0,400)
p=np.column_stack([xs,np.full(400,RELIEF_D-1.5),zs])
n=int(inside(p).sum()); ok=n==0; fail+=(not ok)
print('           recess clear at y=%.1f       : %s (%d/400 in material)'%(
    RELIEF_D-1.5,'PASS' if ok else 'FAIL',n))

p=np.column_stack([xs,np.full(400,RELIEF_D+2.0),zs])
n=int(inside(p).sum()); ok=n>=392; fail+=(not ok)
print('           web behind recess is solid   : %s (%d/400 solid)'%(
    'PASS' if ok else 'FAIL',n))

print('\n%s'%('ALL CHECKS PASS' if fail==0 else '%d CHECK(S) FAILED'%fail))
sys.exit(1 if fail else 0)
