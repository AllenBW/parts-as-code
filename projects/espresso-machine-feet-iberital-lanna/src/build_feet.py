#!/usr/bin/env python3
"""
Espresso machine foot -- Iberital L'Anna (2003). Python port of the V14
OpenSCAD design (Jan 2026 session), same parameters and assembly:

  1. Foot sits cup-down (open bottom on the counter)
  2. 3/8-16 hex bolt slides UP through the hub; head keys into a hex channel
  3. Shaft passes through the ceiling and top flange into the machine's boss
  4. Perimeter-only top flange presses on the machine's base panel; its open
     centre clears the threaded boss that protrudes ~8 mm

NOTE: this port regenerates the design -- STLs are equivalent, not
byte-identical to the January prints. CEILING_T was recorded as 4 mm in the
V14 file text and 8 mm in the session summary; 8 is used here (stronger).
Verify against an in-service foot with calipers before reprinting.
"""

import numpy as np
import trimesh
from trimesh.creation import cylinder, extrude_polygon
from shapely.geometry import Point, box as sbox, Polygon

# ---------------- parameters (V14) ----------------
SHAPE          = 'rounded_square'   # round | square | hex | rounded_square
FOOT_W         = 50.0               # round variant uses 45
BODY_H         = 50.0
WALL           = 4.0
CORNER_R       = 8.0
FLARE          = 3.0                # flared base: +3 over the bottom 10 mm
FLARE_H        = 10.0
SHAFT_D        = 10.5               # 3/8" = 9.525 + clearance
HEAD_AF        = 15.0               # 9/16" head = 14.3 + clearance
CEILING_T      = 8.0                # <-- 4 in V14 file text, 8 in summary
FLANGE_H       = 10.0
FLANGE_WALL    = 4.0
HUB_OD         = 26.0
HUB_DEPTH      = 20.0
N_RIBS         = 6
RIB_T          = 4.0
SEG            = 128


def profile(width):
    h = width / 2.0
    if SHAPE == 'round':
        return Point(0, 0).buffer(h, quad_segs=SEG // 2)
    if SHAPE == 'square':
        return sbox(-h, -h, h, h)
    if SHAPE == 'hex':
        a = np.linspace(0, 2 * np.pi, 7)[:6]
        r = h / np.cos(np.pi / 6)
        return Polygon(np.column_stack([r * np.cos(a), r * np.sin(a)]))
    return sbox(-h, -h, h, h).buffer(-CORNER_R, quad_segs=32).buffer(CORNER_R, quad_segs=32)


def _ex(poly, z0, z1):
    m = extrude_polygon(poly, height=z1 - z0)
    m.apply_translation([0, 0, z0])
    return m


def _hex_prism(af, z0, z1):
    r = af / np.sqrt(3.0)
    a = np.linspace(0, 2 * np.pi, 7)[:6] + np.pi / 6
    return _ex(Polygon(np.column_stack([r * np.cos(a), r * np.sin(a)])), z0, z1)


def build():
    W = 45.0 if SHAPE == 'round' else FOOT_W
    outer, inner = profile(W), profile(W - 2 * WALL)
    H = BODY_H

    body = [_ex(outer, 0, H)]
    # flared base: stepped loft over the bottom FLARE_H, widest at the counter
    n = 6
    for i in range(n):
        w = W + 2 * FLARE * (1 - (i + 0.5) / n)
        body.append(_ex(profile(w), i * FLARE_H / n, (i + 1) * FLARE_H / n))
    # top flange ring, perimeter only -- open centre clears the machine boss
    body.append(_ex(outer.difference(profile(W - 2 * FLANGE_WALL)), H, H + FLANGE_H))

    hub = cylinder(radius=HUB_OD / 2, height=HUB_DEPTH + CEILING_T, sections=SEG)
    hub.apply_translation([0, 0, H - CEILING_T - HUB_DEPTH + (HUB_DEPTH + CEILING_T) / 2])
    body.append(hub)

    ribs = []
    for i in range(N_RIBS):
        r = trimesh.creation.box(extents=[W, RIB_T, H - CEILING_T])
        r.apply_translation([W / 2, 0, (H - CEILING_T) / 2])
        r.apply_transform(trimesh.transformations.rotation_matrix(
            i * 2 * np.pi / N_RIBS, [0, 0, 1]))
        ribs.append(r)
    # clip ribs to the inner cavity so nothing pokes through the outer wall
    ribs = trimesh.boolean.intersection(
        [trimesh.boolean.union(ribs, engine='manifold'), _ex(inner, -1, H)],
        engine='manifold')
    body.append(ribs)

    solid = trimesh.boolean.union(body, engine='manifold')

    cavity = trimesh.boolean.difference(
        [_ex(inner, -1, H - CEILING_T),
         hub.copy(), ribs.copy()], engine='manifold')
    solid = trimesh.boolean.difference([solid, cavity], engine='manifold')

    cuts = [_hex_prism(HEAD_AF, -2, H - CEILING_T),               # hex channel, full hub
            cylinder(radius=SHAFT_D / 2, height=CEILING_T + FLANGE_H + 4,
                     sections=SEG)]
    cuts[1].apply_translation([0, 0, H - CEILING_T + (CEILING_T + FLANGE_H + 4) / 2 - 1])
    return trimesh.boolean.difference([solid] + cuts, engine='manifold')


def clean_export(mesh, path):
    m = mesh.copy()
    m.vertices = np.round(m.vertices, 4)
    m.merge_vertices()
    m.update_faces(m.nondegenerate_faces())
    m.update_faces(m.unique_faces())
    m.remove_unreferenced_vertices()
    m.fix_normals()
    m.export(path)
    chk = trimesh.load(path); chk.process(validate=True); chk.merge_vertices()
    return chk.is_watertight, chk.volume / 1000, chk.bounds[1] - chk.bounds[0]


if __name__ == '__main__':
    import sys, os
    out = sys.argv[1] if len(sys.argv) > 1 else '.'
    for shape in ('rounded_square', 'hex', 'square', 'round'):
        SHAPE = globals()['SHAPE'] = shape
        wt, vol, d = clean_export(build(), os.path.join(out, f'foot_v14_{shape}.stl'))
        print('foot_v14_%-15s %5.1f x %5.1f x %5.1f mm  %6.1f cm3  watertight=%s'
              % (shape, d[0], d[1], d[2], vol, wt))
