#!/usr/bin/env python3
"""
Modular self-watering moss pole, v3 -- one design.

  laser   panel      6 per module, 3 mm sheet, honeycomb lattice. The skin.
  print   cage       base ring (female thread) + 6 slotted corner posts + tube
                     spider, one print. Panels slide down the posts. The bracket.
  print   top ring   captures the panel tops, carries the male thread.
  print   cap        a top ring with a reservoir cup instead of a thread.
  print   stake ring male thread up; the spike plate press-fits inside it.
  print   spike      plate + hollow drip fins; prints plate-down, fins up.
  print   tube       perforated water column, module length; rope wick inside.
  print   lid        drops into the cup; an inverted soda bottle hangs in it.

Stack:  spike + stake ring -> cage -> 6 panels -> top ring -> cage -> 6 panels -> ... -> cap
Every joint is measured on both sides by verify.py.

Module height is set by the sheet: the P2 bed is 600 x 308 mm, so a panel
<= 148 mm long fits four across a 12 x 24" sheet, six rows -> 24 panels ->
exactly four modules per sheet.  MODULE_H = 154 gives that.

Outputs ../stl/*.stl, ../laser/panel.dxf/.svg, ../laser/sheet_12x24.svg/.dxf.
"""

import math
import os
import numpy as np
import trimesh
from trimesh.creation import extrude_polygon, box
from trimesh.transformations import rotation_matrix
from shapely.geometry import Polygon, box as sbox
from shapely.ops import unary_union
from shapely.affinity import translate

from lib import (_ex, _cyl, _hex_poly, _hex_ring_poly, _union, _diff, _rotz,
                 clean_export, water_tube, bottle_lid, reservoir_cup,
                 TUBE_OD, TUBE_CLEAR, HUB_OD, WICK_HOLE_D, CUP_OD, CUP_H)
import lib
lib.CUP_WALL = 2.5          # 5 perimeters at 0.5 nozzle
lib.DRIP_D = 2.7            # side holes print small at 0.3 mm layers

OUT_STL = os.path.join(os.path.dirname(__file__), '..', 'stl')
OUT_LASER = os.path.join(os.path.dirname(__file__), '..', 'laser')

# ---------------- parameters ----------------
MODULE_H   = float(os.environ.get('MOSS_MODULE_H', 154))   # cage height = module pitch
AF_OUT     = 100.0          # across flats
SHEET_T    = 3.0            # laser sheet, nominal; measure yours
KERF       = 0.20
SLOT_W     = SHEET_T + 0.4  # slip fit; sheet + 0.15 for a friction fit
SLOT_D     = 6.0            # panel engagement in base ring and top ring
SLOT_OFF   = 4.5            # slot centreline inset from the outer face
PANEL_CLR  = 1.3            # corner gap between adjacent panels (sets panel width)
HEX_AF     = 13.0           # lattice openings
STRUT      = 2.5
MARGIN     = 2.5
BAND       = 10.0

WALL       = 13.0           # ring radial wall (across flats)
BASE_H     = 18.0           # cage base ring: thread in the bottom 8, slot in the top 6
TOP_H      = 10.0           # top ring below the collar
POST_AF    = 76.0           # posts reach in to this across-flats
POST_ANG   = 6.5            # post half-angle, degrees (fatter = stiffer tower at 0.3 mm layers)

# coarse thread for 0.3 mm layers: 45-degree flanks, root = crest + 2*depth <= pitch
TH_PITCH, TH_TURNS, TH_MINOR_R, TH_DEPTH, TH_CREST, TH_CLR = 5.0, 1.25, 42.0, 1.5, 1.0, 0.45
TH_COLLAR  = TH_PITCH * TH_TURNS + 1.0 + (TH_CREST / 2 + TH_DEPTH + 0.3) + 0.5   # ridge fully inside the collar: no nub past its end

FIN_L, FIN_T, FIN_N, FIN_R_IN, FIN_R_OUT, FIN_WALL, DRIP_D = 100.0, 7.0, 3, 10.0, 42.0, 2.0, 2.4   # 2.0 = 4 perimeters @0.5

SLOT_AF = AF_OUT - 2 * SLOT_OFF
SHEET_W, SHEET_H, GAP = 600.0, 305.0, 1.0      # P2 usable bed x 12" sheet


# ---------------- panel ----------------
def panel_width():
    side = SLOT_AF / math.sqrt(3.0)
    return side - SHEET_T * math.tan(math.pi / 6) - PANEL_CLR


def panel_height():
    return MODULE_H - (BASE_H - SLOT_D) + SLOT_D


def panel():
    W, H = panel_width(), panel_height()
    d = HEX_AF + STRUT; row = d * math.sqrt(3) / 2; R = HEX_AF / math.sqrt(3)
    y_lo, y_hi = BAND, H - BAND
    nrows = int((y_hi - y_lo - 2 * R) // row) + 1
    y0 = (y_lo + y_hi) / 2 - (nrows - 1) * row / 2
    holes = []
    for r in range(nrows):
        y = y0 + r * row; off = (d / 2) if r % 2 else 0.0
        for x in np.arange(-10, 11) * d + off + W / 2:
            if x - HEX_AF / 2 >= MARGIN and x + HEX_AF / 2 <= W - MARGIN and y - R >= y_lo and y + R <= y_hi:
                holes.append(translate(_hex_poly(HEX_AF, math.pi / 6), x, y))
    return sbox(0, 0, W, H).difference(unary_union(holes)), W, H


def kerf(poly):
    return poly.buffer(KERF / 2, join_style=2)


# ---------------- thread ----------------
def helical_thread(r_root, depth, pitch, turns, z0, segs=96):
    w_crest, eps = TH_CREST, 0.3
    w_root = w_crest + 2 * depth                 # 45-degree flanks: self-supporting at any layer height
    assert w_root + 2 * eps <= pitch - 0.3, 'thread flanks would merge'
    # the flank line continues at 45 degrees into the core so the union leaves a true 45-degree surface
    prof = np.array([[r_root - eps, -(w_root / 2 + eps)], [r_root + depth, -w_crest / 2],
                     [r_root + depth, w_crest / 2], [r_root - eps, w_root / 2 + eps]])
    n = int(segs * turns) + 1; k = len(prof)
    V, F = [], []
    for i in range(n):
        th = 2 * math.pi * i / segs; z = z0 + pitch * th / (2 * math.pi)
        for r, dz in prof:
            V.append([r * math.cos(th), r * math.sin(th), z + dz])
    for i in range(n - 1):
        for j in range(k):
            a, b = i * k + j, i * k + (j + 1) % k; c, d = (i + 1) * k + j, (i + 1) * k + (j + 1) % k
            F += [[a, c, b], [b, c, d]]
    F += [[0, 1, 2], [0, 2, 3]]; e = (n - 1) * k; F += [[e, e + 2, e + 1], [e, e + 3, e + 2]]
    m = trimesh.Trimesh(np.array(V), np.array(F), process=True); m.fix_normals()
    assert m.is_watertight
    return m


def male_thread(z0):
    return _union([_cyl(2 * TH_MINOR_R, z0, z0 + TH_COLLAR),
                   helical_thread(TH_MINOR_R, TH_DEPTH, TH_PITCH, TH_TURNS, z0 + 1.0)])


def female_cut(z0):
    """Groove phased to male_thread's ridge (both start 1 mm above the collar
    base at angle 0), with a half-turn lead-in that starts half a turn earlier
    in angle as well."""
    r = TH_MINOR_R + TH_CLR
    groove = helical_thread(r, TH_DEPTH, TH_PITCH, TH_TURNS + 0.5, z0 + 1.0 - 0.5 * TH_PITCH)
    groove.apply_transform(rotation_matrix(math.pi, [0, 0, 1]))
    return _union([_cyl(2 * r, z0 - 1, z0 + TH_COLLAR + 1), groove])


# ---------------- printed parts ----------------
def ring(z0, z1):
    return _ex(_hex_ring_poly(AF_OUT, AF_OUT - 2 * WALL), z0, z1)


def spider(z0, z1):
    hub = _cyl(HUB_OD, z0, z1); arms = []
    for i in range(3):
        a = box(extents=[(AF_OUT - 2 * WALL) / 2 + 2, 4.0, z1 - z0])
        a.apply_translation([((AF_OUT - 2 * WALL) / 2 + 2) / 2, 0, (z0 + z1) / 2])
        arms.append(_rotz(a, i * 2 * math.pi / 3))
    return _diff(_union([hub] + arms), [_cyl(TUBE_OD + TUBE_CLEAR, z0 - 1, z1 + 1)])


def face_slots(z0, z1):
    L = panel_width() + 1.0; cuts = []
    for i in range(6):
        s = box(extents=[SLOT_W, L, (z1 - z0) + 0.02]); s.apply_translation([SLOT_AF / 2, 0, (z0 + z1) / 2])
        cuts.append(_rotz(s, i * math.pi / 3))
    return cuts


def post(i):
    a = math.radians(30 + 60 * i); ha = math.radians(POST_ANG)
    wedge = Polygon([(0, 0), (70 * math.cos(a - ha), 70 * math.sin(a - ha)), (70 * math.cos(a + ha), 70 * math.sin(a + ha))])
    return _hex_ring_poly(AF_OUT, POST_AF).intersection(wedge)


def build_cage():
    H = MODULE_H
    body = _union([ring(0, BASE_H)] + [_ex(post(i), BASE_H - 0.01, H) for i in range(6)])
    r_pocket = TH_MINOR_R + TH_CLR; z_top = TH_COLLAR + 1
    chamfer = trimesh.creation.revolve([[0, z_top - 0.5], [r_pocket, z_top - 0.5], [0, z_top - 0.5 + r_pocket]], sections=64)  # 45-deg cone: no flat ceiling over the pocket
    return _diff(body, face_slots(BASE_H - SLOT_D, H + 1) + [female_cut(0), chamfer])


def build_top_ring():
    body = _union([ring(0, TOP_H), spider(0, 6), male_thread(TOP_H - 0.01)])   # spider prints from the bed
    return _diff(body, face_slots(-0.01, SLOT_D) + [_cyl(2 * TH_MINOR_R - 6.0, TOP_H - 1, TOP_H + TH_COLLAR + 1)])


CAP_T = TOP_H + 3.0
TUBE_RECESS = 12.0


def build_cap():
    """A solid hex block (slicer infill carries the cup floor, no bridge) with
    panel slots and a tube recess underneath, and the reservoir cup on top."""
    body = _union([_ex(_hex_poly(AF_OUT, math.pi / 6), 0, CAP_T), reservoir_cup(CAP_T)])
    return _diff(body, face_slots(-0.01, SLOT_D) + [_cyl(TUBE_OD + TUBE_CLEAR, -1, TUBE_RECESS), _cyl(WICK_HOLE_D, TUBE_RECESS - 1, CAP_T + 1)])


PLATE_AF   = AF_OUT - 2 * WALL - 0.4       # spike plate: slip fit inside the stake ring's hex
PLATE_T    = 6.0
LIP_AF     = PLATE_AF - 6.0                # ring narrows to this over the top 3 mm at 45 degrees; stops the plate
PLENUM_R, PLENUM_DEPTH = 16.0, 13.0        # double cone: widest here, closes at 2*depth+... below


def hollow_fins():
    """Fins below z=0 (the spike plate's underside)."""
    solids, voids, drips = [], [], []
    for i in range(FIN_N):
        outer = Polygon([(FIN_R_IN, 0), (FIN_R_OUT, 0), (FIN_R_IN + 5, -FIN_L), (FIN_R_IN, -FIN_L)])
        inner = outer.buffer(-FIN_WALL, join_style=2)
        for poly, t, lst in ((outer, FIN_T, solids), (inner, FIN_T - 2 * FIN_WALL, voids)):
            f = extrude_polygon(poly, height=t); f.apply_transform(rotation_matrix(math.pi / 2, [1, 0, 0]))
            f.apply_translation([0, t / 2, 0]); lst.append(_rotz(f, i * 2 * math.pi / FIN_N + math.pi / 2))   # on the plate's corners
        for zz in np.arange(-15, -FIN_L + 12, -12):
            r_here = FIN_R_IN + 4 + (FIN_R_OUT - FIN_R_IN - 5) * (1 + zz / FIN_L) / 2
            d = box(extents=[DRIP_D, FIN_T + 2, DRIP_D]); d.apply_translation([r_here, 0, zz])
            drips.append(_rotz(d, i * 2 * math.pi / FIN_N + math.pi / 2))
    return solids, voids, drips


def build_stake_ring():
    """Ring + male collar, like a top ring without slots. The interior narrows
    at 45 degrees (stepped loft, self-supporting when printed ring-down) from
    the ring's hex to LIP_AF near the top, so the spike plate stops against it."""
    inner_af = AF_OUT - 2 * WALL
    steps, h = 10, 3.0                      # 0.3 mm steps = one layer each at 0.3
    loft = [_ex(_hex_ring_poly(inner_af + 0.02, inner_af - (inner_af - LIP_AF) * (i + 1) / steps, math.pi / 6),
                TOP_H - h + i * h / steps, TOP_H - h + (i + 1) * h / steps) for i in range(steps)]
    body = _union([ring(0, TOP_H), male_thread(TOP_H - 0.01)] + loft)
    return _diff(body, [_ex(_hex_poly(LIP_AF, math.pi / 6), TOP_H - h - 1, TOP_H + TH_COLLAR + 2)])   # open centre = collar bore


def build_spike():
    """Plate + double-cone plenum + hollow drip fins. Prints plate-down, fins up:
    every cavity closes at 45 degrees, nothing bridges."""
    solids, voids, drips = hollow_fins()
    body = _union([_ex(_hex_poly(PLATE_AF, math.pi / 6), 0, PLATE_T)] + solids)
    plenum = trimesh.creation.revolve([[0, -2 * PLENUM_DEPTH - 3], [PLENUM_R, -PLENUM_DEPTH], [WICK_HOLE_D / 2, 0.5], [0, 0.5]], sections=64)
    recess = trimesh.creation.revolve([[0, PLATE_T - 5.3], [WICK_HOLE_D / 2, PLATE_T - 5.3], [(TUBE_OD + TUBE_CLEAR) / 2, PLATE_T - 0.5],
                                       [(TUBE_OD + TUBE_CLEAR) / 2, PLATE_T + 1], [0, PLATE_T + 1]], sections=64)
    return _diff(body, voids + drips + [plenum, recess, _cyl(WICK_HOLE_D, -1, PLATE_T + 1)])


def build_tube(length):
    """Perforated water column. Square side holes: a flat 2.7 mm bridge prints
    cleanly at 0.3 mm where a round hole's crown sags."""
    t = _diff(_cyl(TUBE_OD, 0, length), [_cyl(lib.TUBE_ID, -1, length + 1)])
    cuts = []; z = lib.DRIP_PITCH / 2
    while z < length - lib.DRIP_PITCH / 2 + 1e-6:
        for i in range(lib.DRIP_ROWS):
            d = box(extents=[TUBE_OD, lib.DRIP_D, lib.DRIP_D]); d.apply_translation([TUBE_OD / 2, 0, z])
            cuts.append(_rotz(d, i * 2 * math.pi / lib.DRIP_ROWS))
        z += lib.DRIP_PITCH
    return _diff(t, cuts)


# ---------------- sheet nest ----------------
def nest(poly_w, poly_h):
    """Grid nest of the panel on the sheet; returns placements and count."""
    p = kerf(panel()[0]); b = p.bounds; pw, ph = b[2] - b[0], b[3] - b[1]
    # panels lie with their long side along the sheet's long side
    cols = int((SHEET_W + GAP) // (ph + GAP)); rows = int((SHEET_H + GAP) // (pw + GAP))
    items = []
    from shapely.affinity import rotate
    pr = rotate(p, 90, origin=(0, 0)); pb = pr.bounds
    for r in range(rows):
        for c in range(cols):
            items.append(translate(pr, -pb[0] + c * (ph + GAP), -pb[1] + r * (pw + GAP)))
    return items, cols, rows


def write_sheet(items, path_base):
    out = ['<svg xmlns="http://www.w3.org/2000/svg" width="%.1fmm" height="%.1fmm" viewBox="0 0 %.1f %.1f">' % (SHEET_W, SHEET_H, SHEET_W, SHEET_H),
           '<rect x="0" y="0" width="%.1f" height="%.1f" fill="none" stroke="#999" stroke-width="0.2" stroke-dasharray="4 4"/>' % (SHEET_W, SHEET_H)]
    for p in items:
        for ring_ in [p.exterior] + list(p.interiors):
            out.append('<path d="%s" fill="none" stroke="#ff0000" stroke-width="0.1"/>' %
                       ('M ' + ' L '.join('%.3f %.3f' % (x, SHEET_H - y) for x, y in list(ring_.coords)[:-1]) + ' Z'))
    out.append('</svg>'); open(path_base + '.svg', 'w').write('\n'.join(out))
    import ezdxf
    doc = ezdxf.new('R2010'); doc.units = ezdxf.units.MM; msp = doc.modelspace(); doc.layers.add('CUT', color=1)
    for p in items:
        for ring_ in [p.exterior] + list(p.interiors):
            msp.add_lwpolyline(list(ring_.coords)[:-1], close=True, dxfattribs={'layer': 'CUT'})
    doc.saveas(path_base + '.dxf')


def write_panel(poly, W, H, path_base):
    out = ['<svg xmlns="http://www.w3.org/2000/svg" width="%.3fmm" height="%.3fmm" viewBox="0 0 %.3f %.3f">' % (W, H, W, H)]
    for ring_ in [poly.exterior] + list(poly.interiors):
        out.append('<path d="%s" fill="none" stroke="#ff0000" stroke-width="0.1"/>' %
                   ('M ' + ' L '.join('%.3f %.3f' % (x, H - y) for x, y in list(ring_.coords)[:-1]) + ' Z'))
    out.append('</svg>'); open(path_base + '.svg', 'w').write('\n'.join(out))
    import ezdxf
    doc = ezdxf.new('R2010'); doc.units = ezdxf.units.MM; msp = doc.modelspace()
    for ring_ in [poly.exterior] + list(poly.interiors):
        msp.add_lwpolyline(list(ring_.coords)[:-1], close=True, dxfattribs={'layer': 'CUT'})
    doc.saveas(path_base + '.dxf')


if __name__ == '__main__':
    os.makedirs(OUT_STL, exist_ok=True); os.makedirs(OUT_LASER, exist_ok=True)
    tag = 'h%d' % int(MODULE_H)
    print('print')
    clean_export(build_cage(), os.path.join(OUT_STL, 'cage_%s.stl' % tag))
    clean_export(build_top_ring(), os.path.join(OUT_STL, 'top_ring.stl'))
    clean_export(build_cap(), os.path.join(OUT_STL, 'cap.stl'))
    clean_export(build_stake_ring(), os.path.join(OUT_STL, 'stake_ring.stl'))
    clean_export(build_spike(), os.path.join(OUT_STL, 'spike.stl'))
    clean_export(build_tube(MODULE_H + TOP_H), os.path.join(OUT_STL, 'tube_%s.stl' % tag))   # spider to spider
    clean_export(bottle_lid(), os.path.join(OUT_STL, 'lid.stl'))
    print('laser')
    p, W, H = panel()
    pk = kerf(p); b = pk.bounds
    write_panel(translate(pk, -b[0], -b[1]), b[2] - b[0], b[3] - b[1], os.path.join(OUT_LASER, 'panel_%s' % tag))
    print('  panel_%s.dxf/.svg   %.1f x %.1f mm (kerf-compensated), %d openings' % (tag, b[2] - b[0], b[3] - b[1], len(p.interiors)))
    items, cols, rows = nest(W, H)
    write_sheet(items, os.path.join(OUT_LASER, 'sheet_12x24_%s' % tag))
    print('  sheet_12x24_%s.svg/.dxf   %d x %d = %d panels = %.1f modules per sheet' % (tag, cols, rows, cols * rows, cols * rows / 6))
