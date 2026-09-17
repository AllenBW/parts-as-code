#!/usr/bin/env python3
"""
Moss pole v2 -- two complete designs from one parameter set.

  PATH H  laser lattice panels in a printed skeleton
          cage (base ring + 6 slotted corner posts + spider, one print),
          top ring with a printed thread, stake with hollow drip fins,
          reservoir cup + bottle lid, perforated water tube.
          -> ../stl/v2H_*.stl, ../laser/v2H_panel_*.dxf/.svg

  PATH L  100 % sheet. Every 3D feature is a lamination, a tab, or a
          notch. Rings are 5-layer stacks (tab, tab, spider, spigot/socket
          x2). Corner splines make the hex tube rigid without glue.
          -> ../laser/v2L_*.dxf/.svg (one file per part type) plus
             ../stl/v2L_*_assembled.stl (the stacks, for rendering/checking)

Shared: hex prism ~100 mm across flats, opening size, wick-fed bottle
reservoir, module height. Uses helpers from build_moss_pole (v1).
"""

import math
import os
import numpy as np
import trimesh
from trimesh.creation import extrude_polygon, box
from trimesh.transformations import rotation_matrix
from shapely.geometry import Polygon, Point, box as sbox, LineString
from shapely.ops import unary_union
from shapely.affinity import translate, rotate

import build_moss_pole as V1
from build_moss_pole import (_ex, _cyl, _hex_poly, _hex_ring_poly, _union, _diff,
                             _rotz, clean_export, water_tube, bottle_lid,
                             reservoir_cup, export_panel_dxf_svg)

OUT_STL, OUT_LASER = V1.OUT_STL, V1.OUT_LASER

# ---------------- shared ----------------
MODULE_H   = V1.MODULE_H
AF_OUT     = 100.0
SHEET_T    = 3.0            # laser sheet
KERF       = 0.20           # cut width; outlines grow by KERF/2 in the DXF
SLOT_W     = SHEET_T + 0.4
HEX_AF     = 13.0           # panel openings (three columns fit a 50 mm face)
STRUT      = 2.5
MARGIN     = 2.5
BAND       = 10.0

# ---------------- Path H ----------------
H_WALL     = 13.0           # ring radial wall (across flats); inner corner sits inside the peg circle
H_BASE_H   = 18.0           # cage base ring: thread in the bottom 8, slots in the top 6
H_TOP_H    = 10.0           # top ring below the thread collar
H_SLOT_D   = 6.0
H_SLOT_OFF = 4.5            # slot centreline inset from the outer face
H_POST_ANG = 5.0            # half-angle of a corner post, degrees
H_POST_AF  = 76.0           # posts reach this far in (inner hex across flats); pegs sit inside the panel corner
H_PEG_D    = 3.5
H_PEG_R    = 46.5           # peg centre radius on the corner bisector, INSIDE the panels' inner-face corner (~50.8)
H_PEG_H    = 6.0
# thread (male on the top ring / stake, female in the cage base / cap)
TH_PITCH   = 3.5
TH_TURNS   = 1.5
TH_MINOR_R = 42.0           # male root radius
TH_DEPTH   = 1.6
TH_CLR     = 0.35           # radial, female over male
TH_COLLAR  = TH_PITCH * TH_TURNS + 2.5
# stake
FIN_L, FIN_T, FIN_N = 100.0, 6.0, 3
FIN_R_IN, FIN_R_OUT = 12.0, 42.0
FIN_WALL   = 1.6
DRIP_D     = 2.0
PLENUM_D   = 24.0

# ---------------- Path L ----------------
L_RING_WALL = 9.0
L_TAB_W, L_TAB_L, L_TABS = 12.0, 2 * SHEET_T, 2   # tabs per panel edge, through 2 layers
L_SPIG_AF   = 88.0          # spigot layer outer AF; socket layer inner = +0.4
L_SPLINE_W  = 12.0          # radial width, sits inside the corner
L_NOTCH_N   = 3             # egg-crate notches per corner
L_NOTCH_W   = SHEET_T / math.sin(math.radians(60)) + 0.3   # 60 deg dihedral
L_PNOTCH_D  = 6.0           # panel notch depth (from its vertical edge)
L_SNOTCH_D  = L_PNOTCH_D * math.cos(math.radians(60))       # projected onto the bisector
L_CUP_AF    = 60.0
L_CUP_H     = 27.0

SLOT_AF = AF_OUT - 2 * H_SLOT_OFF


# ---------------- geometry helpers ----------------
def panel_width():
    side = SLOT_AF / math.sqrt(3.0)
    return side - 2 * (SHEET_T / 2) * math.tan(math.pi / 6) - 1.0


def face_slots(z0, z1, extra_len=1.0):
    """Six slot boxes on the hex faces (faces at 0,60,..)."""
    slot_len = panel_width() + extra_len
    cuts = []
    for i in range(6):
        s = box(extents=[SLOT_W, slot_len, (z1 - z0) + 0.02])
        s.apply_translation([SLOT_AF / 2, 0, (z0 + z1) / 2])
        cuts.append(_rotz(s, i * math.pi / 3))
    return cuts


def spider(z0, z1, reach):
    return V1.spider(z0, z1, reach)


def helical_thread(r_root, depth, pitch, turns, z0, segs=96):
    """Watertight trapezoid ridge swept on a helix, starting inside r_root so a
    boolean union/difference with the core merges cleanly."""
    w_root, w_crest, eps = 0.55 * pitch, 0.22 * pitch, 0.6
    prof = np.array([[r_root - eps, -w_root / 2], [r_root + depth, -w_crest / 2],
                     [r_root + depth, w_crest / 2], [r_root - eps, w_root / 2]])
    n = int(segs * turns) + 1
    V, F = [], []
    for i in range(n):
        th = 2 * math.pi * i / segs
        z = z0 + pitch * th / (2 * math.pi)
        for r, dz in prof:
            V.append([r * math.cos(th), r * math.sin(th), z + dz])
    V = np.array(V)
    k = len(prof)
    for i in range(n - 1):
        for j in range(k):
            a, b = i * k + j, i * k + (j + 1) % k
            c, d = (i + 1) * k + j, (i + 1) * k + (j + 1) % k
            F += [[a, c, b], [b, c, d]]
    F += [[0, 1, 2], [0, 2, 3]]                              # start cap
    e = (n - 1) * k
    F += [[e, e + 2, e + 1], [e, e + 3, e + 2]]              # end cap
    m = trimesh.Trimesh(V, np.array(F), process=True)
    m.fix_normals()
    assert m.is_watertight, 'thread sweep not watertight'
    return m


def male_thread(z0):
    """Threaded collar from z0 up: core + ridge."""
    core = _cyl(2 * TH_MINOR_R, z0, z0 + TH_COLLAR)
    ridge = helical_thread(TH_MINOR_R, TH_DEPTH, TH_PITCH, TH_TURNS, z0 + 1.0)
    return _union([core, ridge])


def female_thread_cut(z0):
    """Solid to subtract: male shape grown by TH_CLR."""
    r = TH_MINOR_R + TH_CLR
    core = _cyl(2 * r, z0 - 1, z0 + TH_COLLAR + 1)
    ridge = helical_thread(r, TH_DEPTH, TH_PITCH, TH_TURNS + 0.5, z0 - 0.75)
    return _union([core, ridge])


# ---------------- panel (both paths) ----------------
def lattice_holes(W, H, y_lo, y_hi):
    d = HEX_AF + STRUT
    row_pitch = d * math.sqrt(3) / 2
    R = HEX_AF / math.sqrt(3)
    nrows = int((y_hi - y_lo - 2 * R) // row_pitch) + 1
    y0 = (y_lo + y_hi) / 2 - (nrows - 1) * row_pitch / 2
    holes = []
    for r in range(nrows):
        y = y0 + r * row_pitch
        off = (d / 2) if r % 2 else 0.0
        for x in np.arange(-10, 11) * d + off + W / 2:
            if x - HEX_AF / 2 < MARGIN or x + HEX_AF / 2 > W - MARGIN:
                continue
            if y - R < y_lo or y + R > y_hi:
                continue
            holes.append(translate(_hex_poly(HEX_AF, math.pi / 6), x, y))
    return unary_union(holes)


def panel_H_height():
    # base-ring slot bottom .. post top, plus the top ring's slot depth
    return MODULE_H - (H_BASE_H - H_SLOT_D) + H_SLOT_D


def panel_H():
    W, H = panel_width(), panel_H_height()
    return sbox(0, 0, W, H).difference(lattice_holes(W, H, BAND, H - BAND)), W, H


def panel_L():
    """Path L panel: tabs top and bottom, egg-crate notches on the vertical edges."""
    W, H = panel_width(), MODULE_H
    body = sbox(0, 0, W, H)
    tabs = []
    for x in np.linspace(W / 4, 3 * W / 4, L_TABS):
        tabs.append(sbox(x - L_TAB_W / 2, H, x + L_TAB_W / 2, H + L_TAB_L))
        tabs.append(sbox(x - L_TAB_W / 2, -L_TAB_L, x + L_TAB_W / 2, 0))
    body = unary_union([body] + tabs)
    notches = []
    for y in np.linspace(BAND + 8, H - BAND - 8, L_NOTCH_N):
        notches.append(sbox(-1, y - L_NOTCH_W / 2, L_PNOTCH_D, y + L_NOTCH_W / 2))
        notches.append(sbox(W - L_PNOTCH_D, y - L_NOTCH_W / 2, W + 1, y + L_NOTCH_W / 2))
    holes = lattice_holes(W, H, BAND, H - BAND)
    # keep the lattice clear of the notches: solid material around each one
    keep = unary_union([n.buffer(2.5, join_style=2) for n in notches])
    holes = unary_union([h for h in getattr(holes, 'geoms', [holes]) if not h.intersects(keep)])
    return body.difference(unary_union(notches)).difference(holes), W, H + 2 * L_TAB_L


def kerf(poly):
    """Outer outline grows, holes shrink, by KERF/2 -- so the cut part is nominal."""
    return poly.buffer(KERF / 2, join_style=2)


def export_2d(poly, name, note=''):
    b = poly.bounds
    p = translate(poly, -b[0], -b[1])
    export_panel_dxf_svg(p, b[2] - b[0], b[3] - b[1], os.path.join(OUT_LASER, name))


# ================= PATH H =================
def post_profile(i):
    """Corner post cross-section: the ring wall near corner i, as a wedge."""
    ring = _hex_ring_poly(AF_OUT, H_POST_AF)
    a = math.radians(30 + 60 * i)
    ha = math.radians(H_POST_ANG)
    wedge = Polygon([(0, 0), (70 * math.cos(a - ha), 70 * math.sin(a - ha)),
                     (70 * math.cos(a + ha), 70 * math.sin(a + ha))])
    return ring.intersection(wedge)


def build_H_cage():
    H = MODULE_H
    base = _ex(_hex_ring_poly(AF_OUT, AF_OUT - 2 * H_WALL), 0, H_BASE_H)
    sp = spider(H_BASE_H - 6, H_BASE_H, (AF_OUT - 2 * H_WALL) / 2 + 2)
    posts = [_ex(post_profile(i), H_BASE_H - 0.01, H) for i in range(6)]
    pegs = [_cyl(H_PEG_D, H - 0.01, H + H_PEG_H,
                 x=H_PEG_R * math.cos(math.radians(30 + 120 * k)),
                 y=H_PEG_R * math.sin(math.radians(30 + 120 * k)))
            for k in range(3)]
    body = _union([base, sp] + posts + pegs)
    cuts = face_slots(H_BASE_H - H_SLOT_D, H + 1)          # slots up the posts + base top
    cuts.append(female_thread_cut(0))
    return _diff(body, cuts)


def build_H_top_ring():
    ring = _ex(_hex_ring_poly(AF_OUT, AF_OUT - 2 * H_WALL), 0, H_TOP_H)
    sp = spider(0, 6, (AF_OUT - 2 * H_WALL) / 2 + 2)
    body = _union([ring, sp, male_thread(H_TOP_H - 0.01)])
    cuts = face_slots(-0.01, H_SLOT_D)
    cuts.append(_cyl(2 * TH_MINOR_R - 2 * 3.0, H_TOP_H - 1, H_TOP_H + TH_COLLAR + 1))  # hollow collar
    for k in range(3):
        cuts.append(_cyl(H_PEG_D + 0.4, -1, H_PEG_H + 0.5,
                         x=H_PEG_R * math.cos(math.radians(30 + 120 * k)),
                         y=H_PEG_R * math.sin(math.radians(30 + 120 * k))))
    return _diff(body, cuts)


def hollow_fins(z_top):
    solids, voids, drips = [], [], []
    for i in range(FIN_N):
        outer = Polygon([(FIN_R_IN, 0), (FIN_R_OUT, 0), (FIN_R_IN + 5, -FIN_L), (FIN_R_IN, -FIN_L)])
        inner = outer.buffer(-FIN_WALL, join_style=2)
        for poly, t, lst in ((outer, FIN_T, solids), (inner, FIN_T - 2 * FIN_WALL, voids)):
            f = extrude_polygon(poly, height=t)
            f.apply_transform(rotation_matrix(math.pi / 2, [1, 0, 0]))
            f.apply_translation([0, t / 2, z_top])
            lst.append(_rotz(f, i * 2 * math.pi / FIN_N))
        for zz in np.arange(-15, -FIN_L + 12, -12):
            r_here = FIN_R_IN + (FIN_R_OUT - FIN_R_IN - 5) * (1 + zz / FIN_L) / 2 + 4
            d = box(extents=[DRIP_D, FIN_T + 2, DRIP_D])
            d.apply_translation([r_here, 0, z_top + zz])
            drips.append(_rotz(d, i * 2 * math.pi / FIN_N))
    return solids, voids, drips


def build_H_stake():
    ring = _ex(_hex_ring_poly(AF_OUT, AF_OUT - 2 * H_WALL), 0, 12.0)
    plate = _ex(_hex_poly(AF_OUT - 2 * H_WALL + 2, math.pi / 6), 0, 3.0)
    hub = _cyl(V1.HUB_OD, 3.0, 12.0)
    solids, voids, drips = hollow_fins(0.0)
    body = _union([ring, plate, hub, male_thread(12.0 - 0.01)] + solids)
    plenum = _cyl(PLENUM_D, -8.0, 0.5)
    cuts = voids + drips + [plenum,
                            _cyl(V1.TUBE_OD + V1.TUBE_CLEAR, 3.0, 13.0),          # tube locator
                            _cyl(V1.WICK_HOLE_D, -9, 4),                          # column -> plenum
                            _cyl(2 * TH_MINOR_R - 2 * 3.0, 12.0, 12.0 + TH_COLLAR + 1)]
    for i in range(6):
        cuts.append(_rotz(_cyl(6.0, -1, 4, x=30.0), i * math.pi / 3 + math.pi / 6))    # drainage
    return _diff(body, cuts)


def build_H_cap():
    ring = _ex(_hex_ring_poly(AF_OUT, AF_OUT - 2 * H_WALL), 0, H_BASE_H)
    plate = _ex(_hex_poly(AF_OUT, math.pi / 6), H_BASE_H, H_BASE_H + 3.0)
    body = _union([ring, plate, reservoir_cup(H_BASE_H + 3.0)])
    return _diff(body, [female_thread_cut(0), _cyl(V1.WICK_HOLE_D, H_BASE_H - 1, H_BASE_H + 4)])


# ================= PATH L =================
def L_tab_slots(poly):
    """Cut tab slots (2 per face) into a ring-layer polygon."""
    W = panel_width()
    cuts = []
    for i in range(6):
        for x in np.linspace(W / 4, 3 * W / 4, L_TABS) - W / 2:
            s = sbox(SLOT_AF / 2 - SLOT_W / 2, x - (L_TAB_W + 0.4) / 2,
                     SLOT_AF / 2 + SLOT_W / 2, x + (L_TAB_W + 0.4) / 2)
            cuts.append(rotate(s, 60 * i, origin=(0, 0)))
    return poly.difference(unary_union(cuts))


def L_spline_slots(poly):
    r_c = SLOT_AF / 2 / math.cos(math.pi / 6)           # corner radius at the slot hex
    cuts = []
    for i in range(6):
        s = sbox(r_c - L_SPLINE_W, -SLOT_W / 2, r_c, SLOT_W / 2)
        cuts.append(rotate(s, 30 + 60 * i, origin=(0, 0)))
    return poly.difference(unary_union(cuts))


def L_layers():
    """Return {name: polygon} for the five ring layers + base/cup parts."""
    ring = _hex_ring_poly(AF_OUT, AF_OUT - 2 * L_RING_WALL)
    lay = {}
    lay['ring_tab'] = L_spline_slots(L_tab_slots(ring))
    sp = Point(0, 0).buffer(V1.HUB_OD / 2, quad_segs=32)
    arms = unary_union([rotate(sbox(0, -2, (AF_OUT - 2 * L_RING_WALL) / 2 + 2, 2), 120 * k,
                               origin=(0, 0)) for k in range(3)])
    lay['ring_spider'] = unary_union([ring, sp, arms]).difference(
        Point(0, 0).buffer((V1.TUBE_OD + V1.TUBE_CLEAR) / 2, quad_segs=32))
    lay['ring_spigot'] = _hex_ring_poly(L_SPIG_AF, AF_OUT - 2 * L_RING_WALL)
    lay['ring_socket'] = _hex_ring_poly(AF_OUT, L_SPIG_AF + 0.4)
    # spline: strip, notches from the outer (corner) edge
    # spline stands on the bottom ring's spider layer and ends under the top
    # ring's spider layer: it spans both rings' tab layers plus the panel body
    sp_h = MODULE_H + 2 * L_TAB_L - 0.5
    spl = sbox(0, 0, L_SPLINE_W, sp_h)
    for y in np.linspace(BAND + 8, MODULE_H - BAND - 8, L_NOTCH_N) + L_TAB_L + 0.25:
        spl = spl.difference(sbox(L_SPLINE_W - L_SNOTCH_D, y - L_NOTCH_W / 2, L_SPLINE_W + 1, y + L_NOTCH_W / 2))
    lay['spline'] = spl
    # base: plate with drainage + hub layer + fin tabs slots
    plate = _hex_poly(AF_OUT, math.pi / 6)
    drains = unary_union([rotate(Point(30, 0).buffer(4, quad_segs=16), 60 * i + 30, origin=(0, 0)) for i in range(6)])
    fin_slots = unary_union([rotate(sbox(FIN_R_IN, -SLOT_W / 2, FIN_R_OUT - 4, SLOT_W / 2), 120 * k, origin=(0, 0)) for k in range(3)])
    lay['base_plate'] = plate.difference(drains).difference(fin_slots).difference(Point(0, 0).buffer(3.5, quad_segs=16))
    lay['base_hub'] = Point(0, 0).buffer(V1.HUB_OD / 2, quad_segs=32).difference(
        Point(0, 0).buffer((V1.TUBE_OD + V1.TUBE_CLEAR) / 2, quad_segs=32))
    fin = Polygon([(FIN_R_IN, 0), (FIN_R_OUT - 4, 0), (FIN_R_OUT - 4, 2 * SHEET_T), (FIN_R_IN, 2 * SHEET_T),
                   (FIN_R_IN, 0), (FIN_R_IN, -FIN_L), (FIN_R_IN + 5, -FIN_L), (FIN_R_OUT, 0)])
    lay['fin'] = unary_union([Polygon([(FIN_R_IN, 0), (FIN_R_OUT, 0), (FIN_R_IN + 5, -FIN_L), (FIN_R_IN, -FIN_L)]),
                              sbox(FIN_R_IN, 0, FIN_R_OUT - 4, 2 * SHEET_T)])
    # cup: hex box walls (butt-welded) + floor
    side = L_CUP_AF / math.sqrt(3)
    lay['cup_wall'] = sbox(0, 0, side - 2 * SHEET_T * math.tan(math.pi / 6), L_CUP_H)
    lay['cup_floor'] = _hex_poly(L_CUP_AF, math.pi / 6).difference(Point(0, 0).buffer(V1.WICK_HOLE_D / 2, quad_segs=16))
    lay['lid'] = _hex_poly(L_CUP_AF + 6, math.pi / 6).difference(Point(0, 0).buffer(V1.BOTTLE_NECK_D / 2, quad_segs=32))
    lay['lid_plug'] = _hex_poly(L_CUP_AF - 2 * SHEET_T - 1.0, math.pi / 6).difference(Point(0, 0).buffer(V1.BOTTLE_NECK_D / 2, quad_segs=32))
    return lay


def L_stack(lay, order, z0=0.0):
    """Assembled laminate for rendering/verification."""
    parts = []
    z = z0
    for name in order:
        parts.append(_ex(lay[name], z, z + SHEET_T))
        z += SHEET_T
    return _union(parts)


def build_L_assemblies(lay):
    out = {}
    out['ring_bottom'] = L_stack(lay, ['ring_socket', 'ring_socket', 'ring_spider', 'ring_tab', 'ring_tab'])
    out['ring_top'] = L_stack(lay, ['ring_tab', 'ring_tab', 'ring_spider', 'ring_spigot', 'ring_spigot'])
    # base: plate + hub + fins (fins stand in plate slots, tab up 2 layers)
    base = [_ex(lay['base_plate'], 0, SHEET_T), _ex(lay['base_plate'], SHEET_T, 2 * SHEET_T),
            _ex(lay['base_hub'], 2 * SHEET_T, 2 * SHEET_T + 8)]
    for k in range(3):
        f = extrude_polygon(lay['fin'], height=SHEET_T)
        f.apply_transform(rotation_matrix(math.pi / 2, [1, 0, 0]))
        f.apply_translation([0, SHEET_T / 2, 0])
        base.append(_rotz(f, k * 2 * math.pi / 3))
    top = L_stack(lay, ['ring_spigot', 'ring_spigot'], 2 * SHEET_T)
    out['base'] = _union(base + [top])
    # cup
    side = L_CUP_AF / math.sqrt(3)
    cup = [_ex(lay['cup_floor'], 0, SHEET_T)]
    for i in range(6):
        w = extrude_polygon(lay['cup_wall'], height=SHEET_T)
        w.apply_translation([-lay['cup_wall'].bounds[2] / 2, 0, 0])
        w.apply_transform(rotation_matrix(math.pi / 2, [1, 0, 0]))
        w.apply_translation([0, -(L_CUP_AF / 2 - SHEET_T / 2), SHEET_T])
        cup.append(_rotz(w, i * math.pi / 3 + math.pi / 2))
    cap_ring = L_stack(lay, ['ring_socket', 'ring_socket', 'ring_spider'])
    cap_plate = _ex(_hex_poly(AF_OUT, math.pi / 6).difference(Point(0, 0).buffer(V1.WICK_HOLE_D / 2, quad_segs=16)),
                    3 * SHEET_T, 4 * SHEET_T)
    for c in cup:
        c.apply_translation([0, 0, 4 * SHEET_T])
    out['cap'] = _union([cap_ring, cap_plate] + cup)
    out['lid'] = _union([_ex(lay['lid_plug'], 0, SHEET_T), _ex(lay['lid'], SHEET_T, 2 * SHEET_T)])
    return out


# ================= main =================
if __name__ == '__main__':
    tag = 'v2_h%d' % int(MODULE_H)
    print('shared')
    clean_export(water_tube(MODULE_H), os.path.join(OUT_STL, '%s_water_tube.stl' % tag))
    clean_export(bottle_lid(), os.path.join(OUT_STL, 'v2H_bottle_lid.stl'))

    print('PATH H')
    clean_export(build_H_cage(), os.path.join(OUT_STL, '%s_H_cage.stl' % tag))
    clean_export(build_H_top_ring(), os.path.join(OUT_STL, 'v2H_top_ring.stl'))
    clean_export(build_H_stake(), os.path.join(OUT_STL, 'v2H_stake.stl'))
    clean_export(build_H_cap(), os.path.join(OUT_STL, 'v2H_cap_reservoir.stl'))
    p, W, H = panel_H()
    export_2d(kerf(p), '%s_H_panel' % tag)

    print('PATH L')
    lay = L_layers()
    p, W, H = panel_L()
    export_2d(kerf(p), '%s_L_panel_tabbed' % tag)
    for name, poly in lay.items():
        export_2d(kerf(poly), 'v2L_%s' % name if 'spline' not in name else '%s_L_spline' % tag)
    for name, m in build_L_assemblies(lay).items():
        clean_export(m, os.path.join(OUT_STL, 'v2L_%s_assembled.stl' % name))
