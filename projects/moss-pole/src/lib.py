#!/usr/bin/env python3
"""
Modular self-watering moss pole, ~100 mm wide -- two architectures built from
one parameter set so they can be printed and compared head to head.

  A. HEX HYBRID   printed hex joiner rings, laser-cut 3 mm acrylic lattice
                  panels (6 per module) dropped into slots on the ring faces.
                  A ring serves as the top of one module and the bottom of the
                  next, so a pole is:  stake -> [6 panels, ring] x N -> cap.
                  Module height is NOT printer-bound (only the ring is printed).

  B. FULL PRINT   one-piece perforated PETG cylinder per module, honeycomb
                  openings, tapered spigot + 3 keys into the module below.
                  Module height IS printer-bound: MODULE_H <= 165 fits the
                  Mini 2 (180 Z), <= 230 fits the TAZ 2 (250 Z).

Shared parts (both variants):
  - water column: perforated tube down the pole axis, cotton rope wick inside;
    tube segments are module-length and butt end to end through spider hubs
  - reservoir cap: cup on top; an inverted soda bottle rests mouth-down in a
    lid. Water level self-holds at the bottle mouth (chicken-waterer
    principle), the wick draws from the cup. No orifice to clog.
  - stake base: finned anchor below the soil line

Outputs ../stl/*.stl for printing and ../laser/*.dxf + *.svg for the P2.
"""

import math
import os
import numpy as np
import trimesh
from trimesh.creation import cylinder, extrude_polygon, box
from trimesh.transformations import rotation_matrix
from shapely.geometry import Polygon, Point, box as sbox
from shapely.ops import unary_union
from shapely.affinity import translate

# ---------------- shared parameters ----------------
MODULE_H      = float(os.environ.get('MOSS_MODULE_H', 165))   # 165 = Mini 2 safe (B), 230 = TAZ 2 only (B); free for A
AF_OUT        = 100.0   # hex across-flats / cylinder OD (the "4 inch" target)
SEG           = 128

# water column
TUBE_OD       = 16.0
TUBE_ID       = 12.0
TUBE_CLEAR    = 0.6     # spider bore = TUBE_OD + TUBE_CLEAR
DRIP_D        = 2.5
DRIP_ROWS     = 3       # rows around the tube, 120 deg apart
DRIP_PITCH    = 25.0
HUB_OD        = 24.0
SPIDER_ARMS   = 3
ARM_W         = 4.0
SPIDER_T      = 6.0

# reservoir cap (shared)
CUP_OD        = 60.0
CUP_WALL      = 2.4
CUP_H         = 24.0    # above the cap plate
PLATE_T       = 3.0
WICK_HOLE_D   = 7.0
BOTTLE_NECK_D = 28.6    # PCO 1881 thread OD 27.4 + clearance; bead (33) rests on lid
LID_FLANGE_OD = 66.0
LID_FLANGE_T  = 2.0
LID_PLUG_H    = 4.0
LID_PLUG_CLR  = 0.5

# stake base (shared)
STAKE_RING_H  = 14.0
FIN_N         = 3
FIN_T         = 4.0
FIN_L         = 100.0   # below soil
FIN_R_OUT     = 41.0
FIN_R_IN      = 10.0
DRAIN_D       = 8.0
DRAIN_N       = 6

# ---- A: hex hybrid ----
RING_H        = 14.0
RING_WALL     = 9.0     # radial, across flats
SLOT_OFFSET   = 4.5     # slot centreline inset from the outer face
PANEL_T       = 3.0     # nominal acrylic
SLOT_W        = PANEL_T + 0.4
SLOT_D        = 6.0     # per face; RING_H - 2*SLOT_D = web between top/bottom slots
PANEL_CLR     = 1.0     # corner gap between adjacent panel edges
PANEL_HEX_AF  = 12.0
PANEL_STRUT   = 3.0
PANEL_MARGIN  = 3.5     # solid border at the vertical edges
PANEL_BAND    = 10.0    # solid band at top/bottom (slot engagement + a bit)

# ---- B: full-print cylinder ----
CYL_WALL      = 3.0
SPIGOT_H      = 8.0
SPIGOT_CLR    = 0.45    # per side (PETG: pegs print fat, bores print tight)
KEY_N         = 3
KEY_W         = 3.0
KEY_NOTCH_W   = 3.8
CYL_HEX_AF    = 14.0    # opening size (roots need >= 12)
CYL_STRUT     = 4.0
CYL_BAND      = 14.0    # solid band top and bottom of each module

OUT_STL   = os.path.join(os.path.dirname(__file__), '..', 'stl')
OUT_LASER = os.path.join(os.path.dirname(__file__), '..', 'laser')

# ---------------- helpers ----------------
def _ex(poly, z0, z1):
    m = extrude_polygon(poly, height=z1 - z0)
    m.apply_translation([0, 0, z0])
    return m


def _cyl(d, z0, z1, x=0.0, y=0.0):
    c = cylinder(radius=d / 2, height=z1 - z0, sections=SEG)
    c.apply_translation([x, y, (z0 + z1) / 2])
    return c


def _hex_poly(af, rot=0.0):
    """Regular hexagon by across-flats. rot=0 -> flat faces at +-y (pointy at +-x)."""
    r = af / math.sqrt(3.0)
    a = np.arange(6) * math.pi / 3 + rot
    return Polygon(np.column_stack([r * np.cos(a), r * np.sin(a)]))


def _hex_ring_poly(af_out, af_in, rot=math.pi / 6):
    return _hex_poly(af_out, rot).difference(_hex_poly(af_in, rot))


def _union(parts):
    return trimesh.boolean.union(parts, engine='manifold')


def _diff(base, cuts):
    return trimesh.boolean.difference([base] + cuts, engine='manifold')


def _rotz(m, ang):
    m.apply_transform(rotation_matrix(ang, [0, 0, 1]))
    return m


def clean_export(mesh, path):
    m = mesh.copy()
    m.vertices = np.round(m.vertices, 4)
    m.merge_vertices()
    m.update_faces(m.nondegenerate_faces())
    m.update_faces(m.unique_faces())
    m.remove_unreferenced_vertices()
    m.fix_normals()
    os.makedirs(os.path.dirname(path), exist_ok=True)
    m.export(path)
    print('  %-34s %7.1f cm3  %s' % (os.path.basename(path), m.volume / 1000,
                                     'watertight' if m.is_watertight else 'NOT WATERTIGHT'))
    return m


# ---------------- shared sub-assemblies ----------------
def spider(z0, z1, reach):
    """Hub (bored for the tube) + radial arms out to radius `reach`."""
    hub = _cyl(HUB_OD, z0, z1)
    arms = []
    for i in range(SPIDER_ARMS):
        a = box(extents=[reach, ARM_W, z1 - z0])
        a.apply_translation([reach / 2, 0, (z0 + z1) / 2])
        arms.append(_rotz(a, i * 2 * math.pi / SPIDER_ARMS))   # 0/120/240 = hex faces
    return _diff(_union([hub] + arms), [_cyl(TUBE_OD + TUBE_CLEAR, z0 - 1, z1 + 1)])


def water_tube(length):
    t = _diff(_cyl(TUBE_OD, 0, length), [_cyl(TUBE_ID, -1, length + 1)])
    cuts = []
    z = DRIP_PITCH / 2
    while z < length - DRIP_PITCH / 2 + 1e-6:
        for i in range(DRIP_ROWS):
            c = cylinder(radius=DRIP_D / 2, height=TUBE_OD, sections=32)
            c.apply_transform(rotation_matrix(math.pi / 2, [0, 1, 0]))
            c.apply_translation([TUBE_OD / 2, 0, z])
            cuts.append(_rotz(c, i * 2 * math.pi / DRIP_ROWS))
        z += DRIP_PITCH
    return _diff(t, cuts)


def reservoir_cup(plate_z):
    """Cup standing on a plate whose top is at plate_z. Floor is the plate."""
    wall = _diff(_cyl(CUP_OD, plate_z, plate_z + CUP_H),
                 [_cyl(CUP_OD - 2 * CUP_WALL, plate_z - 1, plate_z + CUP_H + 1)])
    return wall


def bottle_lid():
    """Sits in the cup mouth. Inverted bottle drops through the centre hole and
    hangs by its transfer bead. Two vent notches keep the cup at atmosphere."""
    cup_id = CUP_OD - 2 * CUP_WALL
    flange = _cyl(LID_FLANGE_OD, LID_PLUG_H, LID_PLUG_H + LID_FLANGE_T)
    plug = _cyl(cup_id - 2 * LID_PLUG_CLR, 0, LID_PLUG_H)
    lid = _union([flange, plug])
    cuts = [_cyl(BOTTLE_NECK_D, -1, LID_PLUG_H + LID_FLANGE_T + 1)]
    for i in range(2):
        v = box(extents=[6.0, 3.0, LID_PLUG_H + 0.1])
        v.apply_translation([cup_id / 2 - 2.0, 0, LID_PLUG_H / 2])
        cuts.append(_rotz(v, i * math.pi))
    return _diff(lid, cuts)


def fins(z_top):
    """Tapered anchor fins pointing down from z_top."""
    out = []
    for i in range(FIN_N):
        prof = Polygon([(FIN_R_IN, 0), (FIN_R_OUT, 0),
                        (FIN_R_IN + 4.0, -FIN_L), (FIN_R_IN, -FIN_L)])
        f = extrude_polygon(prof, height=FIN_T)           # in xz, extruded along local z
        f.apply_transform(rotation_matrix(math.pi / 2, [1, 0, 0]))  # local z -> -y
        f.apply_translation([0, FIN_T / 2, z_top])
        out.append(_rotz(f, i * 2 * math.pi / FIN_N))
    return _union(out)


def stake_plate_cuts(z0, z1):
    cuts = []
    for i in range(DRAIN_N):
        c = _cyl(DRAIN_D, z0 - 1, z1 + 1, x=28.0)
        cuts.append(_rotz(c, i * 2 * math.pi / DRAIN_N + math.pi / DRAIN_N))
    return cuts


# ---------------- A: hex hybrid ----------------
def hex_slots(z0, z1):
    """Six panel slots on the hex faces between z0..z1 (open toward whichever
    face is at the ring surface)."""
    slot_af = AF_OUT - 2 * SLOT_OFFSET
    side = slot_af / math.sqrt(3.0)
    panel_w = side - 2 * (PANEL_T / 2) * math.tan(math.pi / 6) - PANEL_CLR
    slot_len = panel_w + 1.0
    cuts = []
    for i in range(6):
        s = box(extents=[slot_len, SLOT_W, (z1 - z0) + 0.02])
        s.apply_translation([0, slot_af / 2, (z0 + z1) / 2])
        cuts.append(_rotz(s, i * math.pi / 3 + math.pi / 2))   # faces are at 0,60,..; +y is a corner
    return cuts, panel_w


def hex_ring_body(z0, z1):
    return _ex(_hex_ring_poly(AF_OUT, AF_OUT - 2 * RING_WALL), z0, z1)


def build_joiner_ring():
    """Slots on both faces + spider in the middle."""
    body = hex_ring_body(0, RING_H)
    sp = spider((RING_H - SPIDER_T) / 2, (RING_H + SPIDER_T) / 2,
                (AF_OUT - 2 * RING_WALL) / 2 + 2.0)
    body = _union([body, sp])
    lo, _ = hex_slots(-0.01, SLOT_D)
    hi, _ = hex_slots(RING_H - SLOT_D, RING_H + 0.01)
    return _diff(body, lo + hi)


def build_cap_hex():
    """Ring with slots on the bottom face, closed plate, reservoir cup on top."""
    ring = hex_ring_body(0, RING_H)
    plate = _ex(_hex_poly(AF_OUT, math.pi / 6), RING_H, RING_H + PLATE_T)
    cup = reservoir_cup(RING_H + PLATE_T)
    body = _union([ring, plate, cup])
    lo, _ = hex_slots(-0.01, SLOT_D)
    return _diff(body, lo + [_cyl(WICK_HOLE_D, RING_H - 1, RING_H + PLATE_T + 1)])


def build_stake_hex():
    """Slots on the top face, drained floor, tube locator hub, fins below."""
    ring = hex_ring_body(0, STAKE_RING_H)
    plate = _ex(_hex_poly(AF_OUT - 2 * RING_WALL + 2, math.pi / 6), 0, PLATE_T)
    hub = _diff(_cyl(HUB_OD, PLATE_T, PLATE_T + 8.0),
                [_cyl(TUBE_OD + TUBE_CLEAR, PLATE_T - 0.01, PLATE_T + 9)])
    body = _union([ring, plate, hub, fins(0.0)])
    hi, _ = hex_slots(STAKE_RING_H - SLOT_D, STAKE_RING_H + 0.01)
    return _diff(body, hi + stake_plate_cuts(0, PLATE_T)
                 + [_cyl(WICK_HOLE_D, -1, PLATE_T + 1)])


def panel_polygon():
    """Flat lattice panel for the laser: rectangle minus a uniform honeycomb of
    pointy-top hexes. Width from the slot geometry, height = MODULE_H."""
    _, panel_w = hex_slots(0, 1)
    W, H = panel_w, MODULE_H
    outer = sbox(0, 0, W, H)
    d = PANEL_HEX_AF + PANEL_STRUT           # centre spacing (triangular lattice)
    row_pitch = d * math.sqrt(3) / 2
    R = PANEL_HEX_AF / math.sqrt(3)          # circumradius
    holes = []
    x_lo, x_hi = PANEL_MARGIN, W - PANEL_MARGIN
    y_lo, y_hi = PANEL_BAND, H - PANEL_BAND
    # centre the pattern in the allowed window
    nrows = int((y_hi - y_lo - 2 * R) // row_pitch) + 1
    y0 = (y_lo + y_hi) / 2 - (nrows - 1) * row_pitch / 2
    for r in range(nrows):
        y = y0 + r * row_pitch
        off = (d / 2) if r % 2 else 0.0
        xs = np.arange(-10, 11) * d + off + W / 2
        for x in xs:
            if x - PANEL_HEX_AF / 2 < x_lo or x + PANEL_HEX_AF / 2 > x_hi:
                continue
            if y - R < y_lo or y + R > y_hi:
                continue
            holes.append(translate(_hex_poly(PANEL_HEX_AF, math.pi / 6), x, y))
    return outer.difference(unary_union(holes)), W, H, len(holes)


def export_panel_dxf_svg(poly, W, H, path_base):
    os.makedirs(os.path.dirname(path_base), exist_ok=True)
    rings = [poly.exterior] + list(poly.interiors)
    # DXF (ezdxf if present, else a minimal hand-written R12)
    try:
        import ezdxf
        doc = ezdxf.new('R2010')
        doc.units = ezdxf.units.MM
        msp = doc.modelspace()
        for ring in rings:
            msp.add_lwpolyline(list(ring.coords)[:-1], close=True,
                               dxfattribs={'layer': 'CUT'})
        doc.saveas(path_base + '.dxf')
    except ImportError:
        with open(path_base + '.dxf', 'w') as f:
            f.write('0\nSECTION\n2\nENTITIES\n')
            for ring in rings:
                pts = list(ring.coords)[:-1]
                f.write('0\nPOLYLINE\n8\nCUT\n66\n1\n70\n1\n')
                for x, y in pts:
                    f.write('0\nVERTEX\n8\nCUT\n10\n%.4f\n20\n%.4f\n' % (x, y))
                f.write('0\nSEQEND\n')
            f.write('0\nENDSEC\n0\nEOF\n')
    # SVG in mm (XCS reads this directly); y flipped so it reads upright
    paths = []
    for ring in rings:
        pts = list(ring.coords)
        d = 'M ' + ' L '.join('%.3f %.3f' % (x, H - y) for x, y in pts[:-1]) + ' Z'
        paths.append('<path d="%s" fill="none" stroke="#ff0000" stroke-width="0.1"/>' % d)
    with open(path_base + '.svg', 'w') as f:
        f.write('<svg xmlns="http://www.w3.org/2000/svg" width="%.3fmm" height="%.3fmm" '
                'viewBox="0 0 %.3f %.3f">\n%s\n</svg>\n' % (W, H, W, H, '\n'.join(paths)))
    print('  %-34s %.1f x %.1f mm, %d openings' % (
        os.path.basename(path_base) + '.dxf/.svg', W, H, len(poly.interiors)))


# ---------------- B: full-print cylinder ----------------
def cyl_hex_cutters(z_lo, z_hi):
    """Radial hex prisms (flat-top for printability) in a uniform honeycomb."""
    r_out = AF_OUT / 2
    d = CYL_HEX_AF + CYL_STRUT
    col_pitch = d * math.sqrt(3) / 2                      # horizontal, arc length
    n_cols = int(round(2 * math.pi * (r_out - CYL_WALL / 2) / col_pitch))
    ang = 2 * math.pi / n_cols
    R = CYL_HEX_AF / math.sqrt(3)
    proto = _ex(_hex_poly(CYL_HEX_AF, 0.0), r_out - CYL_WALL - 3, r_out + 3)
    proto.apply_transform(rotation_matrix(-math.pi / 2, [1, 0, 0]))  # local z -> +y ... radial
    proto.apply_transform(rotation_matrix(-math.pi / 2, [0, 0, 1]))  # +y -> +x
    cutters = []
    c_lo, c_hi = z_lo + CYL_HEX_AF / 2, z_hi - CYL_HEX_AF / 2
    for c in range(n_cols):
        off = (d / 2) if c % 2 else 0.0
        z = c_lo + off
        while z <= c_hi + 1e-6:
            m = proto.copy()
            m.apply_translation([0, 0, z])
            cutters.append(_rotz(m, c * ang))
            z += d
    return cutters, n_cols


def spigot_od():
    return (AF_OUT - 2 * CYL_WALL) - 2 * SPIGOT_CLR


def cyl_keys():
    """Keys on the spigot that drop into notches in the rim of the module below."""
    keys = []
    for i in range(KEY_N):
        k = box(extents=[(AF_OUT - 2.0) / 2 - spigot_od() / 2 + 1.0, KEY_W, SPIGOT_H])
        k.apply_translation([(spigot_od() / 2 + (AF_OUT - 2.0) / 2) / 2 - 0.5, 0,
                             -SPIGOT_H / 2])
        keys.append(_rotz(k, i * 2 * math.pi / KEY_N))
    return keys


def cyl_rim_notches(z_top):
    cuts = []
    for i in range(KEY_N):
        n = box(extents=[CYL_WALL + 4.0, KEY_NOTCH_W, SPIGOT_H + 0.5])
        n.apply_translation([AF_OUT / 2 - CYL_WALL / 2, 0, z_top - SPIGOT_H / 2 + 0.25])
        cuts.append(_rotz(n, i * 2 * math.pi / KEY_N))
    return cuts


def build_cyl_module():
    H = MODULE_H
    shell = _diff(_cyl(AF_OUT, 0, H), [_cyl(AF_OUT - 2 * CYL_WALL, -1, H + 1)])
    spig = _diff(_cyl(spigot_od(), -SPIGOT_H, 0.01),
                 [_cyl(spigot_od() - 2 * CYL_WALL, -SPIGOT_H - 1, 1)])
    sp = spider(0, SPIDER_T, AF_OUT / 2 - CYL_WALL + 1.0)
    body = _union([shell, spig, sp] + cyl_keys())
    cutters, n_cols = cyl_hex_cutters(CYL_BAND, H - CYL_BAND)
    body = _diff(body, [_union(cutters)] + cyl_rim_notches(H))
    return body, n_cols, len(cutters)


def build_cap_cyl():
    plate = _cyl(AF_OUT, 0, PLATE_T)
    spig = _diff(_cyl(spigot_od(), -SPIGOT_H, 0.01),
                 [_cyl(spigot_od() - 2 * CYL_WALL, -SPIGOT_H - 1, 1)])
    body = _union([plate, spig, reservoir_cup(PLATE_T)] + cyl_keys())
    return _diff(body, [_cyl(WICK_HOLE_D, -SPIGOT_H - 1, PLATE_T + 1)])


def build_stake_cyl():
    ring = _diff(_cyl(AF_OUT, 0, STAKE_RING_H),
                 [_cyl(AF_OUT - 2 * CYL_WALL, -1, STAKE_RING_H + 1)])
    plate = _cyl(AF_OUT - 2 * CYL_WALL + 2, 0, PLATE_T)
    hub = _diff(_cyl(HUB_OD, PLATE_T, PLATE_T + 8.0),
                [_cyl(TUBE_OD + TUBE_CLEAR, PLATE_T - 0.01, PLATE_T + 9)])
    body = _union([ring, plate, hub, fins(0.0)])
    return _diff(body, cyl_rim_notches(STAKE_RING_H) + stake_plate_cuts(0, PLATE_T)
                 + [_cyl(WICK_HOLE_D, -1, PLATE_T + 1)])


# ---------------- main ----------------
if __name__ == '__main__':
    tag = 'v1_h%d' % int(MODULE_H)
    print('shared')
    clean_export(water_tube(MODULE_H), os.path.join(OUT_STL, '%s_water_tube.stl' % tag))
    clean_export(bottle_lid(), os.path.join(OUT_STL, 'v1_bottle_lid.stl'))

    print('A: hex hybrid')
    clean_export(build_joiner_ring(), os.path.join(OUT_STL, 'v1_A_joiner_ring.stl'))
    clean_export(build_cap_hex(), os.path.join(OUT_STL, 'v1_A_cap_reservoir.stl'))
    clean_export(build_stake_hex(), os.path.join(OUT_STL, 'v1_A_stake_base.stl'))
    poly, W, H, n = panel_polygon()
    export_panel_dxf_svg(poly, W, H, os.path.join(OUT_LASER, '%s_A_panel_3mm' % tag))

    print('B: full-print cylinder')
    mod, n_cols, n_cut = build_cyl_module()
    print('  (%d columns x honeycomb, %d openings)' % (n_cols, n_cut))
    clean_export(mod, os.path.join(OUT_STL, '%s_B_module.stl' % tag))
    clean_export(build_cap_cyl(), os.path.join(OUT_STL, 'v1_B_cap_reservoir.stl'))
    clean_export(build_stake_cyl(), os.path.join(OUT_STL, 'v1_B_stake_base.stl'))
