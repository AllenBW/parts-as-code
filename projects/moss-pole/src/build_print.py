#!/usr/bin/env python3
"""
Moss pole, print-only, four parts.

  body_open    honeycomb tube, moss goes inside. Male neck on top, female
               socket in the bottom. Modules screw together.
  body_water   same tube, hexes recessed not cut: it holds ~0.7 L. A 45-degree
               funnel floor drains to a wick hole that drips onto the moss
               module below (no pinhole to clog: the wick sets the rate).
  spike        four hollow-hubbed fins, male thread up, screws into the first body.
  lid          flat cap with a fill hole, screws onto the top neck.

Stack:   spike -> body_open x N -> body_water -> lid.  Module pitch = BODY_H - CONE_H (rim seats on rim).

Printing (0.5 nozzle, 0.3 mm layers, no supports):
  bodies   socket down, neck up. Every shoulder is a 45-degree cone inside
           and out; the socket ledge is chamfered; hex openings are flat-top
           so their tops are short bridges.
  spike    tip UP (collar on the bed). Collar floor is a 45-degree funnel
           into the hub channel; channel closes at 45 degrees.
  lid      plate down, threaded skirt up.

Body height BODY_H: 165 fits the Mini 2 (180 Z), 230 fits the TAZ 2 (250 Z).
"""
import math
import os
import numpy as np
import trimesh
from trimesh.creation import box, revolve, extrude_polygon
from trimesh.transformations import rotation_matrix
from shapely.geometry import Polygon

from lib import _cyl, _hex_poly, _union, _diff, _rotz, clean_export
from build_hybrid import helical_thread, TH_PITCH, TH_TURNS, TH_MINOR_R, TH_DEPTH, TH_CREST, TH_CLR, TH_COLLAR

OUT_STL = os.path.join(os.path.dirname(__file__), '..', 'stl')

# ---------------- parameters ----------------
BODY_H   = float(os.environ.get('MOSS_BODY_H', 165))
OD       = 100.0
WALL     = 3.6                 # 7 perimeters at 0.5
NECK_R   = TH_MINOR_R          # 42: male core radius
NECK_BORE_R = NECK_R - WALL    # 38.4
# nested-cone joint: a flat rim, then a 45-degree cone into the neck; the next
# body's bottom has the matching recess. Rim seats on rim, cones self-centre.
RIM_W    = 3.0
CONE_H   = OD / 2 - RIM_W - NECK_R          # 5 mm
CONE_CLR = 0.3                 # recess is this much larger than the cone
SOCK_H   = CONE_H + 12.0       # recess + female socket ring
SHOULDER = OD / 2 - NECK_R     # spike only: full 8 mm cone (joint is at the soil line)
HEX_AF, STRUT = 21.0, 4.5      # 21 mm openings, 14 columns; struts 4.5 vertical, 3.5 at the bore, 5.2 outside
RECESS   = 1.2                 # watering body: hexes recessed, not cut (4 layers)
FUNNEL_T = 3.6
WICK_D   = 6.0                 # funnel hole; a 6 mm rope wick sets the drip rate
FILL_D   = 24.0                # lid fill hole
# spike
FIN_N, FIN_T, FIN_L, FIN_TIP_R = 4, 6.0, 100.0, 3.0
HUB_R, CHAN_R, CHAN_L = 9.0, 5.0, 60.0
COLLAR_BORE_R = 10.0           # 20 mm opening in the collar; funnels to the 10 mm hub channel
HOLE = 2.7                     # spike side holes, square

ID_R = OD / 2 - WALL


def male_thread(z0):
    return _union([_cyl(2 * TH_MINOR_R, z0, z0 + TH_COLLAR),
                   helical_thread(TH_MINOR_R, TH_DEPTH, TH_PITCH, TH_TURNS, z0 + 1.0)])


def female_cut(z0):
    """Socket for a male_thread whose collar base sits at z0. The groove is
    PHASED to the ridge: both start 1 mm above the collar base at angle 0,
    so 'rim seated' and 'thread home' are the same position. The groove runs
    half a turn further down as a lead-in."""
    r = TH_MINOR_R + TH_CLR
    groove = helical_thread(r, TH_DEPTH, TH_PITCH, TH_TURNS + 0.5, z0 + 1.0 - 0.5 * TH_PITCH)
    groove.apply_transform(rotation_matrix(math.pi, [0, 0, 1]))     # start half a turn EARLIER in angle too
    return _union([_cyl(2 * r, z0 - 1, z0 + TH_COLLAR + 1), groove])


def rev(pts):
    return revolve(np.array(pts, float), sections=96)


# ---------------- honeycomb ----------------
def hex_cutters(z_lo, z_hi, r_in, r_out):
    """Radial flat-top hex prisms on a uniform honeycomb around the tube."""
    d = HEX_AF + STRUT; col = d * math.sqrt(3) / 2
    n = int(round(2 * math.pi * (OD / 2 - WALL / 2) / col / 2.0)) * 2      # EVEN, or the pattern breaks at the seam
    ang = 2 * math.pi / n
    proto = extrude_polygon(_hex_poly(HEX_AF, 0.0), r_out - r_in); proto.apply_translation([0, 0, r_in])
    proto.apply_transform(rotation_matrix(-math.pi / 2, [1, 0, 0])); proto.apply_transform(rotation_matrix(-math.pi / 2, [0, 0, 1]))
    out = []
    for c in range(n):
        z = z_lo + HEX_AF / 2 + ((d / 2) if c % 2 else 0.0)
        while z <= z_hi - HEX_AF / 2 + 1e-6:
            m = proto.copy(); m.apply_translation([0, 0, z]); out.append(_rotz(m, c * ang)); z += d
    return out, n


# ---------------- bodies ----------------
def body_shell():
    """Tube with socket ring below, 45-degree shoulders and male neck above."""
    H = BODY_H; R = OD / 2
    outer = rev([[0, 0], [R, 0], [R, H - CONE_H], [R - RIM_W, H - CONE_H], [NECK_R, H], [0, H]])   # rim, then 45-deg cone into the neck
    inner = rev([[0, SOCK_H - 0.01], [ID_R, SOCK_H - 0.01], [ID_R, H - (ID_R - NECK_BORE_R)], [NECK_BORE_R, H],
                 [NECK_BORE_R, H + TH_COLLAR + 1], [0, H + TH_COLLAR + 1]])                        # bore w/ 45-deg inner shoulder
    body = _union([outer, male_thread(H - 0.01)])
    return _diff(body, [inner, female_cut(CONE_H), cone_recess(), socket_chamfer()])


def cone_recess():
    """Bottom of a body / the lid: the recess that nests on the cone below."""
    r0 = OD / 2 - RIM_W + CONE_CLR
    return rev([[0, -1], [r0 + 1.0, -1], [r0 - CONE_H, CONE_H], [0, CONE_H]])


def socket_chamfer():
    """Socket ledge -> 45-degree cone (no flat ceiling over the female pocket)."""
    r_p = TH_MINOR_R + TH_CLR; z_t = CONE_H + TH_COLLAR + 1 - 0.5
    return rev([[0, z_t], [r_p, z_t], [ID_R + 0.05, z_t + (ID_R + 0.05 - r_p)], [0, z_t + (ID_R + 0.05 - r_p)]])


def build_body_open():
    cutters, n = hex_cutters(SOCK_H + 6, BODY_H - CONE_H - 4, ID_R - 3, OD / 2 + 3)
    return _diff(body_shell(), [_union(cutters)]), n, len(cutters)


def funnel_z():
    return SOCK_H + 2 + FUNNEL_T          # apex of the funnel's TOP surface


def build_body_water():
    z_a = funnel_z(); R = ID_R + 0.05
    funnel = rev([[0, z_a - FUNNEL_T], [R, z_a + R - FUNNEL_T], [R, z_a + R], [0, z_a]])
    body = _union([body_shell(), funnel])
    cutters, n = hex_cutters(SOCK_H + 6, BODY_H - CONE_H - 4, OD / 2 - RECESS, OD / 2 + 3)   # recessed, full height
    return _diff(body, [_union(cutters), _cyl(WICK_D, z_a - FUNNEL_T - 1, z_a + 1)]), n


# ---------------- spike ----------------
def build_spike():
    """Built in USE orientation (collar up at z>=0, fins below). Print it flipped."""
    collar = male_thread(0.0)
    shoulder = rev([[0, -SHOULDER], [OD / 2, -SHOULDER], [NECK_R, 0.0], [0, 0.0]])
    hub = _cyl(2 * HUB_R, -CHAN_L - 12, -SHOULDER + 0.01)
    fins = []
    for i in range(FIN_N):
        prof = Polygon([(0, -SHOULDER + 0.01), (OD / 2, -SHOULDER + 0.01), (FIN_TIP_R, -FIN_L), (0, -FIN_L)])
        f = extrude_polygon(prof, height=FIN_T); f.apply_transform(rotation_matrix(math.pi / 2, [1, 0, 0]))
        f.apply_translation([0, FIN_T / 2, 0]); fins.append(_rotz(f, i * 2 * math.pi / FIN_N))
    body = _union([collar, shoulder, hub] + fins)
    # collar bore floors into the channel at 45 degrees; channel closes at 45 degrees
    cavity = rev([[0, -CHAN_L - CHAN_R], [CHAN_R, -CHAN_L], [CHAN_R, -(COLLAR_BORE_R - CHAN_R)],
                  [COLLAR_BORE_R, 0.0], [COLLAR_BORE_R, TH_COLLAR + 1], [0, TH_COLLAR + 1]])
    holes = []
    for zz in (-22.0, -42.0):
        for i in range(FIN_N):
            h = box(extents=[HUB_R + 4, HOLE, HOLE]); h.apply_translation([HUB_R / 2 + 2, 0, zz])
            holes.append(_rotz(h, i * 2 * math.pi / FIN_N + math.pi / FIN_N))       # between the fins
    return _diff(body, [cavity] + holes)


# ---------------- lid ----------------
def build_lid():
    """Built in USE orientation: plate on top, threaded skirt below. Print flipped (plate down)."""
    skirt = _cyl(OD, 0, SOCK_H)
    plate = _cyl(OD, SOCK_H - 0.01, SOCK_H + 3.0)
    lid = _diff(_union([skirt, plate]), [female_cut(CONE_H), cone_recess(), _cyl(FILL_D, SOCK_H - 2, SOCK_H + 4)])
    sink = rev([[0, SOCK_H + 3.01 - 2.0], [FILL_D / 2, SOCK_H + 3.01 - 2.0], [FILL_D / 2 + 2.0, SOCK_H + 3.01], [0, SOCK_H + 3.01]])
    return _diff(lid, [sink])


def reservoir_volume_ml():
    z_a = funnel_z(); R = ID_R
    return (math.pi * R * R * (BODY_H - (z_a + R)) + math.pi * R * R * R / 3) / 1000.0


if __name__ == '__main__':
    os.makedirs(OUT_STL, exist_ok=True); tag = 'h%d' % int(BODY_H)
    print('print-only set')
    m, n, k = build_body_open(); clean_export(m, os.path.join(OUT_STL, 'p_body_open_%s.stl' % tag)); print('    %d columns, %d openings' % (n, k))
    m, n = build_body_water(); clean_export(m, os.path.join(OUT_STL, 'p_body_water_%s.stl' % tag)); print('    reservoir ~%.0f ml' % reservoir_volume_ml())
    clean_export(build_spike(), os.path.join(OUT_STL, 'p_spike.stl'))
    clean_export(build_lid(), os.path.join(OUT_STL, 'p_lid.stl'))
