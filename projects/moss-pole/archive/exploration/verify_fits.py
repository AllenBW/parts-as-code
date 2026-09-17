#!/usr/bin/env python3
"""
Joint registry for moss-pole v2. Every place two parts interlock is listed
once, with the feature on each side MEASURED from the built geometry (STL
sections / cut polygons), the resulting clearance, and the range it must
fall in. A part's purpose is the set of joints it appears in.

Usage: python verify_fits.py [--taz]     (exit 1 on any failure)
"""
import os, sys, math
import numpy as np
import trimesh
from shapely.geometry import Point, LineString, MultiPolygon, box as sbox
from shapely.affinity import rotate
sys.path.insert(0, os.path.dirname(__file__))
import build_v2 as B
import build_moss_pole as V1

STL = B.OUT_STL
TAG = 'v2_h%d' % int(B.MODULE_H)
rows, fail = [], 0
T = B.SHEET_T


def load(n):
    m = trimesh.load(os.path.join(STL, n + '.stl')); m.process(validate=True); m.merge_vertices(); return m


def inside(m, pts):
    pts = np.atleast_2d(pts)
    return np.concatenate([m.contains(pts[i:i + 400]) for i in range(0, len(pts), 400)])


def joint(name, a_part, a_feat, a_val, b_part, b_feat, b_val, lo, hi, kind='gap'):
    """kind='gap': clearance = b - a must be in [lo, hi]. kind='eq': |a-b| <= hi."""
    global fail
    if kind == 'gap':
        c = b_val - a_val; ok = lo <= c <= hi; cs = '%+.2f' % c
    else:
        c = abs(a_val - b_val); ok = c <= hi; cs = 'Δ%.2f' % c
    fail += (not ok)
    rows.append((name, '%s: %s = %.2f' % (a_part, a_feat, a_val), '%s: %s = %.2f' % (b_part, b_feat, b_val),
                 cs, '[%.2f, %.2f]' % (lo, hi), 'PASS' if ok else 'FAIL'))


def empty_run(m, p0, p1, n=400):
    """Length of the empty (out-of-material) run that contains the midpoint of segment p0-p1."""
    p0, p1 = np.array(p0, float), np.array(p1, float)
    ts = np.linspace(0, 1, n); pts = p0 + ts[:, None] * (p1 - p0)
    ins = inside(m, pts); mid = n // 2
    if ins[mid]: return 0.0
    i = mid
    while i > 0 and not ins[i - 1]: i -= 1
    j = mid
    while j < n - 1 and not ins[j + 1]: j += 1
    return np.linalg.norm(pts[j] - pts[i]) + np.linalg.norm(p1 - p0) / (n - 1)


def solid_run(m, p0, p1, n=400):
    p0, p1 = np.array(p0, float), np.array(p1, float)
    ts = np.linspace(0, 1, n); pts = p0 + ts[:, None] * (p1 - p0)
    ins = inside(m, pts); idx = np.where(ins)[0]
    if not len(idx): return 0.0
    return (idx[-1] - idx[0]) * np.linalg.norm(p1 - p0) / (n - 1)


def section_pts(m, z):
    s = m.section(plane_origin=[0, 0, z], plane_normal=[0, 0, 1]); return np.vstack(s.discrete)


def circle_dia_near(m, z, cx, cy, tol=3.4):
    p = section_pts(m, z); k = np.hypot(p[:, 0] - cx, p[:, 1] - cy) < tol
    q = p[k]; return 2 * np.hypot(q[:, 0] - q[:, 0].mean(), q[:, 1] - q[:, 1].mean()).mean(), q.mean(axis=0)


def pitch_along_z(m, x, y, z0, z1):
    """Distance between successive solid-run starts along a vertical line (thread pitch)."""
    zs = np.linspace(z0, z1, int((z1 - z0) / 0.05)); ins = inside(m, np.column_stack([np.full_like(zs, x), np.full_like(zs, y), zs]))
    starts = zs[1:][(ins[1:] & ~ins[:-1])]
    return float(np.diff(starts).mean()) if len(starts) > 1 else float('nan')


def poly_gap_along(poly, p0, p1):
    """Length of the empty run of a 2D polygon along a segment, containing its midpoint."""
    line = LineString([p0, p1]); inside_line = line.intersection(poly)
    mid = line.interpolate(0.5, normalized=True)
    outside = line.difference(poly)
    for g in getattr(outside, 'geoms', [outside]):
        if g.distance(mid) < 1e-6: return g.length
    return 0.0


# ======================= PATH H =======================
cage, top, stake, cap = load(TAG + '_H_cage'), load('v2H_top_ring'), load('v2H_stake'), load('v2H_cap_reservoir')
tube, lid = load(TAG + '_water_tube'), load('v2H_bottle_lid')
pH, pW, pHt = B.panel_H()
face, zmid = B.SLOT_AF / 2, B.MODULE_H / 2
yin = pW / 2 - 1.0

# H1  panel edge <-> cage post slot
slot_w = empty_run(cage, [face - 4, yin, zmid], [face + 4, yin, zmid])
joint('H1 panel thickness in post slot', 'H panel', 'thickness', T, 'cage', 'post slot width', slot_w, 0.2, 0.6)
slot_len = empty_run(cage, [face, -35, zmid], [face, 35, zmid])
joint('H1 panel width in post slots', 'H panel', 'width', pW, 'cage', 'slot-to-slot length', slot_len, 0.5, 1.6)
# H2  panel bottom <-> base ring slot ; H3 panel top <-> top ring slot ; height
base_slot_d = empty_run(cage, [face, 0, B.H_BASE_H - 10], [face, 0, B.H_BASE_H + 10]) - 10.0   # run extends above the ring into air
joint('H2 base-ring slot depth', 'cage', 'slot depth', base_slot_d, 'design', 'H_SLOT_D', B.H_SLOT_D, 0, 0.2, 'eq')
top_slot_d = empty_run(top, [face, 0, -10], [face, 0, B.H_SLOT_D + 4]) - 10.0
joint('H3 top-ring slot depth', 'top ring', 'slot depth', top_slot_d, 'design', 'H_SLOT_D', B.H_SLOT_D, 0, 0.2, 'eq')
avail = (B.MODULE_H - (B.H_BASE_H - base_slot_d)) + top_slot_d
joint('H2/H3 panel height in the cage', 'H panel', 'height', pHt, 'cage+top ring', 'slot bottom to slot top', avail, -0.01, 0.5)
# H4  pegs <-> holes
r_peg = (B.SLOT_AF / 2 + 2) / math.cos(math.pi / 6)
cx, cy = r_peg * math.cos(math.radians(30)), r_peg * math.sin(math.radians(30))
def radial_feature(m, z, r0, r1, ang_deg, want_solid, target=None):
    """Centre and length of the contiguous solid/empty run along a radial line that contains `target` r."""
    a = math.radians(ang_deg); rs = np.linspace(r0, r1, 800)
    ins = inside(m, np.column_stack([rs * math.cos(a), rs * math.sin(a), np.full_like(rs, z)]))
    t = (r0 + r1) / 2 if target is None else target
    i = int(np.argmin(np.abs(rs - t)))
    if ins[i] != want_solid: return float('nan'), 0.0
    lo = i
    while lo > 0 and ins[lo - 1] == want_solid: lo -= 1
    hi = i
    while hi < len(rs) - 1 and ins[hi + 1] == want_solid: hi += 1
    return (rs[lo] + rs[hi]) / 2, rs[hi] - rs[lo] + (r1 - r0) / 799
pr, pd = radial_feature(cage, B.MODULE_H + B.H_PEG_H / 2, B.H_PEG_R - 4, B.H_PEG_R + 4, 30, True)
hr, hd = radial_feature(top, B.H_PEG_H / 2, B.H_PEG_R - 4, B.H_PEG_R + 4, 30, False)
corner_in = (B.SLOT_AF / 2 - T / 2) / math.cos(math.pi / 6)
joint('H4 peg clears the panels\' inner corner', 'cage', 'peg outer r', pr + pd / 2, 'panels', 'inner-face corner r', corner_in, 0.5, 10)
ring_in = (B.AF_OUT - 2 * B.H_WALL) / 2 / math.cos(math.pi / 6)
joint('H4 wall inside peg hole (ring inner corner)', 'design', '>= 1.5 mm', 1.5, 'top ring', 'hole inner r - ring corner r', (hr - hd / 2) - ring_in, 0, 10)
joint('H4 peg in peg hole (dia, radial)', 'cage', 'peg dia', pd, 'top ring', 'hole dia', hd, 0.2, 0.6)
joint('H4 peg/hole position', 'cage', 'peg centre r', pr, 'top ring', 'hole centre r', hr, 0, 0.1, 'eq')
# H5-H7 threads
def crest(m, z):
    # thread zone only, and only near hex FACE centres: the female thread is
    # absent at the hex corners where the ring's inner corner lies beyond it
    p = section_pts(m, z); r = np.hypot(p[:, 0], p[:, 1]); ang = np.degrees(np.arctan2(p[:, 1], p[:, 0])) % 60
    return r[(r > 41) & (r < 47) & ((ang < 8) | (ang > 52))]
zm_top = B.H_TOP_H + 1.0 + B.TH_PITCH * 0.75
r_top = crest(top, zm_top); r_cage = crest(cage, B.TH_PITCH * 0.75 + 0.25); r_stake = crest(stake, 12 + 1 + B.TH_PITCH * 0.75); r_cap = crest(cap, B.TH_PITCH * 0.75 + 0.25)
joint('H5 top-ring thread in cage thread (crest/root)', 'top ring', 'male crest r', r_top.max(), 'cage', 'female root r', r_cage.max(), B.TH_CLR - 0.1, B.TH_CLR + 0.1)
joint('H5 thread engagement', 'top ring', 'male root r', r_top.min(), 'cage', 'female crest r', r_cage.min(), 0.3, B.TH_DEPTH)
a60 = (math.cos(math.radians(60)), math.sin(math.radians(60)))    # a hex face centre
rm, rf = B.TH_MINOR_R + 0.8, B.TH_MINOR_R + B.TH_CLR + 0.8
joint('H5 thread pitch match', 'top ring', 'pitch', pitch_along_z(top, rm * a60[0], rm * a60[1], B.H_TOP_H + 0.5, B.H_TOP_H + B.TH_COLLAR + 1),
      'cage', 'pitch', pitch_along_z(cage, rf * a60[0], rf * a60[1], -1, B.TH_COLLAR + 1), 0, 0.05, 'eq')
joint('H6 stake thread in cage thread', 'stake', 'male crest r', r_stake.max(), 'cage', 'female root r', r_cage.max(), B.TH_CLR - 0.1, B.TH_CLR + 0.1)
joint('H7 top-ring thread in cap thread', 'top ring', 'male crest r', r_top.max(), 'cap', 'female root r', r_cap.max(), B.TH_CLR - 0.1, B.TH_CLR + 0.1)
# H8 water tube <-> bores
tube_od = 2 * np.hypot(*section_pts(tube, 40).T).max()
joint('H8 tube in cage spider bore', 'tube', 'OD', tube_od, 'cage', 'bore', empty_run(cage, [-12, 0, B.H_BASE_H - 3], [12, 0, B.H_BASE_H - 3]), 0.3, 0.9)
joint('H8 tube in top-ring spider bore', 'tube', 'OD', tube_od, 'top ring', 'bore', empty_run(top, [-12, 0, 3], [12, 0, 3]), 0.3, 0.9)
joint('H8 tube in stake hub', 'tube', 'OD', tube_od, 'stake', 'hub bore', empty_run(stake, [-12, 0, 8], [12, 0, 8]), 0.3, 0.9)
joint('H8 tube does NOT pass cap plate (wick only)', 'cap', 'wick hole', empty_run(cap, [-12, 0, B.H_BASE_H + 1.5], [12, 0, B.H_BASE_H + 1.5]), 'tube', 'OD', tube_od, 5, 12)
joint('H8 tube length = module pitch', 'tube', 'length', tube.extents[2], 'cage', 'height', cage.extents[2] - B.H_PEG_H, 0, 0.01, 'eq')
# H9 lid <-> cup, bottle
cup_z = B.H_BASE_H + 3 + 10
joint('H9 lid plug in cup bore', 'lid', 'plug OD', 2 * np.hypot(*section_pts(lid, V1.LID_PLUG_H / 2).T).max(),
      'cap', 'cup bore', empty_run(cap, [-35, 0, cup_z], [35, 0, cup_z]), 0.6, 1.4)
joint('H9 bottle neck in lid hole', 'PCO1881 bottle', 'thread OD', 27.4, 'lid', 'hole', empty_run(lid, [-20, 0, V1.LID_PLUG_H / 2], [20, 0, V1.LID_PLUG_H / 2]), 0.8, 1.6)
# H10 stake water path
joint('H10 column hole into plenum', 'stake', 'hole dia at plate', empty_run(stake, [-8, 0, 1.5], [8, 0, 1.5]), 'design', 'WICK_HOLE_D', V1.WICK_HOLE_D, 0, 0.2, 'eq')
fin_r = B.FIN_R_IN + 8
joint('H10 plenum reaches fin void', 'stake', 'void width at fin', empty_run(stake, [fin_r, -6, -3], [fin_r, 6, -3]), 'design', 'FIN_T - 2 wall', B.FIN_T - 2 * B.FIN_WALL, 0, 0.2, 'eq')
dr = B.FIN_R_IN + 4 + (B.FIN_R_OUT - B.FIN_R_IN - 5) * (1 - 15 / B.FIN_L) / 2
joint('H10 drip hole through both fin walls', 'stake', 'solid across fin at drip z', solid_run(stake, [dr, -6, -15], [dr, 6, -15]), 'design', 'zero (through)', 0.0, 0, 0.1, 'eq')
joint('H10 fin wall solid beside drip hole', 'stake', 'solid across fin 4 mm away', solid_run(stake, [dr, -6, -19], [dr, 6, -19]), 'design', 'FIN_T', B.FIN_T, 0, 0.3, 'eq')

# ======================= PATH L =======================
lay = B.L_layers(); pL, pLW, pLH = B.panel_L()
# L1 tab <-> tab slot
tabs = [c for c in pL.exterior.coords if c[1] > B.MODULE_H + 0.5]
tab_w = max(c[0] for c in tabs[:len(tabs)]) - min(c[0] for c in tabs[:len(tabs)])   # spans both tabs; measure one:
xs = sorted(set(round(c[0], 3) for c in tabs)); tab_w = xs[1] - xs[0]
slot = sorted(lay['ring_tab'].interiors, key=lambda i: abs(np.array(i.coords)[:, 0].mean() - face) + abs(np.array(i.coords)[:, 1].mean() - (pLW / 4 - pLW / 2)))[0]
sc = np.array(slot.coords)
joint('L1 tab width in tab slot', 'L panel', 'tab width', tab_w, 'ring_tab', 'slot length', np.ptp(sc[:, 1]), 0.2, 0.6)
joint('L1 tab thickness in tab slot', 'L panel', 'thickness', T, 'ring_tab', 'slot width', np.ptp(sc[:, 0]), 0.2, 0.6)
joint('L1 tab length = 2 tab layers', 'L panel', 'tab length', (pLH - B.MODULE_H) / 2, 'ring stack', '2 x sheet', 2 * T, 0, 0.01, 'eq')
# L2 panel notch <-> spline, spline notch <-> panel, alignment
pn = sbox(0.01, 0.01, B.L_PNOTCH_D - 0.01, B.MODULE_H - 0.01).difference(pL)
pn = MultiPolygon([g for g in getattr(pn, 'geoms', [pn]) if g.bounds[0] < 0.05])   # open to the edge = a notch
pn_ws = sorted(np.ptp(np.array(g.exterior.coords)[:, 1]) for g in pn.geoms)
joint('L2 spline (3 mm at 60°) in panel notch', 'spline', 't/sin60', T / math.sin(math.radians(60)), 'L panel', 'notch width', pn_ws[0], 0.2, 0.5)
joint('L2 panel notch depth = spline overlap', 'L panel', 'notch depth', poly_gap_along(pL, (-0.5, pn.geoms[0].centroid.y), (12, pn.geoms[0].centroid.y)) - 0.5, 'design', 'L_PNOTCH_D', B.L_PNOTCH_D, 0, 0.05, 'eq')
sn = sbox(B.L_SPLINE_W - B.L_SNOTCH_D + 0.01, 0.01, B.L_SPLINE_W - 0.01, lay['spline'].bounds[3] - 0.01).difference(lay['spline'])
sn_g = [g for g in getattr(sn, 'geoms', [sn]) if g.area > 1]
sn_zs = sorted(g.centroid.y for g in sn_g); pn_zs = sorted(g.centroid.y for g in pn.geoms)
joint('L2 panel (3 mm at 60°) in spline notch', 'L panel', 't/sin60', T / math.sin(math.radians(60)), 'spline', 'notch width', np.ptp(np.array(sn_g[0].exterior.coords)[:, 1]), 0.2, 0.5)
joint('L2 notch count matches', 'L panel', 'notches per edge', len(pn.geoms), 'spline', 'notches', len(sn_g), 0, 0, 'eq')
spline_z0 = 3 * T                       # stands on the bottom ring's spider layer (socket, socket, spider)
panel_z0 = 5 * T                        # panel body starts above the two tab layers
for k in range(B.L_NOTCH_N):
    joint('L2 notch alignment #%d (assembled z)' % (k + 1), 'spline', 'notch z', spline_z0 + sn_zs[k], 'L panel', 'notch z', panel_z0 + pn_zs[k], 0, 0.3, 'eq')
joint('L2 spline length = spider-to-spider', 'spline', 'length', lay['spline'].bounds[3], 'ring stacks', 'gap', B.MODULE_H + 2 * T * 2 / 2 * 2 - 0.0 if False else B.MODULE_H + 2 * B.L_TAB_L, 0.3, 0.7)
# L3 spline <-> ring corner notch
r_c = B.SLOT_AF / 2 / math.cos(math.pi / 6); a = math.radians(30)
bis = lambda r: (r * math.cos(a), r * math.sin(a))
notch_out = None
for r in np.arange(46, 58, 0.05):
    if lay['ring_tab'].contains(Point(*bis(r))): notch_out = r; break
joint('L3 spline outer edge vs ring notch bottom', 'spline', 'outer r', r_c, 'ring_tab', 'notch bottom r', notch_out, -0.05, 0.3)
perp = (math.cos(a + math.pi / 2), math.sin(a + math.pi / 2)); p = bis(50)
joint('L3 spline thickness in ring notch', 'spline', 'thickness', T, 'ring_tab', 'notch width',
      poly_gap_along(lay['ring_tab'], (p[0] - 5 * perp[0], p[1] - 5 * perp[1]), (p[0] + 5 * perp[0], p[1] + 5 * perp[1])), 0.2, 0.6)
# L4 spigot <-> socket
sp_r = max(np.hypot(*np.array(lay['ring_spigot'].exterior.coords).T)); so_r = min(np.hypot(*np.array(lay['ring_socket'].interiors[0].coords).T))
joint('L4 spigot layer in socket layer (corner r)', 'ring_spigot', 'outer corner r', sp_r, 'ring_socket', 'inner corner r', so_r, 0.15, 0.35)
# L5 fin tab <-> base plate slot
fs = sorted(lay['base_plate'].interiors, key=lambda i: -np.ptp(np.array(i.coords)[:, 0]))[0]; fsc = np.array(fs.coords)
fin_tab = lay['fin'].intersection(sbox(-100, 0.01, 100, 100)).bounds
joint('L5 fin tab thickness in plate slot', 'fin', 'thickness', T, 'base_plate', 'slot width', np.ptp(fsc[:, 1]), 0.2, 0.6)
joint('L5 fin tab length in plate slot', 'fin', 'tab length', fin_tab[2] - fin_tab[0], 'base_plate', 'slot length', np.ptp(fsc[:, 0]), -0.01, 0.5)
joint('L5 fin tab height = 2 plate layers', 'fin', 'tab height', fin_tab[3] - fin_tab[1], 'base', '2 x sheet', 2 * T, 0, 0.01, 'eq')
# L6 tube <-> bores
joint('L6 tube in base hub', 'tube', 'OD', tube_od, 'base_hub', 'bore', 2 * min(np.hypot(*np.array(lay['base_hub'].interiors[0].coords).T)), 0.3, 0.9)
joint('L6 tube in spider layer', 'tube', 'OD', tube_od, 'ring_spider', 'bore', 2 * min(np.hypot(*np.array(min(lay['ring_spider'].interiors, key=lambda i: i.length).coords).T)), 0.3, 0.9)
# L7 cup
side_in = (B.L_CUP_AF - 2 * T) / math.sqrt(3) * 1.0
joint('L7 cup wall width = inner hex side', 'cup_wall', 'width', lay['cup_wall'].bounds[2], 'cup', 'side at wall plane', B.L_CUP_AF / math.sqrt(3) - 2 * T * math.tan(math.pi / 6), 0, 0.01, 'eq')
plug_af = 2 * min(np.hypot(*np.array(lay['lid_plug'].exterior.coords).T)) * math.cos(math.pi / 6)
joint('L7 lid plug in cup (across flats)', 'lid_plug', 'AF', plug_af, 'cup', 'inner AF', B.L_CUP_AF - 2 * T, 0.6, 1.4)
joint('L7 bottle neck in lid hole', 'PCO1881 bottle', 'thread OD', 27.4, 'lid', 'hole', 2 * min(np.hypot(*np.array(lay['lid'].interiors[0].coords).T)), 0.8, 1.6)

# ======================= report =======================
w = [max(len(r[i]) for r in rows) for i in range(6)]
print(' | '.join(h.ljust(w[i]) for i, h in enumerate(['joint', 'side A (measured)', 'side B (measured)', 'clearance', 'allowed', ''])))
print('-+-'.join('-' * x for x in w))
for r in rows:
    print(' | '.join(r[i].ljust(w[i]) for i in range(6)))
print('\n%d joints, %d failure(s)' % (len(rows), fail))
if '--parts' in sys.argv:
    from collections import OrderedDict
    parts = OrderedDict()
    for r in rows:
        for side in (r[1], r[2]):
            p = side.split(':')[0]
            if p in ('design', 'panels', 'ring stacks', 'ring stack', 'cage+top ring', 'cup', 'base'): continue
            parts.setdefault(p, []).append(r[0].split(' ', 1)[0])
    print('\npart -> joints it takes part in (its purpose, as tested)')
    for p, js in parts.items():
        print('  %-16s %s' % (p, ', '.join(OrderedDict.fromkeys(js))))
sys.exit(1 if fail else 0)
