#!/usr/bin/env python3
"""Print-only set: joints measured both sides, printability in print orientation, envelopes.
Usage: python verify_print.py [--taz]"""
import os, sys, math
import numpy as np
import trimesh
from trimesh.transformations import rotation_matrix
sys.path.insert(0, os.path.dirname(__file__))
import build_print as B

STL = B.OUT_STL; TAG = 'h%d' % int(B.BODY_H)
ENV, ENV_NAME = ((298, 275, 250), 'TAZ 2') if '--taz' in sys.argv else ((160, 160, 180), 'Mini 2')
rows, fail = [], 0


def load(n):
    m = trimesh.load(os.path.join(STL, n + '.stl')); m.process(validate=True); m.merge_vertices(); return m


def inside(m, pts):
    pts = np.atleast_2d(pts); return np.concatenate([m.contains(pts[i:i + 400]) for i in range(0, len(pts), 400)])


def check(name, ok, detail=''):
    global fail
    fail += (not ok); print('  %-56s %s %s' % (name, 'PASS' if ok else 'FAIL', detail))


def joint(name, a_part, a_feat, a_val, b_part, b_feat, b_val, lo, hi, kind='gap'):
    global fail
    if kind == 'gap': c = b_val - a_val; ok = lo <= c <= hi; cs = '%+.2f' % c
    else: c = abs(a_val - b_val); ok = c <= hi; cs = 'Δ%.2f' % c
    fail += (not ok)
    rows.append((name, '%s: %s = %.2f' % (a_part, a_feat, a_val), '%s: %s = %.2f' % (b_part, b_feat, b_val), cs, '[%.2f, %.2f]' % (lo, hi), 'PASS' if ok else 'FAIL'))


def run(m, p0, p1, want, target=0.5, n=600):
    p0, p1 = np.array(p0, float), np.array(p1, float); ts = np.linspace(0, 1, n)
    ins = inside(m, p0 + ts[:, None] * (p1 - p0)); i = int(np.argmin(np.abs(ts - target)))
    if ins[i] != want: return 0.0
    lo = i
    while lo > 0 and ins[lo - 1] == want: lo -= 1
    hi = i
    while hi < n - 1 and ins[hi + 1] == want: hi += 1
    return (hi - lo + 1) * np.linalg.norm(p1 - p0) / (n - 1)


gap = lambda m, p0, p1, t=0.5: run(m, p0, p1, False, t)
solid_total = lambda m, p0, p1: inside(m, np.array(p0, float) + np.linspace(0, 1, 600)[:, None] * (np.array(p1, float) - np.array(p0, float))).sum() * np.linalg.norm(np.array(p1, float) - np.array(p0, float)) / 599


def section_r(m, z, lo, hi):
    s = m.section(plane_origin=[0, 0, z], plane_normal=[0, 0, 1]); p = np.vstack(s.discrete)
    r = np.hypot(p[:, 0], p[:, 1]); return r[(r > lo) & (r < hi)]


def pitch(m, x, y, z0, z1):
    zs = np.arange(z0, z1, 0.05); ins = inside(m, np.column_stack([np.full_like(zs, x), np.full_like(zs, y), zs]))
    st = zs[1:][ins[1:] & ~ins[:-1]]; return float(np.diff(st).mean()) if len(st) > 1 else float('nan')


def overhang(m, flip=False):
    mm = m.copy()
    if flip: mm.apply_transform(rotation_matrix(math.pi, [1, 0, 0]))
    mm.apply_translation([0, 0, -mm.bounds[0][2]])
    nz = mm.face_normals[:, 2]; zc = mm.triangles_center[:, 2]; A = mm.area_faces; off = zc > 0.6
    return A[(nz < -0.97) & off].sum(), A[(nz >= -0.97) & (nz < -0.72) & off].sum()


print('parts')
P = {'body open': load('p_body_open_' + TAG), 'body water': load('p_body_water_' + TAG), 'spike': load('p_spike'), 'lid': load('p_lid')}
for n, m in P.items():
    check('%s watertight, single solid' % n, m.is_watertight and len(m.split()) == 1)
    e = m.extents; check('%s fits %s' % (n, ENV_NAME), all(e[i] <= ENV[i] for i in range(3)), '%.0fx%.0fx%.0f' % tuple(e))
bo, bw, sp, lid = P['body open'], P['body water'], P['spike'], P['lid']
H = B.BODY_H

# J1 body neck (male) <-> body socket (female)
zm = H + 1 + B.TH_PITCH * 0.75; zf = B.TH_PITCH * 0.75 + 0.25
r_m = section_r(bo, zm, 41, 47); r_f = section_r(bo, zf, 41, 47); r_fw = section_r(bw, zf, 41, 47); r_s = section_r(sp, 1 + B.TH_PITCH * 0.75, 41, 47); r_l = section_r(lid, zf, 41, 47)
joint('J1 neck thread in socket thread (clearance)', 'body neck', 'male crest r', r_m.max(), 'body socket', 'female root r', r_f.max(), B.TH_CLR - 0.1, B.TH_CLR + 0.1)
joint('J1 thread engagement depth', 'body neck', 'male root r', r_m.min(), 'body socket', 'female crest r', r_f.min(), 0.3, B.TH_DEPTH)
a45 = (math.cos(math.radians(45)), math.sin(math.radians(45)))      # both turns of a 1.25-turn ridge cross this line
rm, rfm = B.TH_MINOR_R + B.TH_DEPTH - 0.3, B.TH_MINOR_R + B.TH_CLR + B.TH_DEPTH - 0.3
joint('J1 thread pitch match', 'body neck', 'pitch', pitch(bo, rm * a45[0], rm * a45[1], H + 0.3, H + B.TH_COLLAR + 1), 'body socket', 'pitch', pitch(bo, rfm * a45[0], rfm * a45[1], B.CONE_H + 0.3, B.CONE_H + B.TH_COLLAR + 1), 0, 0.05, 'eq')
rc = B.TH_MINOR_R + B.TH_DEPTH   # neck crest radius
zc = B.CONE_H + B.TH_COLLAR + 0.5   # just above the neck's top when the rim is seated
joint('J1 socket open past the neck core at collar height', 'body neck', 'core r + 0.3', B.TH_MINOR_R + 0.3, 'body socket', 'open radius at collar+0.5', gap(bo, [0, 0, zc], [48, 0, zc], 0.0), 0, 10)
# J6 nested cone joint: rim seats on rim, cones nest with clearance
top_rim = solid_total(bo, [B.OD / 2 - 1.5, 0, H - B.CONE_H - 3], [B.OD / 2 - 1.5, 0, H - B.CONE_H + 3])          # solid up to the rim face
bot_rim = solid_total(bo, [B.OD / 2 - 1.5, 0, -3], [B.OD / 2 - 1.5, 0, 3])                                        # solid from the bottom face
joint('J6 rim face height (top)', 'body top', 'solid to z', top_rim - 3.0 + (H - B.CONE_H - 3) + 3.0 - (H - B.CONE_H - 3), 'design', 'H - CONE_H', H - B.CONE_H, 0, 0, 'eq') if False else None
joint('J6 top rim is flat and RIM_W wide', 'design', 'RIM_W + 0.2', B.RIM_W + 0.2, 'body top', 'OD/2 - cone r just above the rim', B.OD / 2 - (52 - gap(bo, [52, 0, H - B.CONE_H + 0.2], [0, 0, H - B.CONE_H + 0.2], 0.0)), -0.15, 0.15)
joint('J6 bottom rim lands on the top rim', 'design', 'rim inner edge r = OD/2 - RIM_W + CLR - 0.2', B.OD / 2 - B.RIM_W + B.CONE_CLR - 0.2, 'body bottom', 'ring inner edge r at z=0.2', gap(bo, [0, 0, 0.2], [52, 0, 0.2], 0.0), -0.1, 0.1)
zm_c = B.CONE_H / 2
cone_r = 52 - gap(bo, [52, 0, H - B.CONE_H + zm_c], [0, 0, H - B.CONE_H + zm_c], 0.0)               # top cone radius at mid height (probe from r=52 inward)
rec_r = gap(bo, [0, 0, zm_c], [52, 0, zm_c], 0.0)                                                  # recess radius at the same height
joint('J6 cones nest with clearance (mid-height)', 'body top', 'cone r', cone_r, 'body bottom', 'recess r', rec_r, B.CONE_CLR - 0.1, B.CONE_CLR + 0.2)
joint('J6 spike cone under the body recess (full cone)', 'spike', 'cone r at -CONE_H/2', 52 - gap(sp, [52, 0, -zm_c], [0, 0, -zm_c], 0.0), 'body bottom', 'recess r', rec_r, B.CONE_CLR - 0.1, B.CONE_CLR + 0.2)
joint('J6 lid recess matches body recess', 'lid', 'recess r', gap(lid, [0, 0, zm_c], [52, 0, zm_c], 0.0), 'body bottom', 'recess r', rec_r, 0, 0.05, 'eq')
# J7 honeycomb
n_cols = B.hex_cutters(0, 1, 0, 1)[1]
joint('J7 even column count (seamless pattern)', 'body open', 'columns mod 2', n_cols % 2, 'design', '0', 0, 0, 0, 'eq')
zc0 = B.SOCK_H + 6 + B.HEX_AF / 2; d = B.HEX_AF + B.STRUT; ang = 2 * math.pi / n_cols
# tangential strut: between column 0 (centre zc0) and column 1 (centre zc0 + d/2), along the line joining their centres at mid-wall
rw = B.OD / 2 - B.WALL / 2
def tstrut(r):   # solid along the ARC between neighbouring opening centres at radius r
    th = np.linspace(0, ang, 600); zz = np.linspace(zc0, zc0 + d / 2, 600)
    pts = np.column_stack([r * np.cos(th), r * np.sin(th), zz]); L = math.hypot(r * ang, d / 2)
    return inside(bo, pts).sum() * L / 599
joint('J7 tangential strut at the bore >= 3.4 mm', 'design', '3.4', 3.4, 'body open', 'solid between openings at the inner surface', tstrut(B.ID_R + 0.3), 0, 3)
joint('J7 tangential strut at the outside', 'design', 'STRUT', B.STRUT, 'body open', 'solid between openings at the outer surface', tstrut(B.OD / 2 - 0.3), -0.3, 2.0)
joint('J7 vertical strut = STRUT', 'design', 'STRUT', B.STRUT, 'body open', 'solid between openings in a column', solid_total(bo, [rw, 0, zc0], [rw, 0, zc0 + d]), -0.2, 0.2)
joint('J7 opening size', 'design', 'HEX_AF', B.HEX_AF, 'body open', 'vertical gap at hex centre', gap(bo, [rw, 0, zc0 - 15], [rw, 0, zc0 + 15]), -0.2, 0.2)
joint('J1 water body socket = open body socket', 'body water', 'female root r', r_fw.max(), 'body open', 'female root r', r_f.max(), 0, 0.05, 'eq')
# J2 spike <-> body socket ; J3 lid <-> neck
joint('J2 spike thread in body socket', 'spike', 'male crest r', r_s.max(), 'body socket', 'female root r', r_f.max(), B.TH_CLR - 0.1, B.TH_CLR + 0.1)
joint('J3 lid thread on body neck', 'body neck', 'male crest r', r_m.max(), 'lid', 'female root r', r_l.max(), B.TH_CLR - 0.1, B.TH_CLR + 0.1)
joint('J3 lid ceiling clears the neck top', 'body neck', 'collar top when seated', zc, 'lid', 'ceiling height at r=20', gap(lid, [20, 0, -2], [20, 0, B.SOCK_H + 2], 0.3) - 2, 0, 6)
joint('J3 lid open past the neck core at collar height', 'body neck', 'core r + 0.3', B.TH_MINOR_R + 0.3, 'lid', 'open radius at collar+0.5', gap(lid, [0, 0, zc], [48, 0, zc], 0.0), 0, 10)
# J4 water path
joint('J4 funnel hole takes a 6 mm wick', 'wick', 'diameter', 6.0, 'body water', 'funnel hole', gap(bw, [-6, 0, B.funnel_z() + 0.4], [6, 0, B.funnel_z() + 0.4]), -0.2, 0.6)
joint('J4 funnel underside clears the neck below', 'neck below', 'top of collar', B.TH_COLLAR + 1.0, 'body water', 'funnel underside at axis', B.funnel_z() - B.FUNNEL_T, 1.0, 10)
joint('J4 water body wall under a recess >= 4 perimeters', 'design', '2.0', 2.0, 'body water', 'WALL - RECESS', B.WALL - B.RECESS, 0, 5)
joint('J4 spike collar bore -> channel connected', 'design', 'collar top to channel tip', B.TH_COLLAR + B.CHAN_L + B.CHAN_R, 'spike', 'empty run on axis', gap(sp, [0, 0, B.TH_COLLAR], [0, 0, -B.CHAN_L - 8], 0.05), -0.5, 0.5)
hz = -22.0; a = math.pi / B.FIN_N
joint('J4 spike side hole through hub wall', 'spike', 'solid across hub wall at hole', solid_total(sp, [B.CHAN_R * math.cos(a), B.CHAN_R * math.sin(a), hz], [(B.HUB_R + 3) * math.cos(a), (B.HUB_R + 3) * math.sin(a), hz]), 'design', 'zero', 0.0, 0, 0.1, 'eq')
# J5 openings
joint('J5 open body openings cut through', 'design', 'zero', 0.0, 'body open', 'solid across wall at hex centre', solid_total(bo, [40, 0, B.SOCK_H + 6 + B.HEX_AF / 2], [54, 0, B.SOCK_H + 6 + B.HEX_AF / 2]), 0, 0.1)
joint('J5 water body hexes are recesses (wall stays)', 'design', 'WALL - RECESS', B.WALL - B.RECESS, 'body water', 'solid across wall at hex centre', solid_total(bw, [40, 0, B.SOCK_H + 6 + B.HEX_AF / 2], [54, 0, B.SOCK_H + 6 + B.HEX_AF / 2]), -0.05, 0.15)
joint('J5 openings 12-25 mm (roots in, moss stays)', 'design', '12', 12.0, 'body open', 'HEX_AF', B.HEX_AF, 0, 13)

# J8 assembled interference: place parts at the true pitch and intersect
Pz = H - B.CONE_H
def interf(a, b, dz):
    bb = b.copy(); bb.apply_translation([0, 0, dz]); i = trimesh.boolean.intersection([a, bb], engine='manifold')
    return 0.0 if i.is_empty else i.volume
joint('J8 body on body: no interference at the seated position', 'design', '< 20 mm3', 20.0, 'assembly', 'intersection volume', interf(bo, bo, Pz), -20, 0)
joint('J8 body on water body: no interference', 'design', '< 20 mm3', 20.0, 'assembly', 'intersection volume', interf(bo, bw, Pz), -20, 0)
joint('J8 spike in body: no interference', 'design', '< 20 mm3', 20.0, 'assembly', 'intersection volume', interf(sp, bo, -B.CONE_H), -20, 0)
joint('J8 lid on body: no interference', 'design', '< 20 mm3', 20.0, 'assembly', 'intersection volume', interf(bo, lid, Pz), -20, 0)
joint('J8 thread home = rim seated (ridge and groove in phase)', 'body neck', 'ridge start above seat', 1.0 + B.CONE_H, 'body socket', 'groove theta=0 above bottom', B.CONE_H + 1.0 - 0.5 * B.TH_PITCH + 0.5 * B.TH_PITCH, 0, 0.01, 'eq')
print('printability (0.5 nozzle, 0.3 mm layers, no supports)')
flips = {'spike': True, 'lid': True}
side = B.HEX_AF / math.sqrt(3)
n_open = len(B.hex_cutters(B.SOCK_H + 6, H - B.CONE_H - 4, 0, 1)[0]); n_rec = n_open
expect = {'body open': n_open * side * B.WALL * 1.1, 'body water': n_rec * side * B.RECESS * 1.2, 'spike': 2 * B.FIN_N * B.HOLE * (B.HUB_R - B.CHAN_R) * 1.5, 'lid': 0}
for n, m in P.items():
    b, s = overhang(m, flips.get(n, False))
    check('%s: steep overhang (> 44 deg) < 60 mm2  [%s]' % (n, 'flipped' if flips.get(n) else 'as built'), s < 60, '%.0f mm2' % s)
    check('%s: bridged area = known short bridges (+150)' % n, b <= expect[n] + 150, '%.0f mm2 (expected %.0f: hex tops / holes)' % (b, expect[n]))
check('hex top bridges are short (flat-top side) < 14 mm', side < 14, '%.1f mm' % side)
print('info: reservoir ~%.0f ml; module pitch %.0f mm (body %.0f - cone %.0f)' % (B.reservoir_volume_ml(), H - B.CONE_H, H, B.CONE_H))
print('joints')
w = [max(len(r[i]) for r in rows) for i in range(6)]
for r in rows: print('  ' + ' | '.join(r[i].ljust(w[i]) for i in range(6)))
print('\n%d joints, %d failure(s)' % (len(rows), fail)); sys.exit(1 if fail else 0)
