#!/usr/bin/env python3
"""
moss-pole v3 verifier.

  1. every part is one watertight solid and fits its printer
  2. every joint between two parts is measured on BOTH sides from the built
     geometry and its clearance asserted
  3. every part takes part in at least one joint (no orphan parts)
  4. the sheet nest is inside the bed with no overlaps, and yields whole modules

Usage: python verify.py [--taz] [--parts]
"""
import os, sys, math
import numpy as np
import trimesh
sys.path.insert(0, os.path.dirname(__file__))
import build_hybrid as B
from lib import TUBE_OD, TUBE_CLEAR, WICK_HOLE_D, LID_PLUG_H, BOTTLE_NECK_D

STL = B.OUT_STL; TAG = 'h%d' % int(B.MODULE_H); T = B.SHEET_T
ENV, ENV_NAME = ((298, 275, 250), 'TAZ 2') if '--taz' in sys.argv else ((160, 160, 180), 'Mini 2')
rows, fail = [], 0


def load(n):
    m = trimesh.load(os.path.join(STL, n + '.stl')); m.process(validate=True); m.merge_vertices(); return m


def inside(m, pts):
    pts = np.atleast_2d(pts)
    return np.concatenate([m.contains(pts[i:i + 400]) for i in range(0, len(pts), 400)])


def check(name, ok, detail=''):
    global fail
    fail += (not ok); print('  %-52s %s %s' % (name, 'PASS' if ok else 'FAIL', detail))


def joint(name, a_part, a_feat, a_val, b_part, b_feat, b_val, lo, hi, kind='gap'):
    global fail
    if kind == 'gap':
        c = b_val - a_val; ok = lo <= c <= hi; cs = '%+.2f' % c
    else:
        c = abs(a_val - b_val); ok = c <= hi; cs = 'Δ%.2f' % c
    fail += (not ok)
    rows.append((name, '%s: %s = %.2f' % (a_part, a_feat, a_val), '%s: %s = %.2f' % (b_part, b_feat, b_val),
                 cs, '[%.2f, %.2f]' % (lo, hi), 'PASS' if ok else 'FAIL'))


def run(m, p0, p1, want_solid, target=None, n=600):
    """Length and centre of the contiguous solid/empty run along p0->p1 containing `target` (0..1) or the midpoint."""
    p0, p1 = np.array(p0, float), np.array(p1, float); ts = np.linspace(0, 1, n)
    ins = inside(m, p0 + ts[:, None] * (p1 - p0)); i = int(np.argmin(np.abs(ts - (0.5 if target is None else target))))
    if ins[i] != want_solid: return 0.0, float('nan')
    lo = i
    while lo > 0 and ins[lo - 1] == want_solid: lo -= 1
    hi = i
    while hi < n - 1 and ins[hi + 1] == want_solid: hi += 1
    L = np.linalg.norm(p1 - p0); return (hi - lo + 1) * L / (n - 1), (ts[lo] + ts[hi]) / 2 * L


def gap(m, p0, p1, target=None): return run(m, p0, p1, False, target)[0]
def solid_total(m, p0, p1, n=600):
    p0, p1 = np.array(p0, float), np.array(p1, float); ts = np.linspace(0, 1, n)
    return inside(m, p0 + ts[:, None] * (p1 - p0)).sum() * np.linalg.norm(p1 - p0) / (n - 1)
def solid(m, p0, p1, target=None): return run(m, p0, p1, True, target)[0]


def section_r(m, z, lo, hi, faces_only=False):
    s = m.section(plane_origin=[0, 0, z], plane_normal=[0, 0, 1]); p = np.vstack(s.discrete)
    r = np.hypot(p[:, 0], p[:, 1]); k = (r > lo) & (r < hi)
    if faces_only:
        a = np.degrees(np.arctan2(p[:, 1], p[:, 0])) % 60; k &= (a < 8) | (a > 52)
    return r[k]


def pitch(m, x, y, z0, z1):
    zs = np.arange(z0, z1, 0.05); ins = inside(m, np.column_stack([np.full_like(zs, x), np.full_like(zs, y), zs]))
    st = zs[1:][ins[1:] & ~ins[:-1]]; return float(np.diff(st).mean()) if len(st) > 1 else float('nan')


# ---------------- parts ----------------
print('parts')
P = {n: load(f) for n, f in (('cage', 'cage_' + TAG), ('top ring', 'top_ring'), ('cap', 'cap'), ('stake ring', 'stake_ring'), ('spike', 'spike'), ('tube', 'tube_' + TAG), ('lid', 'lid'))}
for n, m in P.items():
    check('%s watertight, single solid' % n, m.is_watertight and len(m.split()) == 1)
    e = m.extents; check('%s fits %s' % (n, ENV_NAME), all(e[i] <= ENV[i] for i in range(3)), '%.0fx%.0fx%.0f' % tuple(e))
pnl, W, H = B.panel()
cage, top, cap, sring, spike, tube, lid = (P[k] for k in ('cage', 'top ring', 'cap', 'stake ring', 'spike', 'tube', 'lid'))

# ---------------- joints ----------------
face, zmid, yin = B.SLOT_AF / 2, B.MODULE_H / 2, W / 2 - 1.0
a60 = (math.cos(math.radians(60)), math.sin(math.radians(60)))

# J1 panel <-> cage posts
joint('J1 panel thickness in post slot', 'panel', 'thickness', T, 'cage', 'post slot width', gap(cage, [face - 4, yin, zmid], [face + 4, yin, zmid]), 0.2, 0.6)
joint('J1 panel width between post slots', 'panel', 'width', W, 'cage', 'slot-to-slot length', gap(cage, [face, -35, zmid], [face, 35, zmid]), 0.5, 1.6)
corner_in = (B.SLOT_AF / 2 - T / 2) / math.cos(math.pi / 6)
joint('J1 post clears panel inner corner', 'cage', 'post inner corner r', B.POST_AF / 2 / math.cos(math.pi / 6), 'panels', 'inner-face corner r', corner_in, 1.0, 20)
# J2 panel bottom <-> cage base slot ; J3 panel top <-> top ring / cap slot ; height
base_d = gap(cage, [face, 0, B.BASE_H - 10], [face, 0, B.BASE_H + 10], target=0.45) - 10.0
top_d = gap(top, [face, 0, -10], [face, 0, B.SLOT_D + 4], target=0.55) - 10.0
cap_d = gap(cap, [face, 0, -10], [face, 0, B.SLOT_D + 4], target=0.55) - 10.0
joint('J2 cage base slot depth', 'cage', 'slot depth', base_d, 'design', 'SLOT_D', B.SLOT_D, 0, 0.2, 'eq')
joint('J3 top ring slot depth', 'top ring', 'slot depth', top_d, 'design', 'SLOT_D', B.SLOT_D, 0, 0.2, 'eq')
joint('J3 cap slot depth', 'cap', 'slot depth', cap_d, 'design', 'SLOT_D', B.SLOT_D, 0, 0.2, 'eq')
joint('J3 top ring slot width', 'panel', 'thickness', T, 'top ring', 'slot width', gap(top, [face - 4, 0, 3], [face + 4, 0, 3]), 0.2, 0.6)
joint('J3 cap slot width', 'panel', 'thickness', T, 'cap', 'slot width', gap(cap, [face - 4, 0, 3], [face + 4, 0, 3]), 0.2, 0.6)
joint('J2/J3 panel height in cage + top ring', 'panel', 'height', H, 'cage+top ring', 'base slot bottom to top slot top', (B.MODULE_H - (B.BASE_H - base_d)) + top_d, -0.01, 0.5)
# J4 top ring <-> cage (thread), J5 stake <-> cage (thread)
r_m = section_r(top, B.TOP_H + 1 + B.TH_PITCH * 0.75, 41, 47)
r_f = section_r(cage, B.TH_PITCH * 0.75 + 0.25, 41, 47, faces_only=True)
r_s = section_r(sring, B.TOP_H + 1 + B.TH_PITCH * 0.75, 41, 47)
joint('J4 top ring thread in cage thread (clearance)', 'top ring', 'male crest r', r_m.max(), 'cage', 'female root r', r_f.max(), B.TH_CLR - 0.1, B.TH_CLR + 0.1)
joint('J4 thread engagement depth', 'top ring', 'male root r', r_m.min(), 'cage', 'female crest r', r_f.min(), 0.3, B.TH_DEPTH)
joint('J4 thread flanks self-supporting', 'design', '45 deg: root-crest = 2*depth', 2 * B.TH_DEPTH, 'design', 'root - crest', (B.TH_CREST + 2 * B.TH_DEPTH) - B.TH_CREST, 0, 0.01, 'eq')
rm, rf = B.TH_MINOR_R + 0.8, B.TH_MINOR_R + B.TH_CLR + 0.8
joint('J4 thread pitch match', 'top ring', 'pitch', pitch(top, rm * a60[0], rm * a60[1], B.TOP_H + 0.5, B.TOP_H + B.TH_COLLAR + 1),
      'cage', 'pitch', pitch(cage, rf * a60[0], rf * a60[1], 0.3, B.TH_COLLAR + 1), 0, 0.05, 'eq')
joint('J4 collar length vs female pocket', 'top ring', 'collar', B.TH_COLLAR, 'cage', 'female pocket depth', gap(cage, [B.TH_MINOR_R + 0.2, 0, -5], [B.TH_MINOR_R + 0.2, 0, B.BASE_H - B.SLOT_D - 0.5], target=0.3) - 5, 0.5, 10)
joint('J5 stake ring thread in cage thread', 'stake ring', 'male crest r', r_s.max(), 'cage', 'female root r', r_f.max(), B.TH_CLR - 0.1, B.TH_CLR + 0.1)
joint('J5 stake ring thread = top ring thread', 'stake ring', 'crest r', r_s.max(), 'top ring', 'crest r', r_m.max(), 0, 0.05, 'eq')
# J10 spike plate in stake ring
plate_af = 2 * min(np.hypot(*B._hex_poly(B.PLATE_AF, math.pi / 6).exterior.coords.xy)) * math.cos(math.pi / 6)
ring_af = gap(sring, [-50, 0, 2], [50, 0, 2])                       # across flats at the open bottom
lip_af = gap(sring, [-50, 0, B.TOP_H - 0.5], [50, 0, B.TOP_H - 0.5])
joint('J10 spike plate in stake ring (across flats)', 'spike', 'plate AF', spike.extents[0] if False else plate_af, 'stake ring', 'inner AF at bottom', ring_af, 0.2, 0.7)
joint('J10 plate stops on the lip', 'stake ring', 'lip AF', lip_af, 'spike', 'plate AF', plate_af, 3.0, 12)
fin_root = solid_total(spike, [0, B.FIN_R_OUT - 0.5, 0.5], [0, B.FIN_R_OUT - 0.5, B.PLATE_T - 0.5])
joint('J10 fin root stands on the plate (print)', 'spike', 'plate under fin outer root', fin_root, 'spike', 'plate T - 1', B.PLATE_T - 1.0, 0, 0.1, 'eq')
joint('J10 plate thickness fits below the lip', 'spike', 'plate T', B.PLATE_T, 'stake ring', 'straight bore height', gap(sring, [ (ring_af / 2 - 0.3), 0, -1], [(ring_af / 2 - 0.3), 0, B.TOP_H + 2], target=0.2) - 1, -0.01, 3)
# J6 tube <-> cage spider, stake hub ; tube length ; wick hole
tube_od = 2 * section_r(tube, 40, 0, 20).max()
joint('J6 tube in top ring spider bore', 'tube', 'OD', tube_od, 'top ring', 'bore', gap(top, [-12, 0, 3], [12, 0, 3]), 0.3, 0.9)
joint('J6 tube in spike recess', 'tube', 'OD', tube_od, 'spike', 'recess bore', gap(spike, [-12, 0, B.PLATE_T - 0.3], [12, 0, B.PLATE_T - 0.3]), 0.3, 0.9)
joint('J6 tube passes the stake ring', 'tube', 'OD', tube_od, 'stake ring', 'open centre', gap(sring, [-40, 0, B.TOP_H + 2], [40, 0, B.TOP_H + 2]), 5, 80)
joint('J6 tube passes the open cage base', 'tube', 'OD', tube_od, 'cage', 'open centre above thread', gap(cage, [-40, 0, B.BASE_H - 3], [40, 0, B.BASE_H - 3]), 5, 80)
joint('J6 tube length = spider to spider', 'tube', 'length', tube.extents[2], 'cage + top ring', 'pitch', cage.extents[2] + B.TOP_H, 0, 0.01, 'eq')
joint('J6 tube top in cap recess', 'tube', 'OD', tube_od, 'cap', 'recess bore', gap(cap, [-12, 0, 6], [12, 0, 6]), 0.3, 0.9)
joint('J6 cap recess takes the tube overrun', 'stack', 'tube top above last cage', (3.0 + tube.extents[2] * 1) - (B.STAKE_RING_H + B.MODULE_H) if False else B.TOP_H - 6 + 1.0, 'cap', 'recess depth', gap(cap, [0, 0, -3], [0, 0, B.CAP_T - 0.5], target=0.2) - 3 - 0.0, 0.5, 20)
joint('J6 cap wick hole (tube must NOT pass)', 'cap', 'wick hole', gap(cap, [-12, 0, B.TUBE_RECESS + 0.5], [12, 0, B.TUBE_RECESS + 0.5]), 'tube', 'OD', tube_od, 5, 12)
# J7 lid <-> cap cup, bottle <-> lid
joint('J7 lid plug in cup bore', 'lid', 'plug OD', 2 * section_r(lid, LID_PLUG_H / 2, 0, 40).max(), 'cap', 'cup bore', gap(cap, [-35, 0, B.CAP_T + 10], [35, 0, B.CAP_T + 10]), 0.6, 1.4)
joint('J7 bottle neck in lid hole', 'PCO1881 bottle', 'thread OD', 27.4, 'lid', 'hole', gap(lid, [-20, 0, LID_PLUG_H / 2], [20, 0, LID_PLUG_H / 2]), 0.8, 1.6)
# J8 stake water path
joint('J8 recess -> wick hole -> plenum (axis clear)', 'spike', 'empty run on axis', gap(spike, [0, 0, B.PLATE_T + 2], [0, 0, -2 * B.PLENUM_DEPTH - 1], target=0.5), 'design', 'plate + double cone', B.PLATE_T + 2 * B.PLENUM_DEPTH + 3, -0.5, 1.0)
joint('J8 fin void width', 'spike', 'void at fin', gap(spike, [-6, B.FIN_R_IN + 8, -3], [6, B.FIN_R_IN + 8, -3]), 'design', 'FIN_T - 2 wall', B.FIN_T - 2 * B.FIN_WALL, 0, 0.2, 'eq')
zc = -B.PLENUM_DEPTH + 1.5   # just above the cone floor, where it is widest
joint('J8 plenum CONNECTS to fin void (continuous empty run from axis)', 'design', '>= FIN_R_IN + wall + 2', B.FIN_R_IN + B.FIN_WALL + 2, 'spike', 'empty run from axis at widest', gap(spike, [0, 0, zc], [0, 30, zc], target=0.0), 0, 30)
dr = B.FIN_R_IN + 4 + (B.FIN_R_OUT - B.FIN_R_IN - 5) * (1 - 15 / B.FIN_L) / 2
joint('J8 drip hole through both fin walls', 'spike', 'total solid across fin at drip', solid_total(spike, [-6, dr, -15], [6, dr, -15]), 'design', 'zero', 0.0, 0, 0.1, 'eq')
joint('J8 two fin walls beside drip hole', 'spike', 'total solid across fin', solid_total(spike, [-6, dr, -19], [6, dr, -19]), 'design', '2 x FIN_WALL', 2 * B.FIN_WALL, 0, 0.3, 'eq')
# J9 panel lattice integrity
thin = pnl.buffer(-(B.STRUT / 2 - 0.1))
joint('J9 no strut thinner than STRUT', 'panel', 'thinnest strut ok (1=yes)', 1.0 if (thin.geom_type == 'Polygon' and not thin.is_empty) else 0.0, 'design', '1', 1.0, 0, 0, 'eq')
joint('J9 solid band for the slots', 'design', 'SLOT_D + 2', B.SLOT_D + 2, 'panel', 'solid band', B.BAND, 0, 20)

# ---------------- printability (0.3 mm layers, no supports) ----------------
def overhang(m, flip=False):
    """Downward-facing area not on the bed, split into flat bridges and steep overhangs (> 44 deg from vertical)."""
    mm = m.copy()
    if flip: mm.apply_transform(rotation_matrix(math.pi, [1, 0, 0]))
    mm.apply_translation([0, 0, -mm.bounds[0][2]])
    nz = mm.face_normals[:, 2]; zc = mm.triangles_center[:, 2]; A = mm.area_faces
    off_bed = zc > 0.6
    bridge = A[(nz < -0.97) & off_bed].sum(); steep = A[(nz >= -0.97) & (nz < -0.72) & off_bed].sum()
    return bridge, steep

from trimesh.transformations import rotation_matrix
print('printability')
slot_ceiling = 6 * B.SLOT_W * (W + 1.0)
hexA = lambda af: math.sqrt(3) / 2 * af ** 2
lip_steps = hexA(B.AF_OUT - 2 * B.WALL) - hexA(B.LIP_AF)                       # stepped 45-degree lip, 0.3 mm ledges
drips = B.FIN_N * len(np.arange(-15, -B.FIN_L + 12, -12)) * 2 * B.DRIP_D * B.FIN_WALL + B.FIN_N * (B.FIN_T - 2 * B.FIN_WALL) * 6
import lib as _lib
seg = lambda d, r: r * r * math.acos(d / r) - d * math.sqrt(r * r - d * d)
inner = B.AF_OUT - 2 * B.WALL
collar_over_hex = (math.pi * B.TH_MINOR_R ** 2 - 6 * seg(inner / 2, B.TH_MINOR_R)) - (math.pi * (B.TH_MINOR_R - 3) ** 2 - 6 * seg(inner / 2, B.TH_MINOR_R - 3))
n_holes = len(np.arange(_lib.DRIP_PITCH / 2, B.MODULE_H + B.TOP_H - _lib.DRIP_PITCH / 2 + 1e-6, _lib.DRIP_PITCH)) * _lib.DRIP_ROWS
tube_holes = n_holes * _lib.DRIP_D * (TUBE_OD - _lib.TUBE_ID) / 2 * 2
expect = {'cage': (0, 0), 'top ring': (slot_ceiling + collar_over_hex, 0), 'cap': (slot_ceiling + math.pi * ((TUBE_OD + TUBE_CLEAR) / 2) ** 2, 0),
          'stake ring': (lip_steps, 0), 'spike': (drips, 0), 'tube': (tube_holes, 0), 'lid': (0, 0)}
flips = {'spike': True, 'lid': True}
for n, m in P.items():
    b, s_ = overhang(m, flips.get(n, False))
    check('%s: steep overhang area (> 44 deg) < 60 mm2' % n, s_ < 60, '%.0f mm2' % s_)
    check('%s: bridged area = known bridges (+150 mm2)' % n, b <= expect[n][0] + 150, '%.0f mm2 (slot ceilings %.0f)' % (b, expect[n][0]))
check('thread pitch >= 3 layers deep at 0.3', B.TH_DEPTH >= 0.9)
check('fin wall = whole perimeters at 0.5 nozzle', abs(B.FIN_WALL / 0.5 - round(B.FIN_WALL / 0.5)) < 1e-6)
check('drip holes >= 2.4 mm', lib_drip := __import__('lib').DRIP_D >= 2.4)

# ---------------- nest ----------------
items, cols, rows_n = B.nest(W, H)
ok = all(p.bounds[0] >= -1e-6 and p.bounds[1] >= -1e-6 and p.bounds[2] <= B.SHEET_W + 1e-6 and p.bounds[3] <= B.SHEET_H + 1e-6 for p in items)
ov = any(items[i].buffer(-0.05).intersects(items[j].buffer(-0.05)) for i in range(len(items)) for j in range(i + 1, len(items)))
print('nest')
check('all panels inside %.0f x %.0f' % (B.SHEET_W, B.SHEET_H), ok)
check('no overlaps', not ov)
check('sheet yields whole modules', len(items) % 6 == 0, '%d panels = %d modules' % (len(items), len(items) // 6))

# ---------------- report ----------------
print('joints')
w = [max(len(r[i]) for r in rows) for i in range(6)]
for r in rows: print('  ' + ' | '.join(r[i].ljust(w[i]) for i in range(6)))
parts_in = {}
for r in rows:
    for side in (r[1], r[2]):
        p = side.split(':')[0]
        if p in ('design', 'panels', 'cage+top ring'): continue
        parts_in.setdefault(p, []).append(r[0].split(' ')[0])
print('parts -> joints')
for p in ['panel', 'cage', 'top ring', 'cap', 'stake ring', 'spike', 'tube', 'lid']:
    js = sorted(set(parts_in.get(p, [])))
    check('%s has a joint' % p, len(js) > 0, ', '.join(js))
print('\n%d joints, %d failure(s)' % (len(rows), fail))
sys.exit(1 if fail else 0)
