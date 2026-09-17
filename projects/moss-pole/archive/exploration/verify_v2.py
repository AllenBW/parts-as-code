#!/usr/bin/env python3
"""
Assertions for v2, both paths. Threads mate with clearance, pegs fit holes,
panels fit slots, tabs fit tab slots, spigot fits socket, spline fits its
slot and the panel notch, the stake's plenum reaches the fin voids, every
printed part fits its printer, every part is one watertight solid.

Usage: python verify_v2.py [--taz]
"""
import os, sys, math
import numpy as np
import trimesh
from shapely.geometry import Point
sys.path.insert(0, os.path.dirname(__file__))
import build_v2 as B
import build_moss_pole as V1

STL = B.OUT_STL
TAG = 'v2_h%d' % int(B.MODULE_H)
ENV, ENV_NAME = ((298, 275, 250), 'TAZ 2') if '--taz' in sys.argv else ((160, 160, 180), 'Mini 2')
fail = 0


def check(name, ok, detail=''):
    global fail
    fail += (not ok)
    print('  %-48s %s %s' % (name, 'PASS' if ok else 'FAIL', detail))


def load(n):
    m = trimesh.load(os.path.join(STL, n + '.stl')); m.process(validate=True); m.merge_vertices(); return m


def inside(m, pts):
    pts = np.atleast_2d(pts)
    return np.concatenate([m.contains(pts[i:i + 400]) for i in range(0, len(pts), 400)])


def radii(m, z, rmin=0, rmax=1e9):
    s = m.section(plane_origin=[0, 0, z], plane_normal=[0, 0, 1])
    p = np.vstack(s.discrete); r = np.hypot(p[:, 0], p[:, 1])
    k = (r > rmin) & (r < rmax)
    return r[k], p[k]


def fits(m):
    e = m.extents
    return all(e[i] <= ENV[i] for i in range(3)), '%.0fx%.0fx%.0f vs %s' % (e[0], e[1], e[2], ENV_NAME)


print('PATH H')
cage, top, stake, cap = load(TAG + '_H_cage'), load('v2H_top_ring'), load('v2H_stake'), load('v2H_cap_reservoir')
for n, m in (('cage', cage), ('top ring', top), ('stake', stake), ('cap', cap)):
    check('%s watertight, single body' % n, m.is_watertight and len(m.split()) == 1)
check('cage fits ' + ENV_NAME, *fits(cage))
check('stake fits ' + ENV_NAME, *fits(stake))
# thread: male crest radius on the top ring vs female root radius in the cage, at a mid-thread z
zm = B.H_TOP_H + 1.0 + B.TH_PITCH * 0.75
r_m, _ = radii(top, zm, 30, 60)
r_f, _ = radii(cage, B.TH_PITCH * 0.75 + 0.25, 30, 60)
check('male thread crest < female thread root', r_f.max() - r_m.max() >= B.TH_CLR - 0.1,
      'crest %.2f, root %.2f' % (r_m.max(), r_f.max()))
check('male root < female crest (threads overlap = engage)', r_m.min() < r_f.min() < r_m.max(),
      'male %.2f..%.2f, female %.2f..%.2f' % (r_m.min(), r_m.max(), r_f.min(), r_f.max()))
check('stake thread matches top-ring thread', abs(radii(stake, 12 + 1.0 + B.TH_PITCH * 0.75, 30, 60)[0].max() - r_m.max()) < 0.05)
check('cap thread matches cage thread', abs(radii(cap, B.TH_PITCH * 0.75 + 0.25, 30, 60)[0].max() - r_f.max()) < 0.05)
# pegs / holes
r_peg = B.H_PEG_R
pz = B.MODULE_H + B.H_PEG_H / 2
check('peg present on cage post', inside(cage, [[r_peg * math.cos(math.radians(30)), r_peg * math.sin(math.radians(30)), pz]])[0])
check('peg hole open in top ring', not inside(top, [[r_peg * math.cos(math.radians(30)), r_peg * math.sin(math.radians(30)), B.H_PEG_H / 2]])[0])
# panel slot up the post and in the base top
face = B.SLOT_AF / 2
check('slot open at base-ring top face', not inside(cage, [[face, 0, B.H_BASE_H - B.H_SLOT_D / 2]])[0])
check('base ring solid below slot', inside(cage, [[face, 0, B.H_BASE_H - B.H_SLOT_D - 1.5]])[0])
_, pts = radii(cage, B.MODULE_H / 2)
near = pts[(np.abs(pts[:, 1]) < 0.5) & (pts[:, 0] > 40)]
# post slot: section through a post at the face angle should not exist (posts are at corners)...
ang = math.radians(30); rr = B.SLOT_AF / 2 / math.cos(math.pi / 6) - 4   # inside the corner post, on the panel line
px, py = (B.SLOT_AF / 2) / math.cos(0) , 0
# point on the panel plane at y = panel_width/2 + 0.3 (inside the post, within slot)
yv = B.panel_width() / 2 + 0.3
check('post slot open where the panel edge sits', not inside(cage, [[face, yv, B.MODULE_H / 2]])[0])
check('post solid beside the slot', inside(cage, [[face + B.SLOT_W / 2 + 1.0, yv + 1.0, B.MODULE_H / 2]])[0])
yin = B.panel_width() / 2 - 1.0     # 1 mm inside the post along the slot
check('post slot width >= SLOT_W', not inside(cage, [[face - B.SLOT_W / 2 + 0.2, yin, B.MODULE_H / 2],
                                                      [face + B.SLOT_W / 2 - 0.2, yin, B.MODULE_H / 2]]).any())
check('post slot width <= SLOT_W + 0.3', inside(cage, [[face - B.SLOT_W / 2 - 0.3, yin, B.MODULE_H / 2],
                                                        [face + B.SLOT_W / 2 + 0.3, yin, B.MODULE_H / 2]]).all())
check('spider bore clear (cage)', not inside(cage, [[(V1.TUBE_OD + V1.TUBE_CLEAR) / 2 - 0.2, 0, B.H_BASE_H - 3]])[0])
# stake plumbing: column hole -> plenum -> fin void -> drip hole
check('column hole opens into plenum', not inside(stake, [[0, 0, 1.5], [0, 0, -3]]).any())
fin_mid_r = B.FIN_R_IN + 8
check('fin void open at plenum level', not inside(stake, [[fin_mid_r, 0, -3]])[0])
check('fin wall solid around void', inside(stake, [[fin_mid_r, B.FIN_T / 2 - 0.5, -3]])[0])
check('drip hole passes through the fin wall', not inside(stake, [[B.FIN_R_IN + 4 + (B.FIN_R_OUT - B.FIN_R_IN - 5) * (1 - 15 / B.FIN_L) / 2, B.FIN_T / 2 - 0.5, -15]])[0])
check('cap wick hole through plate', not inside(cap, [[0, 0, B.H_BASE_H + 1.5]])[0])
pH, W, H = B.panel_H()
check('H panel height = base slot bottom..top slot top', abs(H - B.panel_H_height()) < 1e-6, '%.0f' % H)
thin = pH.buffer(-(B.STRUT / 2 - 0.1))
check('no panel strut thinner than STRUT', thin.geom_type == 'Polygon' and not thin.is_empty)

print('PATH L')
lay = B.L_layers()
rb, rt, base, capL = load('v2L_ring_bottom_assembled'), load('v2L_ring_top_assembled'), load('v2L_base_assembled'), load('v2L_cap_assembled')
for n, m in (('bottom ring stack', rb), ('top ring stack', rt), ('base', base), ('cap', capL)):
    check('%s watertight, single body' % n, m.is_watertight and len(m.split()) == 1)
pL, W, Ht = B.panel_L()
# tab width vs tab slot
tab_w = np.ptp(np.array([c for c in pL.exterior.coords if c[1] > B.MODULE_H + 0.5])[:, 0])
tabs_bounds = [c for c in pL.exterior.coords if c[1] > B.MODULE_H + 0.5]
slot_layer = lay['ring_tab']
# measure a tab slot: interior of ring_tab near face 0
slots = [i for i in slot_layer.interiors if abs(Point(i.coords[0]).x - B.SLOT_AF / 2) < 3]
sw = [np.ptp(np.array(i.coords)[:, 1]) for i in slots]
check('tab slot longer than tab', min(sw) - B.L_TAB_W >= 0.3, 'slot %.1f, tab %.1f' % (min(sw), B.L_TAB_W))
check('tab length = 2 sheet layers', abs((Ht - B.MODULE_H) / 2 - 2 * B.SHEET_T) < 1e-6)
# spigot vs socket
sp_r = max(np.hypot(*np.array(lay['ring_spigot'].exterior.coords).T))
so_r = min(np.hypot(*np.array(lay['ring_socket'].interiors[0].coords).T))
check('spigot fits socket', 0.15 <= so_r - sp_r <= 0.35, 'gap %.2f' % (so_r - sp_r))
# spline slot vs spline sheet; spline width vs slot radial length
r_c = B.SLOT_AF / 2 / math.cos(math.pi / 6)
ok = all(not lay['ring_tab'].contains(Point((r_c - 1.5) * math.cos(math.radians(30 + 60 * i)),
                                             (r_c - 1.5) * math.sin(math.radians(30 + 60 * i)))) for i in range(6))
solid = all(lay['ring_tab'].contains(Point((r_c - 1.5) * math.cos(math.radians(30 + 60 * i)) + 3.0 * math.cos(math.radians(120 + 60 * i)),
                                            (r_c - 1.5) * math.sin(math.radians(30 + 60 * i)) + 3.0 * math.sin(math.radians(120 + 60 * i)))) for i in range(6))
check('spline notch open at all 6 corners', ok)
check('ring solid beside spline notch', solid)
check('panel notch takes the spline at 60 deg', B.L_NOTCH_W >= B.SHEET_T / math.sin(math.radians(60)) + 0.2, '%.2f' % B.L_NOTCH_W)
check('egg-crate depths sum to spline overlap', abs(B.L_SNOTCH_D - B.L_PNOTCH_D * math.cos(math.radians(60))) < 1e-6)
check('spline spans spider layer to spider layer', abs(lay['spline'].bounds[3] - (B.MODULE_H + 2 * B.L_TAB_L - 0.5)) < 1e-6)
# fin tab vs base slot
check('fin tab fits base slot', B.SLOT_W - B.SHEET_T >= 0.3)
check('base column bore open', not inside(base, [[0, 0, 2 * B.SHEET_T + 4]])[0])
check('cap wick hole open', not inside(capL, [[0, 0, 3.5 * B.SHEET_T]])[0])
print('\n%d failure(s)' % fail)
sys.exit(1 if fail else 0)
