#!/usr/bin/env python3
"""
Geometric assertions for the moss-pole parts. Checks the things a render can
lie about: the water bore is clear end to end, the panel slots are on the hex
FACES (not corners -- a v1 bug caught here), the B spigot actually fits the B
socket with clearance, the keys land in the notches, the openings really cut
through the wall, the wick hole passes the cap plate, and every part fits the
printer it's meant for.

Usage:  python verify_moss_pole.py            (reads ../stl and ../laser)
        python verify_moss_pole.py --taz      (envelope check against TAZ 2)
"""

import os
import sys
import math
import numpy as np
import trimesh
from shapely.geometry import Point

sys.path.insert(0, os.path.dirname(__file__))
import build_moss_pole as B  # noqa: E402  (parameters only; nothing is rebuilt)

STL = os.path.join(os.path.dirname(__file__), '..', 'stl')
LASER = os.path.join(os.path.dirname(__file__), '..', 'laser')
TAG = 'v1_h%d' % int(B.MODULE_H)
MINI2 = (160.0, 160.0, 180.0)
TAZ2 = (298.0, 275.0, 250.0)
ENV = TAZ2 if '--taz' in sys.argv else MINI2
ENV_NAME = 'TAZ 2' if '--taz' in sys.argv else 'Mini 2'

rng = np.random.default_rng(0)
fail = 0


def check(name, ok, detail=''):
    global fail
    fail += (not ok)
    print('  %-46s %s %s' % (name, 'PASS' if ok else 'FAIL', detail))


def load(name):
    m = trimesh.load(os.path.join(STL, name + '.stl'))
    m.process(validate=True)
    m.merge_vertices()
    return m


def inside(m, pts):
    pts = np.atleast_2d(pts)
    return np.concatenate([m.contains(pts[i:i + 500]) for i in range(0, len(pts), 500)])


def envelope(m):
    e = m.extents
    return (e[0] <= ENV[0] and e[1] <= ENV[1] and e[2] <= ENV[2]), \
        '%.0fx%.0fx%.0f vs %s' % (e[0], e[1], e[2], ENV_NAME)


def section_radii(m, z):
    s = m.section(plane_origin=[0, 0, z], plane_normal=[0, 0, 1])
    pts = np.vstack(s.discrete)
    return np.hypot(pts[:, 0], pts[:, 1]), pts


def axis_clear(m, z0, z1, r=0.0):
    zs = np.linspace(z0, z1, 60)
    return not inside(m, np.column_stack([np.full_like(zs, r), np.zeros_like(zs), zs])).any()


# ---------------- shared ----------------
print('shared')
tube = load(TAG + '_water_tube')
check('tube watertight, single body', tube.is_watertight and len(tube.split()) == 1)
check('tube bore clear end to end', axis_clear(tube, 0.5, B.MODULE_H - 0.5))
check('tube bore clear at wick radius', axis_clear(tube, 0.5, B.MODULE_H - 0.5, B.TUBE_ID / 2 - 0.5))
# a drip hole: ray through the wall at the first drip z, row 0 (+x)
z = B.DRIP_PITCH / 2
xs = np.linspace(B.TUBE_ID / 2 + 0.2, B.TUBE_OD / 2 - 0.2, 20)
check('drip hole passes through the wall', not inside(tube, np.column_stack([xs, np.zeros(20), np.full(20, z)])).any())
check('tube wall solid between drip holes', inside(tube, np.column_stack([xs, np.zeros(20), np.full(20, z + B.DRIP_PITCH / 2)])).all())

lid = load('v1_bottle_lid')
check('lid watertight', lid.is_watertight)
r, _ = section_radii(lid, B.LID_PLUG_H / 2)
check('lid bottle hole = neck clearance', abs(r.min() - B.BOTTLE_NECK_D / 2) < 0.15, '%.2f' % (2 * r.min()))
cup_id = B.CUP_OD - 2 * B.CUP_WALL
check('lid plug clears cup bore', cup_id / 2 - r.max() >= B.LID_PLUG_CLR - 0.05, 'gap %.2f/side' % (cup_id / 2 - r.max()))

# ---------------- A: hex hybrid ----------------
print('A: hex hybrid')
ring = load('v1_A_joiner_ring')
check('ring watertight, single body', ring.is_watertight and len(ring.split()) == 1)
check('ring fits ' + ENV_NAME, *envelope(ring))
slot_r = (B.AF_OUT - 2 * B.SLOT_OFFSET) / 2           # slot centreline, across-flats radius
face = np.array([slot_r, 0.0])                         # face at angle 0
corner = slot_r / math.cos(math.pi / 6) * np.array([math.cos(math.pi / 6), math.sin(math.pi / 6)])
check('slot open on the FACE (bottom)', not inside(ring, [[face[0], face[1], B.SLOT_D / 2]])[0])
check('slot open on the FACE (top)', not inside(ring, [[face[0], face[1], B.RING_H - B.SLOT_D / 2]])[0])
check('corner is solid (slots do not meet)', inside(ring, [[corner[0] * 0.99, corner[1] * 0.99, B.SLOT_D / 2]])[0])
check('web solid between top/bottom slots', inside(ring, [[face[0], face[1], B.RING_H / 2]])[0])
check('slot depth = SLOT_D', (not inside(ring, [[face[0], face[1], B.SLOT_D - 0.3]])[0])
      and inside(ring, [[face[0], face[1], B.SLOT_D + 0.3]])[0])
# slot width from a section through the slot zone
_, pts = section_radii(ring, B.SLOT_D / 2)
near = pts[(np.abs(pts[:, 1]) < 1.0) & (pts[:, 0] > 0)]
check('slot width = SLOT_W', abs(np.ptp(near[:, 0]) - B.SLOT_W) < 0.15, '%.2f' % np.ptp(near[:, 0]))
check('spider bore clear for the tube', axis_clear(ring, 0.5, B.RING_H - 0.5, (B.TUBE_OD + B.TUBE_CLEAR) / 2 - 0.2))
check('spider hub solid', inside(ring, [[B.HUB_OD / 2 - 1.0, 0, B.RING_H / 2]])[0])
# slot length vs panel width
_, panel_w = B.hex_slots(0, 1)
along = pts[(np.abs(pts[:, 0] - slot_r) < B.SLOT_W / 2 - 0.2) & (np.abs(pts[:, 1]) < 26)]   # < 26: exclude the neighbouring slot end at the corner
check('slot longer than panel by ~1 mm', 0.8 <= np.ptp(along[:, 1]) - panel_w <= 1.2,
      'slot %.1f, panel %.1f' % (np.ptp(along[:, 1]), panel_w))

cap = load('v1_A_cap_reservoir')
check('A cap watertight', cap.is_watertight)
check('A cap wick hole through plate', axis_clear(cap, B.RING_H - 0.5, B.RING_H + B.PLATE_T + 0.5))
check('A cap plate solid beside wick hole', inside(cap, [[B.WICK_HOLE_D / 2 + 1.5, 0, B.RING_H + B.PLATE_T / 2]])[0])
check('A cap slots on bottom face', not inside(cap, [[face[0], face[1], B.SLOT_D / 2]])[0])
mouth = B.CUP_H + B.LID_FLANGE_T - 17.0   # PCO 1881: ~17 mm bead-to-mouth (measure a bottle!)
check('bottle mouth 5-12 mm above cup floor', 5.0 <= mouth <= 12.0, 'water line ~%.0f mm' % mouth)

stake = load('v1_A_stake_base')
check('A stake watertight, single body', stake.is_watertight and len(stake.split()) == 1)
check('A stake fits ' + ENV_NAME, *envelope(stake))
check('A stake slots on top face', not inside(stake, [[face[0], face[1], B.STAKE_RING_H - B.SLOT_D / 2]])[0])
check('A stake tube locator bore clear', axis_clear(stake, B.PLATE_T + 0.5, B.PLATE_T + 7.5, B.TUBE_OD / 2))
check('A stake fins reach FIN_L', abs(stake.bounds[0][2] + B.FIN_L) < 0.1)

# laser panel
svg = os.path.join(LASER, TAG + '_A_panel_3mm.svg')
dxf = os.path.join(LASER, TAG + '_A_panel_3mm.dxf')
check('panel DXF + SVG exist', os.path.exists(svg) and os.path.exists(dxf))
poly, W, H, n = B.panel_polygon()
check('panel height = MODULE_H', abs(H - B.MODULE_H) < 1e-6)
check('panel has openings', n > 0, '%d' % n)
thin = poly.buffer(-(B.PANEL_STRUT / 2 - 0.1))
check('no strut thinner than PANEL_STRUT', thin.is_valid and not thin.is_empty and
      thin.geom_type == 'Polygon', 'min strut ok' if thin.geom_type == 'Polygon' else thin.geom_type)
check('panel top band solid for the slot',
      all(poly.contains(Point(x, H - B.SLOT_D / 2)) for x in np.linspace(1, W - 1, 15)))

# ---------------- B: full-print cylinder ----------------
print('B: full-print cylinder')
mod = load(TAG + '_B_module')
check('B module watertight, single body', mod.is_watertight and len(mod.split()) == 1)
check('B module fits ' + ENV_NAME, *envelope(mod))
def away_from_keys(pts):
    ang = np.degrees(np.arctan2(pts[:, 1], pts[:, 0])) % (360.0 / B.KEY_N)
    return (ang > 10) & (ang < 360.0 / B.KEY_N - 10)


def spigot_od_of(m, z):
    r, p = section_radii(m, z)
    return 2 * r[away_from_keys(p)].max()


def socket_id_of(m, z):
    r, p = section_radii(m, z)
    k = away_from_keys(p) & (r > 40)                       # ignore hub/spider
    return 2 * r[k].min()


r_sp, pts_sp = section_radii(mod, -B.SPIGOT_H / 2)
spig_od = spigot_od_of(mod, -B.SPIGOT_H / 2)
sock_id = socket_id_of(mod, B.MODULE_H - B.SPIGOT_H / 2)
check('spigot OD < socket ID with clearance', 0.7 <= sock_id - spig_od <= 1.1,
      'spigot %.2f, socket %.2f' % (spig_od, sock_id))
check('keys stay inside the outer wall', 2 * r_sp.max() < B.AF_OUT - 1.0, 'key OD %.1f' % (2 * r_sp.max()))
nz = B.MODULE_H - B.SPIGOT_H / 2
check('rim notch open at key angle', not inside(mod, [[B.AF_OUT / 2 - B.CYL_WALL / 2, 0, nz]])[0])
a = 2 * math.pi / B.KEY_N / 2
check('rim solid between notches', inside(mod, [[(B.AF_OUT / 2 - B.CYL_WALL / 2) * math.cos(a),
                                                   (B.AF_OUT / 2 - B.CYL_WALL / 2) * math.sin(a), nz]])[0])
check('B spider bore clear for the tube', axis_clear(mod, 0.5, B.SPIDER_T - 0.5, (B.TUBE_OD + B.TUBE_CLEAR) / 2 - 0.2))
# opening at column 0's first hex centre, strut halfway to the next hex
rw = B.AF_OUT / 2 - B.CYL_WALL / 2
zc = B.CYL_BAND + B.CYL_HEX_AF / 2
d = B.CYL_HEX_AF + B.CYL_STRUT
check('honeycomb opening cuts through the wall', not inside(mod, [[rw, 0, zc]])[0])
check('strut solid between openings', inside(mod, [[rw, 0, zc + d / 2]])[0])
check('solid band below first row', inside(mod, [[rw, 0, B.CYL_BAND / 2]])[0])
a60 = math.pi / 3   # off the key/notch angle
check('solid band above last row', inside(mod, [[rw * math.cos(a60), rw * math.sin(a60), B.MODULE_H - B.CYL_BAND / 2 - 1.0]])[0])
# open area fraction on the lattice band (sampled on the mid-wall surface)
t = rng.uniform(0, 2 * math.pi, 600)
zz = rng.uniform(B.CYL_BAND, B.MODULE_H - B.CYL_BAND, 600)
frac = 1 - inside(mod, np.column_stack([rw * np.cos(t), rw * np.sin(t), zz])).mean()
check('open area 35-60 % of the lattice band', 0.35 <= frac <= 0.60, '%.0f %%' % (100 * frac))

capb = load('v1_B_cap_reservoir')
check('B cap watertight', capb.is_watertight)
check('B cap wick hole through plate', axis_clear(capb, -0.5, B.PLATE_T + 0.5))
check('B cap spigot matches module spigot', abs(spigot_od_of(capb, -B.SPIGOT_H / 2) - spig_od) < 0.05)

stakeb = load('v1_B_stake_base')
check('B stake watertight, single body', stakeb.is_watertight and len(stakeb.split()) == 1)
check('B stake fits ' + ENV_NAME, *envelope(stakeb))
check('B stake socket ID matches module socket', abs(socket_id_of(stakeb, B.STAKE_RING_H - B.SPIGOT_H / 2) - sock_id) < 0.05)
check('B stake notch open at key angle', not inside(stakeb, [[B.AF_OUT / 2 - B.CYL_WALL / 2, 0,
                                                              B.STAKE_RING_H - B.SPIGOT_H / 2]])[0])
check('B stake tube locator bore clear', axis_clear(stakeb, B.PLATE_T + 0.5, B.PLATE_T + 7.5, B.TUBE_OD / 2))

print('\n%d failure(s)' % fail)
sys.exit(1 if fail else 0)
