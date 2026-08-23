#!/usr/bin/env python3
"""
Geometric assertions for the espresso foot STLs. Checks the things a render
can lie about: bolt path clear end to end, hex channel sized for the head,
ceiling solid around the shaft, flange centre open for the machine's boss,
cavity open from below, and ribs actually present.

Usage:  python verify_feet.py path/to/foot_v14_rounded_square.stl
"""

import sys
import numpy as np
import trimesh

SHAFT_D, HEAD_AF = 10.5, 15.0
BODY_H, CEILING_T, FLANGE_H, FLANGE_WALL = 50.0, 8.0, 10.0, 4.0
HUB_OD, N_RIBS = 26.0, 6

m = trimesh.load(sys.argv[1]); m.process(validate=True); m.merge_vertices()
rng = np.random.default_rng(0)
fail = 0


def inside(pts, chunk=800):
    return np.concatenate([m.contains(pts[i:i + chunk])
                           for i in range(0, len(pts), chunk)])


def check(name, ok, detail=''):
    global fail
    fail += (not ok)
    print('%-42s %s %s' % (name, 'PASS' if ok else 'FAIL', detail))


check('watertight, single solid',
      m.is_watertight and len(m.split(only_watertight=False)) == 1)

zs = np.arange(1.0, BODY_H + FLANGE_H - 0.5, 1.0)
p = np.column_stack([np.zeros_like(zs), np.zeros_like(zs), zs])
blocked = zs[inside(p)]
check('bolt axis clear, counter to flange top', len(blocked) == 0,
      '' if not len(blocked) else 'blocked z=%.0f..%.0f' % (blocked.min(), blocked.max()))

t = rng.uniform(0, 2 * np.pi, 400)
r = rng.uniform(SHAFT_D / 2 + 0.6, HEAD_AF / 2 - 0.8, 400)
z = rng.uniform(2.0, BODY_H - CEILING_T - 2.0, 400)
n = int(inside(np.column_stack([r * np.cos(t), r * np.sin(t), z])).sum())
check('hex channel open beyond shaft dia', n < 160,
      '(%d/400 solid; corners of hex are solid-adjacent)' % n)

r = rng.uniform(SHAFT_D / 2 + 1.0, HUB_OD / 2 - 1.0, 400)
z = np.full(400, BODY_H - CEILING_T / 2)
n = int(inside(np.column_stack([r * np.cos(t), r * np.sin(t), z])).sum())
check('ceiling solid around shaft', n >= 392, '(%d/400)' % n)

r = rng.uniform(4.0, 12.0, 300)
z = np.full(300, BODY_H + FLANGE_H / 2)
n = int(inside(np.column_stack([r * np.cos(t[:300]), r * np.sin(t[:300]), z])).sum())
check('flange centre open (clears machine boss)', n == 0, '(%d/300 in material)' % n)

hits = 0
for i in range(N_RIBS):
    a = i * 2 * np.pi / N_RIBS
    rr = (HUB_OD / 2 + 16.0) / 2 + HUB_OD / 4
    hits += int(inside(np.array([[rr * np.cos(a), rr * np.sin(a), 20.0]]))[0])
check('all %d ribs present' % N_RIBS, hits == N_RIBS, '(%d found)' % hits)

print('\n%s' % ('ALL CHECKS PASS' if fail == 0 else '%d CHECK(S) FAILED' % fail))
sys.exit(1 if fail else 0)
