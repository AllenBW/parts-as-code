#!/usr/bin/env python3
import os, math, numpy as np, trimesh, matplotlib
matplotlib.use('Agg'); import matplotlib.pyplot as plt
from trimesh.transformations import rotation_matrix
from render_hybrid import project, PRINT
from matplotlib.collections import PolyCollection
import build_print as B
DOCS = os.path.join(os.path.dirname(__file__), '..', 'docs'); TAG = 'h%d' % int(B.BODY_H)
OPEN, WATER, SPIKE, LID = '#c9a27c', '#8fa98a', '#b08968', '#a89a8a'
def L(n): return trimesh.load(os.path.join(B.OUT_STL, n + '.stl'))
def at(m, z): m = m.copy(); m.apply_translation([0, 0, z]); return m
def draw(ax, parts, title, elev=16):
    p, c = project(parts, 32, elev); ax.add_collection(PolyCollection(p, facecolors=c, edgecolors='none', antialiased=False))
    ax.autoscale(); ax.set_aspect('equal'); ax.set_axis_off(); ax.set_title(title, fontsize=11)
def pole(g):
    H = B.BODY_H - B.CONE_H; parts = [(L('p_spike'), SPIKE)]; z = g     # module pitch: rim seats on rim
    parts.append((at(L('p_body_open_' + TAG), z), OPEN)); z += H + g
    parts.append((at(L('p_body_water_' + TAG), z), WATER)); z += H + g
    parts.append((at(L('p_lid'), z), LID)); return parts
fig = plt.figure(figsize=(11, 11)); fig.suptitle('moss-pole, print-only: spike, open body, watering body, lid', fontsize=13)
draw(fig.add_subplot(1, 2, 1), pole(0), 'assembled'); draw(fig.add_subplot(1, 2, 2), pole(25), 'exploded')
plt.tight_layout(); plt.savefig(os.path.join(DOCS, 'fig_print_pole.png'), dpi=80); plt.close()
fig, axes = plt.subplots(1, 4, figsize=(18, 7))
for ax, (n, c, flip) in zip(axes, [('p_body_open_' + TAG, OPEN, False), ('p_body_water_' + TAG, WATER, False), ('p_spike', SPIKE, True), ('p_lid', LID, True)]):
    m = L(n)
    if flip: m.apply_transform(rotation_matrix(math.pi, [1, 0, 0]))
    draw(ax, [(m, c)], '%s\n%.0f x %.0f x %.0f mm, %.0f cm³%s' % (n, *m.extents, m.volume / 1000, '  (printed flipped)' if flip else ''), elev=22)
plt.suptitle('parts in PRINT orientation — 0.5 nozzle, 0.3 mm layers, no supports', fontsize=12)
plt.tight_layout(); plt.savefig(os.path.join(DOCS, 'fig_print_parts.png'), dpi=80); plt.close()
# section through the watering body to show the funnel
m = L('p_body_water_' + TAG); s = m.section(plane_origin=[0, 0, 0], plane_normal=[0, 1, 0])
fig, ax = plt.subplots(figsize=(5, 9))
for e in s.discrete: ax.plot(e[:, 0], e[:, 2], 'k-', lw=0.8)
ax.set_aspect('equal'); ax.set_title('watering body, section: socket, 45° funnel to wick hole, recessed hexes, neck', fontsize=9)
plt.tight_layout(); plt.savefig(os.path.join(DOCS, 'fig_print_water_section.png'), dpi=80); plt.close()
print('ok')
