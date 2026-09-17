#!/usr/bin/env python3
"""docs figures: assembled + exploded pole, parts gallery, sheet nest."""
import os, math, glob, numpy as np, trimesh, matplotlib
matplotlib.use('Agg'); import matplotlib.pyplot as plt
from matplotlib.collections import PolyCollection
from trimesh.creation import extrude_polygon
from trimesh.transformations import rotation_matrix
import build_hybrid as B
from lib import _rotz
DOCS = os.path.join(os.path.dirname(__file__), '..', 'docs'); os.makedirs(DOCS, exist_ok=True)
PRINT, ACRYL = '#c9a27c', '#4f8fb5'; TAG = 'h%d' % int(B.MODULE_H)

def L(n): return trimesh.load(os.path.join(B.OUT_STL, n + '.stl'))
def at(m, z): m = m.copy(); m.apply_translation([0, 0, z]); return m

def project(parts, azim=32, elev=16):
    a, e = math.radians(azim), math.radians(elev)
    R = np.array([[1, 0, 0], [0, math.cos(e), -math.sin(e)], [0, math.sin(e), math.cos(e)]]) @ \
        np.array([[math.cos(a), -math.sin(a), 0], [math.sin(a), math.cos(a), 0], [0, 0, 1]])
    light = np.array([0.3, -0.5, 0.8]); light /= np.linalg.norm(light)
    P, D, C = [], [], []
    for m, c in parts:
        v = m.vertices @ R.T; tri = v[m.faces]; n = m.face_normals @ R.T
        sh = 0.55 + 0.45 * np.clip(n @ light, 0, 1); base = np.array(matplotlib.colors.to_rgb(c))
        P.append(tri[:, :, [0, 2]]); D.append(tri[:, :, 1].mean(axis=1)); C.append(np.clip(base[None] * sh[:, None], 0, 1))
    P, D, C = np.vstack(P), np.concatenate(D), np.vstack(C); o = np.argsort(-D); return P[o], C[o]

def draw(ax, parts, title, elev=16):
    p, c = project(parts, 32, elev); ax.add_collection(PolyCollection(p, facecolors=c, edgecolors='none', antialiased=False))
    ax.autoscale(); ax.set_aspect('equal'); ax.set_axis_off(); ax.set_title(title, fontsize=11)

def panels(z0):
    poly, W, H = B.panel(); out = []
    for i in range(6):
        p = extrude_polygon(poly, height=B.SHEET_T); p.apply_translation([-W / 2, 0, -B.SHEET_T / 2])
        p.apply_transform(rotation_matrix(math.pi / 2, [1, 0, 0])); p.apply_translation([0, -B.SLOT_AF / 2, z0])
        out.append((_rotz(p, i * math.pi / 3 + math.pi / 2), ACRYL))
    return out

def pole(g, modules=2):
    parts = [(L('spike'), PRINT), (at(L('stake_ring'), 0 + g * 0.4), PRINT)]; z = B.TOP_H + g
    for k in range(modules):
        parts.append((at(L('cage_' + TAG), z), PRINT))
        parts += panels(z + B.BASE_H - B.SLOT_D + g * 0.5)
        parts.append((at(L('tube_' + TAG), z - B.TOP_H + 3 + g * 0.3), PRINT))
        z += B.MODULE_H + g
        if k < modules - 1:
            parts.append((at(L('top_ring'), z), PRINT)); z += B.TOP_H + g
    parts.append((at(L('cap'), z), PRINT))
    parts.append((at(L('lid'), z + B.TOP_H + 3 + 24 - 4 + g), PRINT))
    return parts

fig = plt.figure(figsize=(12, 11)); fig.suptitle('moss-pole v3: printed skeleton (tan) + laser lattice (blue)', fontsize=13)
draw(fig.add_subplot(1, 2, 1), pole(0), 'assembled, two modules'); draw(fig.add_subplot(1, 2, 2), pole(22), 'exploded')
plt.tight_layout(); plt.savefig(os.path.join(DOCS, 'fig_pole.png'), dpi=80); plt.close()

names = ['cage_' + TAG, 'top_ring', 'cap', 'stake_ring', 'spike', 'tube_' + TAG, 'lid', 'coupon_thread_male']
fig, axes = plt.subplots(2, 4, figsize=(18, 10))
for ax, n in zip(axes.flat, names):
    m = L(n)
    if n in ('spike', 'lid'): m.apply_transform(rotation_matrix(math.pi, [1, 0, 0]))   # printed flipped
    draw(ax, [(m, PRINT)], '%s   %.0f x %.0f x %.0f mm, %.0f cm³%s' % (n, *m.extents, m.volume / 1000, '  (flipped)' if n in ('spike', 'lid') else ''), elev=24)
plt.suptitle('printed parts (PETG, 0.5 nozzle, 0.3 mm layers, no supports) — shown in PRINT orientation', fontsize=13); plt.tight_layout(); plt.savefig(os.path.join(DOCS, 'fig_parts.png'), dpi=80); plt.close()

items, cols, rows = B.nest(*B.panel()[1:])
fig, ax = plt.subplots(figsize=(12, 6.5)); ax.add_patch(plt.Rectangle((0, 0), B.SHEET_W, B.SHEET_H, fill=False, ls='--', color='gray'))
for p in items:
    for r in [p.exterior] + list(p.interiors):
        x, y = r.xy; ax.plot(x, y, 'r-', lw=0.6)
ax.set_aspect('equal'); ax.set_xlim(-5, B.SHEET_W + 5); ax.set_ylim(-5, B.SHEET_H + 5)
ax.set_title('sheet_12x24_%s: %d panels = %d modules on one 3 mm sheet (P2 bed 600 x 308)' % (TAG, len(items), len(items) // 6))
plt.tight_layout(); plt.savefig(os.path.join(DOCS, 'fig_sheet.png'), dpi=80); plt.close()
print('ok')
