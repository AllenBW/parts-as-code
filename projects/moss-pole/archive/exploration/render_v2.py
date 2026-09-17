#!/usr/bin/env python3
"""Renders for docs: assembled + exploded views of both paths, and the sheet parts."""
import math, os, sys, numpy as np, trimesh, matplotlib
matplotlib.use('Agg'); import matplotlib.pyplot as plt
from trimesh.creation import extrude_polygon
from trimesh.transformations import rotation_matrix
import build_v2 as B
from build_moss_pole import _rotz
STL = B.OUT_STL; DOCS = os.path.join(os.path.dirname(__file__), '..', 'docs')
PRINT, ACRYL, SHEET = '#c9a27c', '#4f8fb5', '#3a3d45'

def L(n): return trimesh.load(os.path.join(STL, n + '.stl'))
def at(m, z, rot=0):
    m = m.copy()
    if rot: m.apply_transform(rotation_matrix(rot, [0, 0, 1]))
    m.apply_translation([0, 0, z]); return m

def panels(poly, z0, y_off=0.0):
    out = []
    for i in range(6):
        p = extrude_polygon(poly, height=B.SHEET_T)
        b = poly.bounds; p.apply_translation([-(b[0] + b[2]) / 2, -b[1] + y_off, -B.SHEET_T / 2])
        p.apply_transform(rotation_matrix(math.pi / 2, [1, 0, 0]))
        p.apply_translation([0, -B.SLOT_AF / 2, z0])
        out.append(_rotz(p, i * math.pi / 3 + math.pi / 2))
    return out

def H_assembly(g):
    tag = 'v2_h%d' % int(B.MODULE_H); H = B.MODULE_H
    parts = [(L('v2H_stake'), PRINT)]; z = 12.0 + g
    pH, _, _ = B.panel_H()
    for k in range(2):
        parts.append((at(L(tag + '_H_cage'), z), PRINT))
        parts += [(p, ACRYL) for p in panels(pH, z + B.H_BASE_H - B.H_SLOT_D + (g * 0.6 if g else 0))]
        parts.append((at(L(tag + '_water_tube'), z + B.H_BASE_H - 6 + g * 0.3), PRINT))
        z += H + g; parts.append((at(L('v2H_top_ring'), z), PRINT)); z += B.H_TOP_H + g
    parts.append((at(L('v2H_cap_reservoir'), z), PRINT))
    parts.append((at(L('v2H_bottle_lid'), z + B.H_BASE_H + 3 + B.CUP_H_ if hasattr(B, 'CUP_H_') else z + B.H_BASE_H + 3 + 24 - 4 + g), PRINT))
    return parts

def L_assembly(g):
    tag = 'v2_h%d' % int(B.MODULE_H); H = B.MODULE_H; t = B.SHEET_T
    lay = B.L_layers(); pL, _, _ = B.panel_L()
    parts = [(L('v2L_base_assembled'), SHEET)]; z = 2 * t + g
    r_c = B.SLOT_AF / 2 / math.cos(math.pi / 6)
    for k in range(2):
        parts.append((at(L('v2L_ring_bottom_assembled'), z), SHEET)); z += 5 * t + g
        zp = z - 2 * t                                     # tabs go 2 layers into the ring
        parts += [(p, ACRYL) for p in panels(pL, zp + (g * 0.5 if g else 0), 0)]
        for i in range(6):                                 # splines inside the corners
            s = extrude_polygon(lay['spline'], height=t); s.apply_translation([-lay['spline'].bounds[2], 0, 0])
            s.apply_transform(rotation_matrix(math.pi / 2, [1, 0, 0])); s.apply_translation([r_c, t / 2, zp + 0.25])
            parts.append((_rotz(s, math.radians(30 + 60 * i)), SHEET))
        parts.append((at(L(tag + '_water_tube'), zp + g * 0.3), SHEET))
        z = zp + 2 * t + H - 2 * t + g                     # panel body top
        parts.append((at(L('v2L_ring_top_assembled'), z - 2 * t + (g if g else 0)), SHEET)); z += 3 * t + g
    parts.append((at(L('v2L_cap_assembled'), z - 2 * t + g), SHEET))
    parts.append((at(L('v2L_lid_assembled'), z + 2 * t + B.L_CUP_H + 2 * g), SHEET))
    return parts

from matplotlib.collections import PolyCollection

def project(parts, azim=32, elev=16):
    a, e = math.radians(azim), math.radians(elev)
    Rz = np.array([[math.cos(a), -math.sin(a), 0], [math.sin(a), math.cos(a), 0], [0, 0, 1]])
    Rx = np.array([[1, 0, 0], [0, math.cos(e), -math.sin(e)], [0, math.sin(e), math.cos(e)]])
    R = Rx @ Rz
    light = np.array([0.3, -0.5, 0.8]); light /= np.linalg.norm(light)
    polys, depth, cols = [], [], []
    for m, c in parts:
        v = m.vertices @ R.T                      # x right, y depth (into screen), z up
        tri = v[m.faces]
        n = m.face_normals @ R.T
        shade = 0.55 + 0.45 * np.clip(n @ (light @ np.eye(3)), 0, 1)
        base = np.array(matplotlib.colors.to_rgb(c))
        polys.append(tri[:, :, [0, 2]]); depth.append(tri[:, :, 1].mean(axis=1))
        cols.append(np.clip(base[None, :] * shade[:, None], 0, 1))
    polys, depth, cols = np.vstack(polys), np.concatenate(depth), np.vstack(cols)
    order = np.argsort(-depth)                    # far first
    return polys[order], cols[order]

def draw(ax, parts, title, zwin=None, azim=32, elev=16):
    polys, cols = project(parts, azim, elev)
    ax.add_collection(PolyCollection(polys, facecolors=cols, edgecolors='none', antialiased=False))
    zmax = max(m.bounds[1][2] for m, _ in parts); zmin = min(m.bounds[0][2] for m, _ in parts)
    if zwin: zmin, zmax = zwin
    e = math.radians(elev); zsc = math.cos(e)
    ax.set_xlim(-72, 72); ax.set_ylim(zmin * zsc - 40 * math.sin(e), zmax * zsc + 40 * math.sin(e))
    ax.set_aspect('equal'); ax.set_axis_off(); ax.set_title(title, fontsize=11)

for name, fn, sub, zwin in (('H', H_assembly, 'Path H: printed skeleton (tan) + laser lattice (blue)', (150, 300)),
                            ('L', L_assembly, 'Path L: 100 % sheet (dark = structural sheet, blue = lattice)', (140, 290))):
    fig = plt.figure(figsize=(16, 11)); fig.suptitle(sub, fontsize=13)
    gs = fig.add_gridspec(1, 3, width_ratios=[1, 1, 1.4])
    draw(fig.add_subplot(gs[0]), fn(0), 'assembled')
    draw(fig.add_subplot(gs[1]), fn(22), 'exploded')
    draw(fig.add_subplot(gs[2]), fn(0), 'module joint, close-up', zwin=zwin, elev=22)
    plt.tight_layout(); plt.savefig(os.path.join(DOCS, 'fig_v2_%s.png' % name), dpi=80); plt.close()

# part gallery: H prints
fig, axes = plt.subplots(1, 4, figsize=(18, 6))
for ax, n in zip(axes, ['v2_h165_H_cage', 'v2H_top_ring', 'v2H_stake', 'v2H_cap_reservoir']):
    m = L(n); polys, cols = project([(m, PRINT)], 35, 24)
    ax.add_collection(PolyCollection(polys, facecolors=cols, edgecolors='none', antialiased=False))
    ax.autoscale(); ax.set_aspect('equal'); ax.set_axis_off()
    ax.set_title('%s  %s mm' % (n.replace('v2_h165_', '').replace('v2', ''), np.round(m.extents, 0).astype(int)), fontsize=9)
plt.tight_layout(); plt.savefig(os.path.join(DOCS, 'fig_v2_H_parts.png'), dpi=75); plt.close()

# sheet parts: L
lay = B.L_layers(); pL, _, _ = B.panel_L(); pH, _, _ = B.panel_H()
items = [('H panel', pH), ('L panel (tabs+notches)', pL)] + [(k, v) for k, v in lay.items()]
fig, axes = plt.subplots(2, 8, figsize=(22, 8))
for ax, (k, p) in zip(axes.flat, items):
    for ring in [p.exterior] + list(p.interiors):
        x, y = ring.xy; ax.plot(x, y, 'r-', lw=0.9)
    ax.set_aspect('equal'); ax.set_title(k, fontsize=9); ax.set_xticks([]); ax.set_yticks([])
for ax in axes.flat[len(items):]: ax.set_axis_off()
plt.suptitle('laser parts (red = cut). Path L uses all of them; Path H uses only the H panel', fontsize=11)
plt.tight_layout(); plt.savefig(os.path.join(DOCS, 'fig_v2_sheet_parts.png'), dpi=75); plt.close()
print('ok')
