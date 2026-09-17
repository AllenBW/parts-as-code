#!/usr/bin/env python3
"""One PNG with every STL in the project, rendered with the same painter renderer."""
import os, glob, math, numpy as np, matplotlib
matplotlib.use('Agg'); import matplotlib.pyplot as plt
from matplotlib.collections import PolyCollection
import trimesh, build_v2 as B
from render_v2 import project, PRINT, SHEET
STL = B.OUT_STL; DOCS = os.path.join(os.path.dirname(__file__), '..', 'docs')
files = sorted(f for f in glob.glob(os.path.join(STL, '*.stl')) if not os.path.basename(f).startswith('v1_') and 'h230' not in f)
n = len(files); cols = 5; rows = math.ceil(n / cols)
fig, axes = plt.subplots(rows, cols, figsize=(4.2 * cols, 4.4 * rows))
for ax, f in zip(axes.flat, files):
    m = trimesh.load(f); name = os.path.basename(f)[:-4]
    col = SHEET if name.startswith('v2L') else PRINT
    polys, cs = project([(m, col)], 35, 24)
    ax.add_collection(PolyCollection(polys, facecolors=cs, edgecolors='none', antialiased=False))
    ax.autoscale(); ax.set_aspect('equal'); ax.set_axis_off()
    e = m.extents
    ax.set_title('%s\n%.0f x %.0f x %.0f mm, %.0f cm³' % (name, e[0], e[1], e[2], m.volume / 1000), fontsize=9)
for ax in axes.flat[n:]: ax.set_axis_off()
fig.suptitle('moss-pole v2 STLs (tan = print, dark = laminated sheet stack, for reference only)', fontsize=12)
plt.tight_layout(); plt.savefig(os.path.join(DOCS, 'fig_v2_stl_gallery.png'), dpi=75)
print('%d parts' % n)
