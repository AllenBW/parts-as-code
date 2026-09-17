#!/usr/bin/env python3
"""
Gallery compositions in the collage style: hero on top, supporting tiles
below, one stacked block with rounded OUTER corners and sharp gutters,
floating on an off-white field, serif lead line + grey sans + one accent rule.

  docs/gallery_pole.png    assembled + cut-in-half hero, the four parts below
  docs/gallery_parts.png   the four parts in a row, then the three section views

Renders: pyrender + OSMesa (LD_LIBRARY_PATH to a Mesa with libOSMesa,
PYOPENGL_PLATFORM=osmesa). Composition: PIL. Docs only.
"""
import os, math
os.environ.setdefault('PYOPENGL_PLATFORM', 'osmesa')
import numpy as np
import trimesh, pyrender
from PIL import Image, ImageDraw, ImageFont
import build_print as B

DOCS = os.path.join(os.path.dirname(__file__), '..', 'docs'); os.makedirs(DOCS, exist_ok=True)
TAG = 'h%d' % int(B.BODY_H)

# palette: one printed material family, warm, with the cut faces reading naturally
MOSS   = (0.42, 0.50, 0.36)     # open body   (sage)
CLAY   = (0.72, 0.45, 0.34)     # watering body (terracotta)
CHAR   = (0.26, 0.26, 0.28)     # spike (charcoal)
BONE   = (0.84, 0.80, 0.72)     # lid
FIELD  = (245, 243, 238)        # off-white field
TILE   = (255, 255, 255)
INK    = (28, 28, 30)
GREY   = (110, 108, 104)
ACCENT = (96, 122, 82)          # moss green rule
FONT = '/usr/share/fonts/truetype/liberation/'
RHO = 1.27


def load(n): return trimesh.load(os.path.join(B.OUT_STL, n + '.stl'))
def at(m, z=0.0, x=0.0):
    m = m.copy(); m.apply_translation([x, 0, z]); return m


def look_at(eye, target, up=(0, 0, 1)):
    eye, target, up = np.array(eye, float), np.array(target, float), np.array(up, float)
    f = target - eye; f /= np.linalg.norm(f); s = np.cross(f, up); s /= np.linalg.norm(s); u = np.cross(s, f)
    m = np.eye(4); m[:3, 0] = s; m[:3, 1] = u; m[:3, 2] = -f; m[:3, 3] = eye; return m


def render(parts, w, h, elev=20, azim=35, fov=24, pad=1.0, center=None, radius=None, bg=TILE):
    scene = pyrender.Scene(bg_color=[c / 255 for c in bg], ambient_light=[0.45, 0.45, 0.45])
    allv = np.vstack([m.vertices for m, _ in parts])
    c = allv.mean(axis=0) if center is None else np.array(center, float)
    r = np.linalg.norm(allv - c, axis=1).max() if radius is None else radius
    for m, col in parts:
        sm = trimesh.graph.smooth_shade(m, angle=math.radians(35))
        mat = pyrender.MetallicRoughnessMaterial(baseColorFactor=[*col, 1.0], metallicFactor=0.0, roughnessFactor=0.8)
        scene.add(pyrender.Mesh.from_trimesh(sm, material=mat, smooth=True))
    d = r * pad / math.sin(math.radians(fov) / 2) / min(1.0, w / h)
    e, a = math.radians(elev), math.radians(azim)
    eye = c + d * np.array([math.cos(e) * math.cos(a), math.cos(e) * math.sin(a), math.sin(e)])
    scene.add(pyrender.PerspectiveCamera(yfov=math.radians(fov), aspectRatio=w / h), pose=look_at(eye, c))
    for vec, inten in (((0.35, -0.55, 0.75), 2.4), ((-0.8, 0.3, 0.35), 1.2), ((0.1, 0.9, -0.1), 0.7)):
        scene.add(pyrender.DirectionalLight(color=np.ones(3), intensity=inten), pose=look_at(c + d * np.array(vec), c))
    rr = pyrender.OffscreenRenderer(w, h); img, _ = rr.render(scene); rr.delete()
    return Image.fromarray(img)


def half(m, normal=(0, 1, 0)):
    return trimesh.intersections.slice_mesh_plane(m, plane_normal=-np.array(normal, float), plane_origin=[0, 0, 0], cap=True)


def slab(m, y0=-12.0, y1=0.0, z0=None, z1=None):
    m = trimesh.intersections.slice_mesh_plane(m, plane_normal=[0, -1, 0], plane_origin=[0, y1, 0], cap=True)
    m = trimesh.intersections.slice_mesh_plane(m, plane_normal=[0, 1, 0], plane_origin=[0, y0, 0], cap=True)
    if z0 is not None: m = trimesh.intersections.slice_mesh_plane(m, plane_normal=[0, 0, 1], plane_origin=[0, 0, z0], cap=True)
    if z1 is not None: m = trimesh.intersections.slice_mesh_plane(m, plane_normal=[0, 0, -1], plane_origin=[0, 0, z1], cap=True)
    return m


def grams(m, eff): return m.volume / 1000 * eff * RHO


def quarter(m, z0=None, z1=None):
    """Remove the x>0, y>0 quadrant (and optionally crop z) so a section shows with the outside for context."""
    a = trimesh.intersections.slice_mesh_plane(m, plane_normal=[-1, 0, 0], plane_origin=[0, 0, 0], cap=True)     # x <= 0 half
    b = trimesh.intersections.slice_mesh_plane(m, plane_normal=[1, 0, 0], plane_origin=[0, 0, 0], cap=True)      # x >= 0 half
    b = trimesh.intersections.slice_mesh_plane(b, plane_normal=[0, -1, 0], plane_origin=[0, 0, 0], cap=True)     # ... and y <= 0
    out = trimesh.util.concatenate([a, b])
    if z0 is not None: out = trimesh.intersections.slice_mesh_plane(out, plane_normal=[0, 0, 1], plane_origin=[0, 0, z0], cap=True)
    if z1 is not None: out = trimesh.intersections.slice_mesh_plane(out, plane_normal=[0, 0, -1], plane_origin=[0, 0, z1], cap=True)
    return out


def fit(im, w, h, margin=0.09, bg=TILE):
    """Crop to content, then letterbox into w x h with a margin. Render bigger than the tile for quality."""
    a = np.asarray(im).astype(int); diff = np.abs(a - np.array(bg)).sum(axis=2) > 12
    ys, xs = np.where(diff)
    if len(xs) == 0: return im.resize((w, h), Image.LANCZOS)
    x0, x1, y0, y1 = xs.min(), xs.max(), ys.min(), ys.max()
    cw, ch = x1 - x0 + 1, y1 - y0 + 1
    crop = im.crop((x0, y0, x1 + 1, y1 + 1))
    sc = min(w * (1 - 2 * margin) / cw, h * (1 - 2 * margin) / ch)
    nw, nh = max(1, int(cw * sc)), max(1, int(ch * sc))
    crop = crop.resize((nw, nh), Image.LANCZOS)
    tile = Image.new('RGB', (w, h), bg); tile.paste(crop, ((w - nw) // 2, (h - nh) // 2 - int(h * 0.02)))
    return tile


# ---------------- composition helpers ----------------
def rounded_block(tiles, W, gutter, radius):
    """tiles: list of rows; each row a list of PIL images already sized. Returns one RGBA block with rounded outer corners."""
    H = sum(max(im.height for im in row) for row in tiles) + gutter * (len(tiles) - 1)
    block = Image.new('RGB', (W, H), FIELD); y = 0
    for row in tiles:
        x = 0
        for im in row:
            block.paste(im, (x, y)); x += im.width + gutter
        y += max(im.height for im in row) + gutter
    mask = Image.new('L', (W * 4, H * 4), 0)
    ImageDraw.Draw(mask).rounded_rectangle((0, 0, W * 4 - 1, H * 4 - 1), radius=radius * 4, fill=255)
    mask = mask.resize((W, H), Image.LANCZOS)
    out = Image.new('RGBA', (W, H), (0, 0, 0, 0)); out.paste(block, (0, 0), mask)
    return out


def label(im, text, sub=None):
    """Small caps label in the tile's lower-left, like a plate number."""
    d = ImageDraw.Draw(im)
    f = ImageFont.truetype(FONT + 'LiberationSans-Bold.ttf', 22); fs = ImageFont.truetype(FONT + 'LiberationSans-Regular.ttf', 20)
    d.text((28, im.height - 66 if sub else im.height - 44), ' '.join(text.upper()), font=f, fill=INK)
    if sub: d.text((28, im.height - 40), sub, font=fs, fill=GREY)
    return im


def caption(W, lead, subs):
    f_serif = ImageFont.truetype(FONT + 'LiberationSerif-Regular.ttf', 44)
    f_sans = ImageFont.truetype(FONT + 'LiberationSans-Regular.ttf', 26)
    H = 60 + 3 + 26 + 56 + 36 * len(subs) + 40
    band = Image.new('RGB', (W, H), FIELD); d = ImageDraw.Draw(band)
    d.rectangle((W // 2 - 32, 60, W // 2 + 32, 63), fill=ACCENT)
    y = 60 + 3 + 26; d.text(((W - d.textlength(lead, font=f_serif)) / 2, y), lead, font=f_serif, fill=INK); y += 56
    for s in subs:
        d.text(((W - d.textlength(s, font=f_sans)) / 2, y), s, font=f_sans, fill=GREY); y += 36
    return band


def page(block, cap, margin=48):
    W = block.width + 2 * margin; H = margin + block.height + cap.height + margin // 2
    pg = Image.new('RGB', (W, H), FIELD)
    pg.paste(block, (margin, margin), block); pg.paste(cap, (margin, margin + block.height))
    return pg


# ---------------- parts ----------------
bo, bw, sp, lid = load('p_body_open_' + TAG), load('p_body_water_' + TAG), load('p_spike'), load('p_lid')
P = B.BODY_H - B.CONE_H
G, R = 14, 40                                  # gutter, outer corner radius
W = 2000


def stack(g):
    parts = [(sp, CHAR)]; z = g
    parts.append((at(bo, z), MOSS)); z += P + g
    parts.append((at(bw, z), CLAY)); z += P + g
    parts.append((at(lid, z), BONE)); return parts


# ===== gallery 1: the pole =====
hw = (W - G) // 2; hh = 1180
hero_a = fit(render(stack(0), 1400, 1800, elev=14, azim=35, fov=26), hw, hh, margin=0.07)
hero_b = fit(render([(half(m), c) for m, c in stack(0)], 1400, 1800, elev=12, azim=82, fov=26), hw, hh, margin=0.07)
label(hero_a, 'assembled', 'four parts, screwed together by hand')
label(hero_b, 'cut in half', 'lid → reservoir → funnel → wick → moss → spike → soil')
tw = (W - 3 * G) // 4; th = 600
row = [fit(render([(m, c)], 1000, 1000, elev=22, azim=35, fov=24), tw, th, margin=0.12) for m, c in ((bo, MOSS), (bw, CLAY), (sp, CHAR), (lid, BONE))]
for im, (n, s) in zip(row, (('open body', '%.0f g' % grams(bo, 1)), ('watering body', '%.0f g · ~%.0f ml' % (grams(bw, 1), B.reservoir_volume_ml())),
                            ('spike', '~%.0f g' % grams(sp, 0.65)), ('lid', '~%.0f g' % grams(lid, 0.75)))):
    label(im, n, s)
block = rounded_block([[hero_a, hero_b], row], W, G, R)
cap = caption(W, 'Four parts. One thread. Water finds its own way down.',
              ['A modular, self-watering moss pole, printed in one material and screwed together by hand.',
               'Every part is a Python script; every fit is a test the exported STL has to pass.'])
page(block, cap).save(os.path.join(DOCS, 'gallery_pole.png'), quality=95)

# ===== gallery 2: the parts and their sections =====
AZ = 30; sx, sy = -math.sin(math.radians(AZ)), math.cos(math.radians(AZ))     # screen-horizontal direction for this azimuth
def along(m, t, z=0.0):
    m = m.copy(); m.apply_translation([sx * t, sy * t, z]); return m
lineup = [(along(sp, -230, 0), CHAR), (along(bo, -80, -60), MOSS), (along(bw, 80, -60), CLAY), (along(lid, 230, 10), BONE)]
hero = fit(render(lineup, 2400, 1000, elev=18, azim=AZ, fov=22), W, 820, margin=0.06)
label(hero, 'the family', 'spike · open body · watering body · lid')
sw = (W - 2 * G) // 3; sh = 660
jt = trimesh.util.concatenate([bo, at(bo, P)])
z_lo, z_hi = B.BODY_H - B.CONE_H - 4, P + B.SOCK_H + 6      # solid bands either side of the joint: no hex fragments
def sector(m, z0=None, z1=None):
    """Keep only the x<=0, y<=0 quadrant: a quarter ring whose two cut faces both face the camera at azimuth 45."""
    m = trimesh.intersections.slice_mesh_plane(m, plane_normal=[-1, 0, 0], plane_origin=[0, 0, 0], cap=True)
    m = trimesh.intersections.slice_mesh_plane(m, plane_normal=[0, -1, 0], plane_origin=[0, 0, 0], cap=True)
    if z0 is not None: m = trimesh.intersections.slice_mesh_plane(m, plane_normal=[0, 0, 1], plane_origin=[0, 0, z0], cap=True)
    if z1 is not None: m = trimesh.intersections.slice_mesh_plane(m, plane_normal=[0, 0, -1], plane_origin=[0, 0, z1], cap=True)
    return m
joint_cut = [(sector(bo, z0=z_lo), MOSS), (sector(at(bo, P), z1=z_hi), CLAY)]
sec = [fit(render(joint_cut, 1100, 1100, elev=16, azim=100, fov=20, center=[-44, 0, P], radius=34), sw, sh, margin=0.10),
       fit(render([(quarter(sp), CHAR)], 1100, 1100, elev=22, azim=45, fov=24), sw, sh, margin=0.10),
       fit(render([(quarter(bw), CLAY)], 1100, 1100, elev=20, azim=45, fov=24), sw, sh, margin=0.08)]
label(sec[0], 'the joint', 'rim on rim, cones nested, thread in phase')
label(sec[1], 'the spike', '20 mm opening, 10 mm channel, side holes')
label(sec[2], 'the reservoir', '45° funnel to a 6 mm wick hole')
block = rounded_block([[hero], sec], W, G, R)
cap = caption(W, 'Every part is a script. Every fit is a test.',
              ['Section views are the geometry the verifier probes: thread engaged, cones nested, channel open to the holes.',
               'Placed at the seated position, every mated pair intersects to zero.'])
page(block, cap).save(os.path.join(DOCS, 'gallery_parts.png'), quality=95)
print('ok')
