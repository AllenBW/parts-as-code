#!/usr/bin/env python3
"""
Nord Grand monitor mount, v8 -- round head.

Changes from v5, from the first physical prototype:

 1. FACE RELIEF. The plate was fully flat, so a protruding chassis screw on
    the Nord's rear panel held the whole bracket off the surface. The contact
    face is now recessed by RELIEF_D except for raised LANDS -- a pad around
    each bolt and a strip under the socket. Clamp load goes through the lands,
    and anything protruding between them is cleared. Add entries to RELIEFS
    for a specific obstruction once its position is measured.

 2. ROUNDED. Vertical edges are radiused and the top is chamfered. The whole
    body is a constant Z cross-section, so this is all done on the 2D profile.

 3. ROUND POST OPTION. POST_SHAPE = 'round' takes a 1-1/8" tube. The square
    bore keyed the post against rotation; with a round post the two M6
    through-bolts do that job, so drill them accurately and fit both.

Printing (Mini 2, 0.5 mm nozzle): both parts as exported, no supports.
Use a brim on the base -- the face relief reduces first-layer area.
"""

import numpy as np
import trimesh
from trimesh.creation import box, cylinder, extrude_polygon
from shapely.geometry import Polygon, box as sbox, Point
from shapely.ops import unary_union
from shapely.affinity import scale as sscale

# ============================================================
# CONFIG
# ============================================================

BOLT_AXIS    = 'z'
BOLT_SPACING = 40.0

POST_SHAPE   = 'round'   # 'round' | 'square'
POST_SIZE    = 28.45     # MEASURED with calipers
POST_FIT     = 0.35      # snug slip fit; through-bolts take up the rest

STUD         = 'M8'
STUDS = {'M6': (6.8, 10.4, 5.2), 'M8': (8.8, 13.4, 6.0),
         'M10': (10.8, 17.2, 7.4), '3/8-16': (10.2, 15.2, 7.0)}

BOLT_D       = 6.8
BOLT_SLOT    = 4.0
PLATE_T      = 12.0      # was 9; the relief eats 4 of it
EDGE         = 16.0
DRIVER_R     = 9.0
DRIVER_L     = 26.0
SOCK_GAP     = 16.0

RELIEF_D     = 4.0       # recess depth on the piano face
LAND_R       = 10.5     # annular land outer radius around each bolt
LAND_STRIP   = 10.0     # raised strip width at the outboard edge
RELIEFS      = []        # extra pockets: (x, z, dia, depth) on the piano face

R_OUT        = 4.0       # vertical edge radius
R_SOCK       = 6.0       # socket corner radius
R_FILLET     = 10.0      # plate-to-socket fillet
CHAMFER      = 3.0       # top chamfer
CH_SLICES    = 6

SQ_WALL      = 7.0
SOCK_H       = 76.0
SOCK_FLOOR   = 8.0
SOCK_MERGE   = 3.0
XB_D         = 6.8

HEAD_PL      = 95.0    # shrunk -- gussets + socket carry the stiffness
HEAD_PT      = 9.0
HEAD_SOCK_H  = 55.0
GUSSET       = 24.0

SEG          = 96
BORE         = POST_SIZE + POST_FIT
SQ_OUT       = BORE + 2 * SQ_WALL


# ------------------------------------------------------------
def _bore_poly(cx, cy):
    if POST_SHAPE == 'round':
        return Point(cx, cy).buffer(BORE / 2, quad_segs=48)
    return sbox(cx - BORE / 2, cy - BORE / 2, cx + BORE / 2, cy + BORE / 2)


def _sock_poly(cx, cy):
    s = sbox(cx - SQ_OUT / 2, cy - SQ_OUT / 2, cx + SQ_OUT / 2, cy + SQ_OUT / 2)
    return s.buffer(-R_SOCK, quad_segs=32).buffer(R_SOCK, quad_segs=32)


def _extrude(poly, z0, z1):
    m = extrude_polygon(poly, height=z1 - z0)
    m.apply_translation([0, 0, z0])
    return m


def _chamfered(poly, z0, z1, ch=CHAMFER, n=CH_SLICES):
    """Body with a chamfered top -- cross-section shrinks going up, so it
    prints without support."""
    parts = [_extrude(poly, z0, z1 - ch)]
    dz = ch / n
    for i in range(n):
        p = poly.buffer(-(i + 0.5) * ch / n, quad_segs=24)
        if p.is_empty:
            break
        parts.append(_extrude(p if p.geom_type == 'Polygon' else max(p.geoms, key=lambda g: g.area),
                              z1 - ch + i * dz, z1 - ch + (i + 1) * dz))
    return trimesh.boolean.union(parts, engine='manifold')


def _cyl_axis(r, h, centre, axis):
    c = cylinder(radius=r, height=h, sections=SEG)
    if axis == 'x':
        c.apply_transform(trimesh.transformations.rotation_matrix(np.pi / 2, [0, 1, 0]))
    elif axis == 'y':
        c.apply_transform(trimesh.transformations.rotation_matrix(np.pi / 2, [1, 0, 0]))
    c.apply_translation(centre)
    return c


def _box(x0, x1, y0, y1, z0, z1):
    b = box(extents=[x1 - x0, y1 - y0, z1 - z0])
    b.apply_translation([(x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2])
    return b


def _hex(af, length, centre, axis='z'):
    r = af / np.sqrt(3.0)
    ang = np.linspace(0, 2 * np.pi, 7)[:6] + np.pi / 6
    p = extrude_polygon(Polygon(np.column_stack([r * np.cos(ang), r * np.sin(ang)])),
                        height=length)
    p.apply_translation([0, 0, -length / 2])
    if axis == 'x':
        p.apply_transform(trimesh.transformations.rotation_matrix(np.pi / 2, [0, 1, 0]))
    p.apply_translation(centre)
    return p


def _bolt_cut(bx, bz, d, y0, y1, ext, axis):
    ex = ext if axis == 'x' else 0.0
    ez = ext if axis == 'z' else 0.0
    parts = [_cyl_axis(d / 2, y1 - y0, [bx + ox, (y0 + y1) / 2, bz + oz], 'y')
             for ox, oz in ((-ex / 2, -ez / 2), (ex / 2, ez / 2))]
    if ext > 0:
        parts.append(_box(bx - ex / 2, bx + ex / 2, y0, y1,
                          bz - ez / 2 - d / 2, bz + ez / 2 + d / 2) if axis == 'x'
                     else _box(bx - d / 2, bx + d / 2, y0, y1, bz - ez / 2, bz + ez / 2))
    return parts


def _face_relief(x0, x1, plate_h, bolts, land_x0):
    """Recess the piano face, keeping raised lands. Built in XZ, pushed in Y.
    The slotted (bottom) hole gets a capsule land covering the slot travel."""
    from shapely.geometry import LineString
    face = sbox(x0, 0, x1, plate_h)
    lands = []
    for i, (bx, bz) in enumerate(bolts):
        if i == 0 and BOLT_SLOT > 0:   # slotted hole -- land follows the travel
            a = BOLT_SLOT / 2
            seg = LineString([(bx, bz - a), (bx, bz + a)] if BOLT_AXIS == 'z'
                             else [(bx - a, bz), (bx + a, bz)])
            lands.append(seg.buffer(LAND_R, quad_segs=32))
        else:
            lands.append(Point(bx, bz).buffer(LAND_R, quad_segs=32))
    lands.append(sbox(land_x0, 0, x1, plate_h))
    region = face.difference(unary_union(lands))
    if region.is_empty:
        return []
    geoms = region.geoms if region.geom_type == 'MultiPolygon' else [region]
    out = []
    for g in geoms:
        # rot -90 about X maps local (x,y,z) -> world (x, z, -y), so pre-mirror
        # the profile's second coord or the recess lands at negative Z.
        m = extrude_polygon(sscale(g, xfact=1, yfact=-1, origin=(0, 0)),
                            height=RELIEF_D + 1.0)
        m.apply_transform(trimesh.transformations.rotation_matrix(-np.pi / 2, [1, 0, 0]))
        m.apply_translation([0, -1.0, 0])
        out.append(m)
    for (rx, rz, rd, rdep) in RELIEFS:
        out.append(_cyl_axis(rd / 2, rdep + 1, [rx, (rdep + 1) / 2 - 1, rz], 'y'))
    return out


# ------------------------------------------------------------
def build_base():
    s = BOLT_SPACING
    bolts = ([(0.0, -s / 2), (0.0, s / 2)] if BOLT_AXIS == 'z'
             else [(-s / 2, 0.0), (s / 2, 0.0)])
    bxs, bzs = [b[0] for b in bolts], [b[1] for b in bolts]
    plate_h = (max(bzs) - min(bzs)) + 2 * EDGE
    bolts = [(bx, bz + plate_h / 2) for bx, bz in bolts]

    sx = max(bxs) + SOCK_GAP + SQ_OUT / 2
    sy = PLATE_T + SQ_OUT / 2 - SOCK_MERGE
    x0, x1 = min(bxs) - EDGE, sx + SQ_OUT / 2

    prof = unary_union([sbox(x0, 0, x1, PLATE_T), _sock_poly(sx, sy)])
    prof = prof.buffer(R_FILLET, quad_segs=32).buffer(-R_FILLET, quad_segs=32)   # fillet inside
    prof = prof.buffer(-R_OUT, quad_segs=32).buffer(R_OUT, quad_segs=32)         # round outside
    if prof.geom_type == 'MultiPolygon':
        prof = max(prof.geoms, key=lambda g: g.area)

    body = _chamfered(prof, 0.0, plate_h)

    cuts = [_extrude(_bore_poly(sx, sy), SOCK_FLOOR, plate_h + 6)]
    for zb in (32.0, 64.0):
        cuts.append(_cyl_axis(XB_D / 2, SQ_OUT + 16, [sx, sy, zb], 'x'))
    for i, (bx, bz) in enumerate(bolts):
        cuts += _bolt_cut(bx, bz, BOLT_D, -2, PLATE_T + 2,
                          BOLT_SLOT if i == 0 else 0.0, BOLT_AXIS)
        cuts.append(_cyl_axis(DRIVER_R, DRIVER_L, [bx, PLATE_T + DRIVER_L / 2, bz], 'y'))
    cuts += _face_relief(x0, x1, plate_h, bolts, sx - SQ_OUT / 2 + LAND_STRIP)

    return trimesh.boolean.difference([body] + cuts, engine='manifold')


def build_head():
    """Round head: circular plate, conical skirt instead of gussets.
    Printed inverted -- speaker face on the bed, socket rising. The skirt's
    outer surface leans inward going up (a pyramid, not an overhang), so it
    prints support-free."""
    clear, af, pocket = STUDS[STUD]
    R_PLATE = 46.0          # disc radius (92 dia)
    T_PLATE = 7.0
    SKIRT_H = 20.0
    R_SK0   = 33.0          # skirt radius where it meets the plate

    plate = cylinder(radius=R_PLATE, height=T_PLATE, sections=SEG)
    plate.apply_translation([0, 0, T_PLATE / 2])
    # small edge chamfer for elegance: stack a slightly larger disc below top
    ch = cylinder(radius=R_PLATE - 2.0, height=1.6, sections=SEG)
    ch.apply_translation([0, 0, T_PLATE + 0.8 - 0.001])

    skirt = trimesh.creation.cone(radius=R_SK0, height=SKIRT_H * (R_SK0 / (R_SK0 - SQ_OUT / 2 + 0.01)), sections=SEG)
    skirt.apply_translation([0, 0, T_PLATE])

    sockm = _chamfered(_sock_poly(0, 0), T_PLATE, HEAD_PT + HEAD_SOCK_H, ch=2.0, n=4)

    solid = trimesh.boolean.union([plate, ch, skirt, sockm], engine='manifold')
    # trim the cone back to the socket height envelope
    solid = trimesh.boolean.intersection(
        [solid, _box(-R_PLATE - 2, R_PLATE + 2, -R_PLATE - 2, R_PLATE + 2,
                     0, HEAD_PT + HEAD_SOCK_H)], engine='manifold')

    FACE_WALL = 3.0     # solid plate between speaker face and hex pocket
    cuts = [_extrude(_bore_poly(0, 0), T_PLATE, HEAD_PT + HEAD_SOCK_H + 6),
            _cyl_axis(clear / 2, FACE_WALL + 4, [0, 0, FACE_WALL / 2 - 1], 'z'),
            _hex(af, (T_PLATE - FACE_WALL) * 2,
                 [0, 0, T_PLATE], 'z')]
    for zb in (HEAD_PT + 20.0, HEAD_PT + 42.0):
        cuts.append(_cyl_axis(XB_D / 2, SQ_OUT + 16, [0, 0, zb], 'x'))
    return trimesh.boolean.difference([solid] + cuts, engine='manifold')


def build_fit_test():
    body = _chamfered(_sock_poly(0, 0), 0.0, 40.0, ch=2.0, n=4)
    cuts = [_extrude(_bore_poly(0, 0), 8.0, 46.0),
            _cyl_axis(XB_D / 2, SQ_OUT + 16, [0, 0, 26.0], 'x')]
    return trimesh.boolean.difference([body] + cuts, engine='manifold')


def clean_export(mesh, path):
    m = mesh.copy()
    m.vertices = np.round(m.vertices, 4)
    m.merge_vertices()
    m.update_faces(m.nondegenerate_faces())
    m.update_faces(m.unique_faces())
    m.remove_unreferenced_vertices()
    m.fix_normals()
    m.apply_translation([0, 0, -m.bounds[0][2]])
    m.export(path)
    chk = trimesh.load(path); chk.process(validate=True); chk.merge_vertices()
    return chk.is_watertight, chk.volume / 1000, chk.bounds[1] - chk.bounds[0]


if __name__ == '__main__':
    out = '../stl/'
    print('post = %s %.2f mm   bore = %.2f   socket OD = %.1f\n'
          % (POST_SHAPE, POST_SIZE, BORE, SQ_OUT))
    for name, mesh in [('v8_A_base', build_base()),
                       ('v8_B_head', build_head())]:
        wt, vol, d = clean_export(mesh, out + name + '.stl')
        fits = d[0] <= 160 and d[1] <= 160 and d[2] <= 180
        print('%-20s %6.1f x %5.1f x %5.1f mm  %6.1f cm3  watertight=%s  Mini2=%s'
              % (name, d[0], d[1], d[2], vol, wt, 'OK' if fits else 'TOO BIG'))
        if name == 'v8_A_base':
            m2 = trimesh.load(out + name + '.stl')
            m2.apply_transform(np.diag([-1.0, 1, 1, 1])); m2.fix_normals()
            clean_export(m2, out + name + '_mirrored.stl')
