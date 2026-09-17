#!/usr/bin/env python3
"""
First-session artifacts.

  laser/first_cut_12x24.svg / .dxf
      One nested sheet for a 305 x 610 mm (12 x 24") 3 mm sheet on the P2:
      6 x H panel, 1 x L tabbed panel, 1 x L spline, 1 x L ring_tab layer,
      a 20 mm kerf square, and a slot-width gauge (3.0..3.5 mm) for the
      friction-fit question. Red = cut, blue = engrave (labels).

  stl/coupon_slot_gauge.stl     10-minute print: six slots 3.2..3.7 mm.
                                Push a cut panel edge in; the one that grips
                                without forcing is SLOT_W.
  stl/coupon_thread_male.stl    thread test pair, ~30 min together. Screw
  stl/coupon_thread_female.stl  them; if they bind, raise TH_CLR.
"""
import os, math
import numpy as np
from shapely.geometry import box as sbox, Polygon
from shapely.affinity import translate, rotate
from shapely.ops import unary_union
import build_v2 as B
from build_moss_pole import _ex, _cyl, _hex_poly, _union, _diff, clean_export

SHEET_W, SHEET_H, GAP = 610.0, 305.0, 2.0
OUT = B.OUT_LASER


def place(items, poly, x, y, label):
    b = poly.bounds
    items.append((translate(poly, x - b[0], y - b[1]), label, (x, y + (b[3] - b[1]) + 1)))
    return x + (b[2] - b[0]) + GAP, y + (b[3] - b[1]) + GAP


def slot_gauge_2d():
    body = sbox(0, 0, 100, 25)
    cuts, labels = [], []
    for i, w in enumerate(np.arange(3.0, 3.55, 0.1)):
        x = 8 + i * 15
        cuts.append(sbox(x - w / 2, 12, x + w / 2, 26))
        labels.append(('%.1f' % w, (x - 4, 4)))
    return body.difference(unary_union(cuts)), labels


def write_svg(items, extra_text, path):
    out = ['<svg xmlns="http://www.w3.org/2000/svg" width="%.1fmm" height="%.1fmm" viewBox="0 0 %.1f %.1f">'
           % (SHEET_W, SHEET_H, SHEET_W, SHEET_H)]
    out.append('<rect x="0" y="0" width="%.1f" height="%.1f" fill="none" stroke="#999" stroke-width="0.2" stroke-dasharray="4 4"/>'
               % (SHEET_W, SHEET_H))
    for poly, label, (lx, ly) in items:
        for ring in [poly.exterior] + list(poly.interiors):
            d = 'M ' + ' L '.join('%.3f %.3f' % (x, SHEET_H - y) for x, y in list(ring.coords)[:-1]) + ' Z'
            out.append('<path d="%s" fill="none" stroke="#ff0000" stroke-width="0.1"/>' % d)
        out.append('<text x="%.1f" y="%.1f" font-size="4" fill="#0000ff" font-family="sans-serif">%s</text>'
                   % (lx, SHEET_H - ly, label))
    for t, (x, y) in extra_text:
        out.append('<text x="%.1f" y="%.1f" font-size="3" fill="#0000ff" font-family="sans-serif">%s</text>' % (x, SHEET_H - y, t))
    out.append('</svg>')
    open(path, 'w').write('\n'.join(out))


def write_dxf(items, path):
    import ezdxf
    doc = ezdxf.new('R2010'); doc.units = ezdxf.units.MM; msp = doc.modelspace()
    doc.layers.add('CUT', color=1); doc.layers.add('ENGRAVE', color=5)
    for poly, label, (lx, ly) in items:
        for ring in [poly.exterior] + list(poly.interiors):
            msp.add_lwpolyline(list(ring.coords)[:-1], close=True, dxfattribs={'layer': 'CUT'})
        msp.add_text(label, dxfattribs={'layer': 'ENGRAVE', 'height': 4}).set_placement((lx, ly))
    doc.saveas(path)


def nest_check(items, name):
    """Every part inside the sheet, no two parts overlapping."""
    ok = True
    for p, l, _ in items:
        b = p.bounds
        if b[0] < -1e-6 or b[1] < -1e-6 or b[2] > SHEET_W + 1e-6 or b[3] > SHEET_H + 1e-6:
            print('  NEST FAIL %s: %s outside the sheet %s' % (name, l, np.round(b, 1))); ok = False
    for i in range(len(items)):
        for j in range(i + 1, len(items)):
            if items[i][0].buffer(-0.05).intersects(items[j][0].buffer(-0.05)):
                print('  NEST FAIL %s: %s overlaps %s' % (name, items[i][1], items[j][1])); ok = False
    return ok


def finish_sheet(items, extra, name):
    write_svg(items, extra, os.path.join(OUT, name + '.svg'))
    write_dxf(items, os.path.join(OUT, name + '.dxf'))
    used = max(p.bounds[2] for p, _, _ in items), max(p.bounds[3] for p, _, _ in items)
    print('  %-28s %2d parts, uses %.0f x %.0f of %.0f x %.0f mm  nest %s' % (
        name + '.svg/.dxf', len(items), used[0], used[1], SHEET_W, SHEET_H, 'OK' if nest_check(items, name) else 'FAIL'))
    return items


def gauges(items, x, y):
    place(items, sbox(0, 0, 20, 20), x, y, 'kerf 20.0')
    gauge, labels = slot_gauge_2d()
    place(items, gauge, x, y + 24, 'slot gauge (panel edge in)')
    b = items[-1][0].bounds
    return [(t, (b[0] + lx, b[1] + ly)) for t, (lx, ly) in labels]


def sheet_H():
    """Path H first cut: six H panels + gauges. Everything else on H is printed."""
    items = []
    pH, _, hH = B.panel_H()
    ph = rotate(B.kerf(pH), 90, origin=(0, 0))
    for r in range(2):
        for c in range(3):
            place(items, ph, c * (hH + GAP), r * (B.panel_width() + GAP), 'H panel')
    extra = gauges(items, 0, 2 * (B.panel_width() + GAP) + 4)
    extra.append(('moss-pole v2  PATH H first cut  |  3 mm sheet  |  red = cut, blue = engrave', (2, SHEET_H - 4)))
    return finish_sheet(items, extra, 'first_cut_H_12x24')


def sheet_L(which):
    """Path L: sheet 1 = six tabbed panels, six splines, bottom ring layers, gauges.
    sheet 2 = top ring layers (+ spare panels). Together: one complete module."""
    items = []
    lay = B.L_layers(); pL, _, hL = B.panel_L()
    pl = rotate(B.kerf(pL), 90, origin=(0, 0))
    y = 0.0
    if which == 1:
        for r in range(2):
            for c in range(3):
                place(items, pl, c * (hL + GAP), r * (B.panel_width() + GAP), 'L panel')
        y = 2 * (B.panel_width() + GAP) + 2
        layers = ['ring_socket', 'ring_socket', 'ring_spider', 'ring_tab', 'ring_tab']
        title = 'bottom ring: socket, socket, spider, tab, tab (stack in this order, glue at cut time)'
    else:
        for c in range(3):
            place(items, pl, c * (hL + GAP), 0, 'L panel (spare)')
        y = B.panel_width() + GAP + 2
        layers = ['ring_tab', 'ring_tab', 'ring_spider', 'ring_spigot', 'ring_spigot']
        title = 'top ring: tab, tab, spider, spigot, spigot (stack in this order, glue at cut time)'
    x = 0.0
    for i, n in enumerate(layers):
        x, _ = place(items, B.kerf(lay[n]), x, y, '%d %s' % (i + 1, n))
    y += 118 + 2
    spl = rotate(B.kerf(lay['spline']), 90, origin=(0, 0))
    if which == 1:
        for r in range(2):
            for c in range(3):
                place(items, spl, c * (lay['spline'].bounds[3] + 2 + GAP), y + r * (B.L_SPLINE_W + GAP), 'L spline')
        y += 2 * (B.L_SPLINE_W + GAP) + 2
    if which == 1:                       # sheet is full-height already: kerf square to the right of the panels
        place(items, sbox(0, 0, 20, 20), 3 * (hL + GAP) + 4, 0, 'kerf 20.0'); extra = []
    else:
        extra = gauges(items, 0, y)
    extra.append(('moss-pole v2  PATH L first cut, sheet %d  |  %s' % (which, title), (2, SHEET_H - 4)))
    return finish_sheet(items, extra, 'first_cut_L_sheet%d_12x24' % which)


def coupon_slot_gauge():
    blk = _ex(sbox(0, 0, 60, 14), 0, 14)
    cuts = []
    for i, w in enumerate(np.arange(3.2, 3.75, 0.1)):
        cuts.append(_ex(sbox(6 + i * 9 - w / 2, -1, 6 + i * 9 + w / 2, 15), 6, 15))
    return _diff(blk, cuts)


def coupon_threads():
    female = _diff(_ex(_hex_poly(B.AF_OUT, math.pi / 6), 0, B.TH_COLLAR + 2),
                   [B.female_thread_cut(0)])
    male = _union([_ex(_hex_poly(B.AF_OUT, math.pi / 6), 0, 3.0), B.male_thread(3.0 - 0.01)])
    male = _diff(male, [_cyl(2 * B.TH_MINOR_R - 6.0, -1, 3.0 + B.TH_COLLAR + 1)])
    return male, female


if __name__ == '__main__':
    print('laser')
    sheet_H(); sheet_L(1); sheet_L(2)
    for f in ('first_cut_12x24.svg', 'first_cut_12x24.dxf'):
        p = os.path.join(OUT, f)
        if os.path.exists(p): os.remove(p)
    print('stl coupons')
    clean_export(coupon_slot_gauge(), os.path.join(B.OUT_STL, 'coupon_slot_gauge.stl'))
    m, f = coupon_threads()
    clean_export(m, os.path.join(B.OUT_STL, 'coupon_thread_male.stl'))
    clean_export(f, os.path.join(B.OUT_STL, 'coupon_thread_female.stl'))
