#!/usr/bin/env python3
"""Ten-minute test prints that set the two fit parameters before the real parts.
  coupon_slot_gauge.stl    slots 3.2..3.7 mm: push a cut panel edge in, the one that grips is SLOT_W
  coupon_thread_*.stl      male/female pair: binds -> raise TH_CLR, rattles -> lower it"""
import os, math, numpy as np
from shapely.geometry import box as sbox
import build_hybrid as B
from lib import _ex, _cyl, _hex_poly, _union, _diff, clean_export
blk = _ex(sbox(0, 0, 60, 14), 0, 14)
clean_export(_diff(blk, [_ex(sbox(6 + i * 9 - w / 2, -1, 6 + i * 9 + w / 2, 15), 6, 15) for i, w in enumerate(np.arange(3.2, 3.75, 0.1))]),
             os.path.join(B.OUT_STL, 'coupon_slot_gauge.stl'))
clean_export(_diff(_ex(_hex_poly(B.AF_OUT, math.pi / 6), 0, B.TH_COLLAR + 2), [B.female_cut(0)]), os.path.join(B.OUT_STL, 'coupon_thread_female.stl'))
clean_export(_diff(_union([_ex(_hex_poly(B.AF_OUT, math.pi / 6), 0, 3.0), B.male_thread(2.99)]), [_cyl(2 * B.TH_MINOR_R - 6.0, -1, 3 + B.TH_COLLAR + 1)]),
             os.path.join(B.OUT_STL, 'coupon_thread_male.stl'))
