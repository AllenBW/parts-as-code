# projects/moss-pole — context for Claude Code

Modular, self-watering, twist-together honeycomb moss pole. Four printed
parts (spike, open body, watering body, lid) that screw together; a hybrid
variant (laser-cut lattice in a printed skeleton) kept alongside for a later
iteration. Part of parts-as-code: **the Python script is the source of truth,
the STL is a build artifact, and a verifier interrogates the exported STL.**

## Layout (matches the repo convention)

```
src/build_print.py    the four print-only parts (start here)
src/verify_print.py   32 measured joints + printability + envelopes + assembled boolean intersection
src/build_hybrid.py   laser-lattice variant (cage, top ring, cap, stake ring, spike, tube, lid, panel DXF/SVG, sheet nest)
src/verify_hybrid.py  37 joints + printability + nest
src/lib.py            shared trimesh/shapely helpers (imported as `lib`)
src/coupons.py        thread coupon pair + slot gauge (test prints)
src/render_print.py   section drawings (matplotlib)
src/render_gallery.py LinkedIn-style galleries (pyrender + OSMesa; docs only, see below)
stl/                  p_* = print-only set; others = hybrid; coupon_* = test prints
laser/                hybrid panel + 12x24 sheet nest (DXF + SVG)
docs/                 galleries, section figures
archive/exploration/  v1/v2 history; not maintained, not in the README
```

## Commands

```bash
cd projects/moss-pole/src
pip install trimesh shapely manifold3d numpy scipy networkx rtree ezdxf
python build_print.py && python verify_print.py                      # 165 mm bodies (LulzBot Mini 2)
MOSS_BODY_H=230 python build_print.py && MOSS_BODY_H=230 python verify_print.py --taz   # TAZ 2
python build_hybrid.py && python verify_hybrid.py                    # hybrid, 154 mm modules
python coupons.py
python render_print.py
# galleries only: needs libOSMesa on LD_LIBRARY_PATH and PYOPENGL_PLATFORM=osmesa, plus pyrender
```

Every verifier exits 1 on any failure. `build && verify` gates every slice.
Both verifiers are green at both heights as of this handoff.

## Design decisions already made (don't relitigate without a reason)

- 100 mm across, 165 mm bodies on the Mini 2 (175 printed with the neck), 230 on the TAZ 2.
- Designed for 0.5 nozzle / 0.3 mm layers / no supports: every shoulder is a 45° cone inside and out,
  socket ledge is a cone, hex openings are flat-top (12 mm bridges), watering-body floor is a 45° funnel.
- Thread: 5 mm pitch, 1.5 deep, true 45° flanks, 1.25 turns, 0.45 radial clearance; ridge fully inside its collar.
- **Nested-cone joint**: 3 mm flat rim + 45° cone on every body top, matching recess on every bottom and the lid.
  Module pitch = BODY_H − CONE_H (rim seats on rim). The female groove is PHASED to the male ridge so
  "thread home" = "rim seated" (`female_cut` starts the groove half a pitch lower AND half a turn earlier).
- Honeycomb: 21 mm openings, 4.5 mm struts, column count forced EVEN (odd breaks the pattern at the seam).
  Radial cutters thin the struts at the bore: verify asserts ≥ 3.4 mm inside, ~4.6 outside.
- Watering body: hexes recessed 1.2 mm (not cut), ~750 ml, 6 mm wick hole; the wick sets the drip rate. No pinhole.
- Spike: solid fins, hollow hub channel with side holes, 20 mm collar opening; prints tip-up.
- Lid prints plate-down. Bodies print socket-down.
- Material: Chroma Strand INOVA-1800 (copolyester, PETG-class); glue stick on PEI, 240/60 °C, brim.

## Verifier philosophy

Measure on BOTH sides of a joint from the built geometry, never from the parameters.
The test that has earned its keep: place each mated pair at the seated position and
boolean-intersect them; the answer must be 0 mm³. It caught a thread-phase bug that
every radial measurement had passed. When a bug slips through, the fix lands twice:
the geometry, and a new assertion that would have caught it.

## Open items / what's next

1. Print `coupon_thread_male/female` first; tune `TH_CLR` (bind → raise, rattle → lower).
2. Print `p_lid`, then one `p_body_open_h165`, then `p_spike`, `p_body_water_h165`.
3. After physical prints: record the measured thread fit and the honeycomb strut feel in this file.
4. Hybrid iteration (laser lattice) waits until the print-only set is in service.
5. Possible: parametrize OD so the thread scales with it (a 3.5" / Small variant).

## Style

Match the repo's other project READMEs. No references to any commercial product.
Galleries follow the collage style (hero + tiles, rounded outer corners, off-white field,
serif lead line, grey sans, one accent rule).
