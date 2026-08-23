# Nord Grand Monitor Mount (v8)

Post-and-head speaker mount bolting into the Nord Grand's factory monitor
bosses (M6, rear panel); a vertical tube carries the monitor with its CG over
the post. Fits any Nord with the Monitor screw holes (Piano 4/5/6, Grand,
Stage 3 Rev B 88/HP76, Stage 4 73/88). Two printed parts per side + tube.

## Build & verify

```bash
cd src && python build_mount.py && python verify_mount.py
```

`build_mount.py` — every dimension is a named parameter: measured tube OD
(28.45), bolt spacing, stud thread (M6/M8/M10/3-8-16 table), relief depth and
land sizes, printer fit limits. `verify_mount.py` asserts 8 properties of the
exported base STL: both piano-bolt axes clear through the FULL part depth,
r9 × 26 mm driver envelopes empty, bearing lands proud, relief recessed, web
behind the relief solid.

## Parts & BOM (per side)

`v8_A_base` (+`_mirrored` — confirm handedness first), `v8_B_head`.
Tube 28.45 OD × ~135 mm · 2× M6 into the piano (measure boss depth!) ·
4× M6×55 + nyloc (tube through-bolts) · 1× M8×20 hex (captive stud) ·
neoprene on the head disc.

## Print (t-glase, 0.5 mm nozzle)

245/250 first · bed 75/80 first · 25–30 mm/s · fan off · 0.25 mm layers,
first 0.3 @ 0.65 width · 4 walls, 30 % cubic · 10 mm brim · textured PEI:
no glue, dish-soap wash, cool before flexing.

## Design notes

Piano face relieved 4 mm except annular bolt lands (top fixed, bottom capsule
following the ±2 mm slot) and a 10 mm strip — protruding chassis screws clear
the recess; `RELIEFS` adds pockets for specific obstructions. Round bore
(tube + 0.35); the two through-bolts per socket do all anti-rotation; both
sockets blind so the tube loads the floor in compression. Load on the Nord's
bosses at 4 kg: ~21 N vs ~61 N for Nord's own monitors.

History: v1–2 rod concepts (dead end) → v3 square post → v4 stud head →
v5 through-holes + flat landing → v6 relief/round/rounded → v7 trim →
v8 round head, conical skirt.
