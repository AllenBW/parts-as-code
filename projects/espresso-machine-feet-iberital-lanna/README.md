# Espresso Machine Feet — Iberital L'Anna (2003)

Replacement feet riding the machine's 3/8-16 hex bolts: foot sits cup-down,
bolt slides up through the hub, head keys into a hex channel, shaft threads
into the machine's boss, and a perimeter-only top flange preloads against the
base panel (its open centre clears the ~8 mm protruding threaded boss).
Iterated V10→V14 against physical fit tests; the rounded-square variant is in
service on the machine.

## Build & verify

```bash
cd src && python build_feet.py ../stl && for f in ../stl/*.stl; do python verify_feet.py "$f"; done
```

`build_feet.py` is a Python port of the original V14 OpenSCAD design (same
parameters: 50 mm body / 45 round, 4 mm walls, flared base, hub Ø26, 6 ribs
clipped to the inner wall, head channel AF 15, shaft Ø10.5, 10 mm flange).
Port note: ceiling thickness was recorded as 4 mm in the V14 file and 8 mm in
the session summary; **8 is used** — verify against an in-service foot before
reprinting. `verify_feet.py` asserts: watertight single solid, bolt axis clear
counter-to-flange, hex channel open, ceiling solid around the shaft, flange
centre open, all 6 ribs present.

## Variants

`foot_v14_rounded_square` (in service) · `hex` · `square` · `round` (Ø45).
Trade-off carried from V14: the flange design is incompatible with the
original riser accessory. Print cup-opening down (as modelled), PETG, no
supports.
