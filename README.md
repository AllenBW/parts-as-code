# parts-as-code

Functional 3D-printed parts where **the Python script is the source of truth
and the STL is a build artifact**. Every project ships three things:

```
projects/<name>/
├── src/build_*.py     the part, as parametric code
├── src/verify_*.py    geometric assertions against the exported STL
└── stl/               compiled outputs, regenerable at any time
```

## Why build + verify

STLs are opaque. A render can look perfect while a bolt hole is secretly a
blind pocket, a nut trap leaves 0.8 mm of wall, or a hex socket has eaten
through the face it was recessed into — all three of which happened during
development here and were caught (twice, embarrassingly, only after a
physical print). Eyeballs audit surfaces; these parts fail in their interiors.

**build** scripts construct each part from named parameters — thread sizes,
measured tube diameters, bolt spacings, relief depths — using
trimesh + shapely + manifold3d. Changing a measurement and rerunning the
script is the entire edit workflow; nothing is ever patched at the mesh level.

**verify** scripts then interrogate the *exported STL* (not the in-memory
model — the file the slicer will actually see) with the kinds of tests a
picture can't fake:

- **containment probes**: thousands of sampled points prove a bolt axis is
  clear through the full part depth, a tool-access envelope is empty, a
  bearing land is solid, a relief pocket is actually recessed
- **section slices**: exact cross-sections at chosen depths prove hole
  diameters, pocket steps, and contact footprints
- **manifold checks**: watertight, single solid, after a round-trip through
  float32 STL quantisation
- **printer-envelope checks**: fits the machine it's destined for

A verifier exits nonzero on any failure, so `build && verify` gates every
slice. When a bug slips through anyway, the fix lands in two places: the
geometry, and a new assertion that would have caught it.

## Workflow

```bash
cd projects/<name>/src
python build_*.py           # regenerate ../stl
python verify_*.py          # assert geometry; exit 1 on failure
```

Dependencies: `pip install trimesh shapely manifold3d numpy`

## Projects

| | printed on | status |
|---|---|---|
| [`nord-grand-monitor-mount`](projects/nord-grand-monitor-mount/) — post-and-head studio-monitor mount for the Nord Grand's factory bosses | LulzBot Mini 2, t-glase | v8, printing |
| [`espresso-machine-feet-iberital-lanna`](projects/espresso-machine-feet-iberital-lanna/) — bolt-through replacement feet, four outline variants | LulzBot, PETG | V14, in service |
