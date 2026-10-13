# Car Dashboard — Enclosure (FreeCAD 1.1.4, macOS)

Two-part 3D-printed enclosure for the STM32F446RCT6 in-car telemetry
dashboard PCB (rev A): a **bottom tray** that holds the board, and a **top
lid** with a display window and an SD-card access notch.

**Status (Oct 2026):**
- Bottom tray: printed and test-fitted. The geometry was tidied after that
  print, so a reprint is pending.
- Top lid and spacer tubes: modelled and geometry-checked. Not yet printed.
- One set of four long countersunk self-tapping screws clamps the whole stack
  (lid → display → spacer tube → board → boss). No standoffs, and the only
  visible fasteners are the four screw heads flush in the lid.

**Files** (`hardware/case/`):

| File | Content |
|---|---|
| `bottom_tray.FCStd` | FreeCAD source of the tray. Final object `Fillet001`. |
| `top_case.FCStd` | FreeCAD source of the lid. Final object `TopFillet`. |
| `spacer_tube.FCStd` | FreeCAD source of the spacer tube. Final object `Spacer`. |
| `*.step` | STEP exports of the three final objects, for other CAD tools. Public. |
| `*_meshed.3mf`, `*_meshed.stl` | Print meshes, produced per section 10. Kept on disk, not in the repo. |

The KiCad STEP export of the board (`in_car_dashboard`) is imported into the
tray file for reference only. It is never moved or edited.

---

## 1. Context

Everything is modelled in the **Part** workbench with primitives (Box,
Cylinder, Cone) and booleans — no Sketcher, no constraints. Each feature is a
primitive with typed Placement values, so every dimension traces back to a
number in this file.

Exact primitives are created from the FreeCAD Python console with one-liners
such as:

```python
o=App.ActiveDocument.addObject("Part::Box","Name"); o.Length=10; o.Width=10; o.Height=5; o.Placement.Base=App.Vector(0,0,0)
```

Each verification step compares a printed volume against the value predicted
by hand (section 9).

---

## 2. Coordinate system

All case geometry is dimensioned in the **PCB's own frame**: the board's
bottom-left corner is the origin.

| Reference | Value |
|---|---|
| Board outline | 61.60 (X) × 95.80 (Y) |
| Board occupies | X: 0 → +61.60, Y: 0 → −95.80 |
| Board bottom face | z = 0 |
| Board top face | z = +1.52 |
| Deepest bottom-side protrusion | z = −1.00 (clipped THT pin tips) |

Wall naming: **X = 0** (SD side) and **X = 61.6** (USB-C side) are the long
walls; **Y = 0** (BME680 end) and **Y = −95.8** (coin-cell / antenna end) are
the short walls. Tray cavity inner faces: X = −1 / 62.6, Y = +1.0 / −96.8,
i.e. a **1.0 mm free gap** between the board edge and the wall on every side.

The Top view renders rotated 90° (+X up-screen, +Y left).

---

## 3. Vertical stack

All z values are in the PCB frame.

| Feature | z |
|---|---|
| Tray floor underside | −6.00 |
| Cavity floor / boss base | −4.00 |
| Boss top / board underside | 0.00 |
| Board top face / spacer tube bottom | +1.52 |
| Tray wall top | **+9.00** |
| Lid underside (nominal reveal ≈0.3 above the wall top) | ≈ 9.3 |
| SD card underside → top | 9.67 → 11.92 |
| Spacer tube top / display PCB underside | 12.52 |
| Display PCB top | 14.12 |
| Lid post bottom (0.30 into the display top, by design) | 13.82 |
| Lid plate underside | 18.62 |
| Screen glass top (worst case) | 18.32 |
| Lid top (when seated on the wall) | **+20.62** |

- Tray: floor 2.0, walls 2.0, inside wall height 13.0.
- Lid: plate 2.0 mm, 0.30 mm worst-case gap between glass top and plate
  underside.
- Tray envelope **67.60 × 101.80 × 15.00 mm**; lid envelope
  **67.60 × 101.80 × 11.62 mm**; assembled height ≈ 26.9 mm.
- Tray edges: **2.0 mm** fillet on the 4 vertical corners, **1.5 mm** on the
  4 bottom edges, top edges left sharp (the lid mates there).
- Lid edges: **2.0 mm** vertical corners, **1.5 mm** fillet on the outer top
  loop, **1.0 mm** chamfer on the window's top edges.

Measurements came from calipers on the assembled stack plus the display
datasheet. Cross-check: 8.15 + 8.65 = 16.80 = 11.0 + 5.8 above the board top.

**Tightest gap: 0.60 mm** between the SD-card top (11.92) and the display PCB
underside (12.52). That is the channel the card slides through, so nothing
may intrude on it.

---

## 4. Mounting and fastening

Ø3.20 PCB holes (M3 clearance). The pattern matches the MSP2807 display:
**44.00 × 76.08**, X-centred at (61.6 − 44)/2 = 8.80.

| | X | Y |
|---|---|---|
| H1 | 8.80 | −12.80 |
| H2 | 52.80 | −12.80 |
| H3 | 8.80 | −88.88 |
| H4 | 52.80 | −88.88 |

H-Y values were probed off the STEP and cross-checked against the
datasheet's 76.08 span.

**Tray:** four bosses Ø6.5 × 4.0 mm (z −4 → 0), with a Ø2.5 pilot running
through boss and floor (z −7 → +2). The board sits **on the bosses** and is
located and clamped by the screws. It does not touch the walls.

**Display:** plugs into female pin sockets on the board. The sockets hold the
display, not the board. The display's underside rests on the four spacer
tubes.

**Load path of one screw** (tightening pulls the lid down):

```
screw head (flush in lid) → lid plate → lid post (bears on display top)
  → display PCB → spacer tube (11.0 mm) → board → boss → pilot thread
```

The posts are 0.30 mm longer than the gap to the display, so with nominal
dimensions the lid stays ≈0.3 mm above the tray wall top. This forces the
load through the stack above rather than letting the lid bottom out on the
walls, and the board is actually clamped. The result is a uniform ≈0.1–0.5 mm
reveal line between lid and tray (display and print tolerances decide the
value). The screws also locate the lid laterally (Ø3.4 bore, Ø2.9 shank).

| Item | Spec |
|---|---|
| Screws | 4 × countersunk (flat-head) self-tapping, ≈ ST2.9 × 25 mm, Phillips/Pozidriv (DIN 7982 / ISO 7050 style); buy 8 |
| Head limit | Ø ≤ 6.0 mm, height ≤ 1.9 mm (countersink is Ø6.8 × 1.7) |
| Engagement | tip ends ≈ 4.6 mm below the boss top, ≈ 1.4 mm short of the floor underside → ≈ 4.5 mm in plastic |
| Pilot | Ø2.5 (suits ST2.9 in PETG/ASA) |
| Spacer tube | 4 × printed, OD 5.5, ID 3.1, length 11.0 (section 8) |

Why not M3 standoffs: M3 machine-thread studs do not form a thread in printed
bosses, and they would need exposed screws on top of the lid. A
self-tapping screw forms its own thread and keeps the lid face clean.
Thread-formed plastic tolerates only a few insertion cycles. Hand-tighten.

---

## 5. USB-C cutout

**Connector:** GCT USB4930-00-A (the STEP model is `usb4930_00_a`). The BOM's
drawing link points to the USB4940; the measured model height matches the
USB4940 drawing.

- Mouth flush with the board edge at X = 61.6
- Receptacle centre at **Y = −48.10, z = 3.25**
- Shell top at z ≈ 4.9

**One straight stadium hole, no step.** The first print used a counterbore, a
narrower through-hole and a 0.5 mm lead-in chamfer. It was dimensionally
fine, but printed badly: 0.5 mm on a 1.35 mm ledge reads as a sloppy step. A
single straight opening prints cleanly. It is also **centred 0.4 mm above the
receptacle centre** (z 3.65 vs 3.25), because the first print sat visibly
low.

| Feature | Value |
|---|---|
| Opening | 12.25 × 6.50 stadium, end radius 3.25 |
| Centre | Y = −48.10, z = **3.65** |
| z range | 0.40 → 6.90 |
| X range | 62.0 → 65.1 (overshoots both wall faces) |
| Wall left above the opening | 2.10 mm |

**Why it is this big.** The receptacle mouth sits 3.0 mm behind the outer
wall face (1 mm gap + 2 mm wall), and the datasheet only guarantees ≥ 1.85 mm
between receptacle face and plug body when mated. The cable's moulded plug
body therefore enters the wall by ~1.2 mm. A hole sized for the receptacle
alone would stop the plug short and give intermittent contact; a funnel fails
the same way. 6.50 mm is the USB-C maximum plug-body height, and printed
holes come out 0.1–0.2 mm undersize, so this fits a slim plug body. Measured
cable: plug body **10.20 × 4.55**, 6.6 long. If a print comes out tight, grow
the end radius by +0.15 mm and lift the box by the same amount.

---

## 6. Ventilation

The BME680 sits at **(6.55, −18.65)** on the top side, close to the Y = 0 /
X = 0 corner. Vents are placed so fresh air reaches the sensor first and
leaves at the far end, away from the MCU and regulator heat.

All slots: **1.5 wide × 6.0 tall**, through the 2 mm wall, **z 2.0 → 8.0**
(1.0 mm of wall above them), **3.0 mm pitch**.

| Wall | Role | Slots | Slot X centres |
|---|---|---|---|
| Y = 0 (BME end) | inlet | 5 | 0.55 / 3.55 / **6.55** / 9.55 / 12.55 |
| Y = −95.8 (far end) | exhaust | 8 | 20.3 → 41.3 (centred on the board, 30.8) |

No vents on the long X = 0 wall, to keep dust out. A direct air jet on the
inlet biases the BME680 toward vent-air temperature and humidity rather than
cabin air.

---

## 7. Tray feature tree

```
Fillet001                       ← FINAL — 1.5 mm on the 4 bottom edges
└─ Fillet                       ← 2.0 mm on the 4 vertical corners
   └─ Cut002  (Refine = true)   ← tray minus all cutouts
      ├─ Cut001                 ← base tray with boss pilots
      │  ├─ Fusion              ← shell + bosses
      │  │  ├─ Cut
      │  │  │  ├─ Cube          outer   67.6 × 101.8 × 15.0  @ (−3, −98.8, −6)
      │  │  │  └─ Cube001       cavity  63.6 × 97.8  × 13.5  @ (−1, −96.8, −4)
      │  │  └─ Cylinder…003     bosses  r 3.25, h 4   @ holes, z −4
      │  └─ Fusion001           pilots  r 1.25, h 9   @ holes, z −7
      └─ Fusion002              ← all cutting tools
         ├─ Cube002 (Box002)    USB middle   3.1 × 5.75 × 6.5  @ (62.0, −50.975, 0.4)
         ├─ Cylinder008/009     USB ends     r 3.25, h 3.1     @ y −50.975 / −45.225, z 3.65
         ├─ Cube003             (dormant)    1.9 × 5.8 × 4     @ (62.1, −51.0, 1.25)
         ├─ Cylinder010/011     (dormant)    r 2.0, h 1.9      @ y −51.0 / −45.2, z 3.25
         ├─ Cube004…008         vent A  1.5 × 3 × 6  @ x −0.2…11.8,  y 0.5,   z 2
         └─ Cube009…016         vent C  1.5 × 3 × 6  @ x 19.55…40.55, y −99.3, z 2
```

The dormant tools (`Cube003`, `Cylinder010/011`) sit entirely inside the
enlarged USB opening and do not change the geometry (checked: removing them
leaves the volume identical). They are left in place; deleting them would
mean rebuilding `Fusion002`.

All four pilot cylinders are `Height = 9` at z = −7 (one was 7 mm before; this
was cosmetic only, as its top was coplanar with the boss top).
Tray volume check: **22189.74 mm³**.

USB cylinders are rotated Axis (0, 1, 0), Angle 90°, so they run along +X.

---

## 8. Top lid and spacer tube

The lid is a cap with the same footprint as the tray. It sits on the tray
wall top at nominally z = 9.00 (see the reveal in section 4) and is aligned by
the four screws.

**Shell:** `LidOuter` 67.6 × 101.8 × 11.62 @ (−3, −98.8, 9), vertical edges
filleted r 2.0 (matches the tray), minus `LidCavity` 63.6 × 97.8 × 10.12 @
(−1, −96.8, 8.5). This leaves a 2.0 mm plate and 2.0 mm walls; the cavity is
open at the bottom.

**Display window.** The display's active area (AA) is centred on the case.

| | X | Y | Size |
|---|---|---|---|
| Display PCB | 5.80 → 55.80 | −95.80 → −9.80 | 50 × 86 |
| Glass | — | −16.20 → −85.40 | 50 × 69.2 |
| Viewing area (VA) | — | −18.25 → −77.70 | 45.2 × 59.45 |
| Active area (AA) | — | −19.10 → −76.70 | 43.2 × 57.6 |
| **Window** | 8.6 → 53.0 | −77.3 → −18.5 | **44.4 × 58.8** |

AA centre (30.80, −47.90) = case centre. The window leaves 0.6 mm on every
side of the AA and sits just inside the VA (the bezel hides the last
0.4 mm of VA per side). Top edges carry a 1.0 mm chamfer so the visible lip
is modest and the viewing angle is not cut off. Source: MSP2807 datasheet
plus H1–H4.

**SD notch** `SDnotch` 3 × 25 × 3.9 @ (−3.5, −61.4, 8.5): through the
X = 0 wall, Y −61.4 → −36.4, z 8.5 → 12.4. The card (24 mm wide) spans
Y −60.8 → −36.9, so there is ≈0.5 mm to each side. The notch is **open at
the lid's bottom edge**. The card underside (9.67) is only 0.67 mm above the
wall top (9.00); a closed slot would leave an unprintable 0.17 mm sliver. The
measured card span adds up to 23.9, not 24.0 mm; the notch tolerance covers
it.

**Posts:** four Ø5.6 × 5.18 cylinders at H1–H4, z 13.82 → 19.0. They bear on
the display PCB (section 4).

**Screw holes:** Ø3.4 bores (z 13 → 22) and Ø6.8 → Ø3.4 × 1.7 mm countersink
cones (base z 19.12) at H1–H4.

**GPS antenna keep-out.** The board has no copper keepout under the chip
antenna U6, so the antenna is re-soldered on two stubs of resistor lead about
1 cm above its pads (README, known issues). It works, but the corner is
fragile. Nothing in the case may touch it, and nothing metal or conductive
may sit near it.

| | Value |
|---|---|
| U6 footprint centre | (2.65, −81.77), long side along Y, from the KiCad placement |
| Chip on the board | X 0.65 → 4.65, Y −77.8 → −85.8 (≈4 × 8 mm) |
| Lead stubs | at the pads, (2.65, −77.8) and (2.65, −85.8) |
| Chip height | top ≈ z 11.5, **estimated, not measured** |

Clearances in the current design:

| Neighbour | Clearance |
|---|---|
| Display PCB edge (X 5.8) | 1.15 mm in X; the antenna sits beside the display, not under it |
| Spacer tube / lid post at H3 (r 2.75 / 2.8 at 8.80, −88.88) | ≈1.4 mm to the chip, ≈4 mm to the nearer stub |
| Lid wall inner face (X −1) | 1.65 mm |
| Lid plate underside (z 18.62) | ≈7 mm above the estimated chip top |
| SD card (Y −36.9 → −60.8) | well clear |

The lid is plastic, so it does not affect reception. The steel screw at H3 is
≈9 mm from the chip centre; check that the satellite count does not drop once
the case is closed.

**Print notes:** the window chamfer and the countersinks are 45° overhangs,
fine when the lid is printed top face down.

**Spacer tube** (`spacer_tube.FCStd`, final object `Spacer`, ×4): OD 5.5
(r 2.75), ID 3.1 (r 1.55), length **11.0** (board top 1.52 → display
underside 12.52). Construction: `Outer` cylinder r 2.75 × 11 minus `Inner`
cylinder r 1.55 × 13 @ z −1 (the inner tool overshoots both ends to avoid
coplanar faces). Volume check: **178.3 mm³** each. Print upright, ≥ 3
perimeters. Check against the board STEP that nothing near H1–H4 on the top
side lies inside the Ø5.5 footprint.

---

## 9. Lid feature tree

```
TopFillet                       ← FINAL — 1.5 mm on the 8 outer top edges
└─ WinChamfer                   ← 1.0 mm on the 4 window top edges
   └─ LidCut  (Refine = true)   ← lid body minus all tools
      ├─ Fusion                 ← shell + posts
      │  ├─ Cut                 ← Fillet(LidOuter, r 2.0) − LidCavity
      │  └─ Post1…Post4         r 2.8, h 5.18 @ H1–H4, z 13.82
      └─ Tools                  ← MultiFuse of the cutting tools
         ├─ Window              44.4 × 58.8 × 4 @ (8.6, −77.3, 18)
         ├─ SDnotch             3 × 25 × 3.9    @ (−3.5, −61.4, 8.5)
         ├─ Bore1…Bore4         r 1.7, h 9      @ H1–H4, z 13
         └─ Cone1…Cone4         r 1.7 → 3.4, h 1.7 @ H1–H4, z 19.12
```

Volume checks (hand prediction vs printed), first lid revision with posts
4.6 mm long: shell 20088.05 / 20088.05; with posts ≈20503.9 / 20503.81;
after tools ≈14824.3 / 14824.27; after chamfer ≈14721 / 14719.74; final
14558.83. After lengthening the posts to 5.18 mm at z 13.82 and the bores to
9 mm at z 13, the predicted final volume is **≈14594.9 mm³** (+36.1: annulus
15.55 mm² × 0.58 mm × 4). The final shape must pass `Shape.check(True)` as a
single closed solid.

**Gotchas:**
- **Label ≠ internal name.** Primitives created from the GUI are labelled
  `Cube` but named `Box`. Objects created with `addObject("Part::Box","X")`
  have Name = Label.
- Chamfer and fillet are bound to edge IDs, so they are the **last**
  operations. The edge IDs are found by geometry from the console (filter by
  z = 20.62 and position) rather than by clicking. Changing an upstream
  object can renumber them; redo those two steps if they turn red.
- A selection filter that tests an edge's centre against a window's bounds
  must use a small tolerance. Edges lying exactly on the bound fail strict
  `<` tests (that selected one edge instead of four).
- Console lines must be pasted with no leading spaces, one logical line
  each.
- Every cutting tool overshoots the faces it cuts. Coplanar faces break
  booleans.
- In the Part Fillet dialog, type the radius **before** ticking edges.

---

## 10. Printing

**Export (each part):**
1. Final object (`Fillet001` / `TopFillet` / `Spacer`) → Part → Check
   Geometry. No errors.
2. Hide the board STEP. Delete any stale mesh object.
3. Meshes workbench → select the final object → Meshes → Create mesh from
   shape → **Standard**, surface deviation `0.01 mm`, angular deviation `5°`.
4. Select the **mesh** → File → Export → **3MF** (preferred) or STL.
5. Export the final object as **STEP** from the Part workbench.

3MF stores units explicitly; STL is unitless and is occasionally read as
inches.

**Print setup:**
- Material: PLA is fine for the fit test. For the car, use **PETG or ASA**
  (PLA softens at 55–60 °C, which a parked car exceeds).
- 0.4 mm nozzle, **0.15 mm layers**, ≥ 4 perimeters, ≥ 30 % infill. Use ≥ 4
  perimeters around the screw bosses and lid posts so the self-tapping
  screws do not split them.
- **Tray:** floor on the bed, walls up, no supports.
- **Lid:** printed **top face down** (rotate 180° about X in the slicer),
  no supports.
- **Spacer tubes:** upright, no supports. Print them in the same material as
  the other parts.
- The printer is not precise; features were kept generous (1.0 mm chamfer,
  0.5 mm SD margin, 1.0 mm board gap).

**On the bottom fillet radius.** The first print used 0.8 mm and came out as a
visible staircase: at 0.2 mm layers it spans only 4 layers. 1.5 mm spreads
the curve over 8 layers, and 0.15 mm layers halve each step again. A larger
bottom radius prints better than a small one, until the overhang against the
plate gets too shallow.

**First print (tray), checked:**
- [x] USB-C cable seats fully
- [x] Board sits flat on the bosses, USB mouth centred in the hole
- [ ] Screws form clean threads in the bosses (not yet tested)

The tray was used as the "flat" reference for the board's IMU calibration
(`vehicle_axes.h`). A boss that stands proud would tilt the board and bake
that tilt into the calibration.

---

## 11. Assembly order

1. Place the board on the four bosses (USB-C end towards the USB opening).
2. Slip a spacer tube over each of the four holes on the board top.
3. Plug the display onto the female sockets until its underside rests on the
   four tubes.
4. Place the lid. The SD notch must sit over the card side (X = 0 wall).
5. Drive four countersunk self-tapping screws through the lid until the heads
   sit flush. Hand-tighten only. Expect a ≈0.3 mm reveal between lid and
   tray.

---

## 12. Open items

- **Lid and spacer print and test fit:** check the window margin, SD card
  in/out, the 0.3 mm glass gap, the reveal, and that tightening really
  clamps the board (the board must not move on the bosses).
- **GPS antenna:** measure the lifted chip's real position and top height
  with calipers and update the keep-out table in section 8. After the lid
  is fitted, compare satellites and HDOP with the case open and closed.
- **Screws:** source 8 countersunk self-tapping screws (≈ ST2.9 × 25), check
  the head against the Ø6.0 × 1.9 limit.
- **Tray reprint** with the cosmetic pilot-height fix (section 7); remove the
  stale mesh object `Fillet001 (Meshed)` from the FreeCAD file.
- **Boss heights:** all four should read `Height = 4`, `z = −4`; verify once
  the board is screwed down.
- **Vehicle mounting:** not part of rev A (the tray is taped to the dash for
  testing). It is planned for a rev B redesign.

---

## 13. Reference files

`MSP2807_Size.pdf` (display mechanical: PCB 50 × 86, hole pattern
44 × 76.08, 4 × Ø3.20, glass 50 × 69.2, stack 5.60 ± 0.2),
`USB4940.pdf` / `usb4930-spec.pdf` (USB-C receptacle and mating depth),
`SD_Physical_Layer_Spec.pdf` (card dimensions), `bme680.pdf`. The parts are
listed in the README. The board itself:
`hardware/pcb/in_car_dashboard.kicad_pcb` (KiCad STEP export).
