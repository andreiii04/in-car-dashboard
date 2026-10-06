# Car Dashboard — Bottom Case Tray (FreeCAD 1.1.3, macOS)

Status: **bottom tray printed and test-fitted (Oct 2026).** The board drops
in and is held snugly by the walls alone, sits flat, and the USB-C cable seats.
Not yet screwed down; on the car's dash since 4 Oct 2026, front edge forward,
~43° nose up. Final object: `Fillet001`. Next up: standoffs, then design the
top case.

**Files** (in `hardware/case/`): `bottom_tray.FCStd` is the FreeCAD source;
`bottom_tray.3mf` and `bottom_tray.stl` are the meshes exported from it per
section 8. Inside the FreeCAD file the imported board is still called
`IN_CAR_DASHBOARD 1` — the KiCad project's old name.

---

## 1. Context

Enclosure for the STM32F446RCT6 in-car telemetry dashboard PCB. This note
covers the **bottom tray only** — the part the PCB bolts into. The top case
(display bezel + SD-card access) comes later.

Everything is modelled in the **Part** workbench with primitives and booleans
— no Sketcher, no constraints. Every feature is a Cube or Cylinder with typed
Placement values. The geometry is all rectangular/cylindrical, so this keeps
every dimension explicit and easy to trace back to a number in this file.

---

## 2. Coordinate system

The KiCad STEP export (`IN_CAR_DASHBOARD 1`) is placed so the board's
bottom-left corner sits on the origin. **All case geometry is dimensioned in
the PCB's own frame — never move the STEP.**

| Reference | Value |
|---|---|
| Board outline | 61.60 (X) × 95.80 (Y) |
| Board occupies | X: 0 → +61.60, Y: 0 → −95.80 |
| Board bottom face | z = 0 |
| Board top face | z = +1.52 |
| Deepest bottom-side protrusion | z = −1.00 (clipped THT pin tips) |

Wall naming used below: **X = 0 / X = 61.6** are the long walls, **Y = 0**
(BME680 end) and **Y = −95.8** (coin-cell end) are the short walls.

The Top view renders rotated 90° (+X up-screen, +Y left), so the board looks
landscape even though it's portrait.

---

## 3. Vertical stack

All z values are in the PCB frame, so board thickness is already included.

| Feature | z |
|---|---|
| Floor underside | −6.00 |
| Cavity floor / boss base | −4.00 |
| Boss top / board underside | 0.00 |
| Board top face | +1.52 |
| Wall top | **+9.00** (7.48 above the board top) |
| Display PCB underside (10 mm standoff) | ≈ +11.52 |

- Floor 2.0 mm, walls 2.0 mm, inside wall height 13.0 mm
- Bosses 4.0 mm tall → 3.0 mm clearance under the clipped pin tips
- 2.5 mm gap between the wall top and the display PCB
- External envelope: **67.60 × 101.80 × 15.00 mm**
- Edges broken for handling: 2.0 mm fillet on the 4 vertical corners, 0.8 mm
  on the 4 bottom edges. Top edges left sharp — the top case mates there.

---

## 4. Mounting

Ø3.20 PCB holes (M3 clearance). The pattern matches the MSP2807 display
exactly: **44.00 × 76.08**, X-centred at (61.6 − 44)/2 = 8.80.

| | X | Y |
|---|---|---|
| H1 | 8.80 | −12.80 |
| H2 | 52.80 | −12.80 |
| H3 | 8.80 | −88.88 |
| H4 | 52.80 | −88.88 |

Y values were probed directly off the STEP (hole edges at 12 and 6 o'clock,
averaged) and cross-checked against the datasheet's 76.08 span and 6.92
bottom offset.

Bosses are Ø6.5, with a Ø2.5 pilot through boss + floor. That gives 6 mm of
thread engagement for the standoff stud.

**Fastening stack:**
- **Board → tray:** M3 male–female hex standoff. The 6 mm male stud
  thread-forms into the boss pilot and the standoff shoulder clamps the board.
- **Display → standoff:** M3×6 pan head from above, into the standoff's
  female end.
- Standoff body ~10 mm (10+6 variant); still to be confirmed against the
  MSP2807's back-side clearance.
- Hand-tighten only. Thread-formed plastic tolerates about 3–4 insertion
  cycles.

---

## 5. USB-C cutout

**Connector:** GCT USB4930-00-A (the STEP model is `usb4930_00_a`). The
supplier's drawing link points to the USB4940 — the measured model height
matches the USB4940 drawing.

- Mouth flush with the board edge at X = 61.6
- Centre measured at **Y = −48.10, z = 3.25**
- Shell top at z ≈ 4.9

**Why the hole is stepped.** The mouth sits 3.0 mm behind the outer wall face
(1 mm gap + 2 mm wall). The datasheet only guarantees ≥ 1.85 mm between the
receptacle face and the plug body when fully mated, so the cable's
plug body (the moulded part) has to enter the wall by ~1.2 mm. A hole sized
for the receptacle alone would stop the plug short of full insertion. A
pure funnel fails the same way — it narrows faster than the plug body needs.

The solution is a counterbore for the plug body, a smaller through-hole for
the metal plug, and a small lead-in chamfer between them. All three are
stadium shapes (straight sides, round ends), concentric on the receptacle.

| Feature | Size | X range | z range |
|---|---|---|---|
| Pocket (plug body) | 12.50 × 6.75, r 3.375 | 63.2 → 64.6 (1.4 deep) | −0.125 → 6.625 |
| Through-hole (metal plug) | 9.80 × 4.00, r 2.0 | 62.6 → 63.2 | 1.25 → 5.25 |
| Lead-in chamfer | 0.5 mm equal distance | through-hole rim at X = 63.2 | — |

- 2.375 mm of wall above the pocket, 3.75 mm above the through-hole
- 0.6 mm web between the pocket floor and the inner wall face
- The pocket is sized tight for a slim plug body. The USB-C spec maximum is
  12.35 × 6.5, so bulky plugs may not fit; if one is tight, grow the pocket
  radius by +0.15 mm.

---

## 6. Ventilation

The BME680 sits at **(6.55, −18.65)** on the top side, close to the Y = 0 /
X = 0 corner. Vents are placed so fresh air reaches the sensor first and
leaves at the far end, away from the MCU and regulator heat.

All slots are **1.5 wide × 6.0 tall**, through the 2 mm wall, **z 2.0 → 8.0**
(1.0 mm of wall left above them), with a **3.0 mm pitch**.

| Wall | Role | Slots | Slot X centres |
|---|---|---|---|
| Y = 0 (BME end) | inlet | 5 | 0.55 / 3.55 / **6.55** / 9.55 / 12.55 — centred on the BME680 |
| Y = −95.8 (far end) | exhaust | 8 | 20.3 → 41.3 — centred on the board (30.8) |

I didn't put vents on the long X = 0 wall, to keep dust down. The Y = 0 end
will face the car's climate vents. Keep in mind that a direct air jet on the
inlet will bias the BME680 toward vent-air temperature and humidity rather
than cabin air.

---

## 7. Feature tree

```
Fillet001                       ← FINAL — 0.8 mm on the 4 bottom edges
└─ Fillet                       ← 2.0 mm on the 4 vertical corners
   └─ Chamfer                   ← 0.5 mm lead-in on USB through-hole
      └─ Cut002  (Refine = true)      ← tray minus all cutouts
         ├─ Cut001                 ← base tray with boss pilots
         │  ├─ Fusion               ← shell + bosses
         │  │  ├─ Cut
         │  │  │  ├─ Cube           outer   67.6 × 101.8 × 15.0  @ (−3, −98.8, −6)
         │  │  │  └─ Cube001        cavity  63.6 × 97.8  × 13.5  @ (−1, −96.8, −4)
         │  │  └─ Cylinder…003      bosses  r 3.25, h 4   @ holes, z −4
         │  └─ Fusion001            pilots  r 1.25, h 9   @ holes, z −7
         └─ Fusion002               ← all cutting tools
            ├─ Cube002              pocket middle   2 × 5.75 × 6.75   @ (63.2, −50.975, −0.125)
            ├─ Cylinder008/009      pocket ends     r 3.375, h 2      @ y −50.975 / −45.225, z 3.25
            ├─ Cube003              through middle  1.9 × 5.8 × 4     @ (62.1, −51.0, 1.25)
            ├─ Cylinder010/011      through ends    r 2.0, h 1.9      @ y −51.0 / −45.2, z 3.25
            ├─ Cube004…008          vent A  1.5 × 3 × 6  @ x −0.2…11.8,  y 0.5,   z 2
            └─ Cube009…016          vent C  1.5 × 3 × 6  @ x 19.55…40.55, y −99.3, z 2
```

The vertical corners and the bottom edges ended up as two separate Fillet
operations rather than one, because pressing Enter in the radius box closes
the dialog instead of just committing the value. Same result, one extra node.

USB cylinders are rotated Axis (0, 1, 0), Angle 90°, so they run along +X.

**Modelling rules I stuck to:**
- Every cutting tool overshoots the faces it cuts. Coplanar faces break
  booleans — that's also why the cavity cube pokes 0.5 mm above the wall top.
- The chamfer and the two fillets are the **last** operations, in that order.
  They're bound to edge IDs (the chamfer to Edge193/196/199/202 of `Cut002`),
  so any upstream change can renumber them and break the chain. If that
  happens, delete them and redo only those three steps.
- Bottom fillets stay small. A fillet on a bottom edge is a real overhang
  against the build plate, so anything much past 0.8 mm droops on the first
  layers. The vertical corner fillets cost nothing — that surface stays
  vertical all the way up.

---

## 8. Printing

**Export:**
1. `Fillet001` → Part → Check Geometry. Must report no errors.
2. Hide `IN_CAR_DASHBOARD 1` (select, Space).
3. Mesh Design workbench → select `Fillet001` → Meshes → Create mesh from
   shape → **Standard**, surface deviation `0.01 mm`, angular deviation `5°`.
   The fine deviation matters: the USB hole ends and the corner fillets come
   out visibly faceted at the default.
4. Select the **mesh** object → File → Export → **3MF** (preferred) or STL.
5. Also export `Fillet001` itself as **STEP** from the Part workbench, as the
   editable version to hand over alongside the mesh.

3MF stores units explicitly; STL is unitless and is occasionally read as
inches, so say "mm" when handing it over.

**Print setup:**
- Orientation: floor on the bed, walls up
- No supports — the slot tops and USB openings are short bridges (≤ 5.8 mm),
  and the round ends are self-supporting
- 0.4 nozzle, 0.2 mm layers
- ≥ 4 perimeters and ≥ 30 % infill, so the bosses are close to solid
  around the pilot holes (the standoffs thread-form into them)
- Material: PLA is fine for the fit test. For the car, use **PETG or ASA** —
  PLA starts softening around 55–60 °C, which a parked car easily exceeds.

**After the first print, check:**
- [x] USB-C cable seats fully
- [ ] Standoffs thread in without cracking the bosses — no screws fitted yet
- [x] Board sits flat with the USB mouth centred in the hole — the fit is
  tight enough to hold the board without screws, and the tray sits flat on
  the table

**First fit result (3 Oct 2026):** the board was re-calibrated in the tray, on
the same table as the original bench calibration, and those values are now the
"flat" set in `vehicle_axes.h`. The bench values were off by pitch +0.9° /
roll −1.3° because the bare board had rested on its THT solder tails, not on
its mounting holes; the tray is the better flat reference. Since 4 Oct the
tray sits on the dash and the active set is the car calibration (43.4° nose
up, 2.5° roll), described in [`vehicle_info.md`](vehicle_info.md).

---

## 9. Open items

- **Top case:** needs a cutout at the board edge for the **full-size SD card**
  (measured ~9.5 mm above the board top including board thickness), so the
  card stays removable without opening the case. It mates on the wall top at
  z = 9.0.
- **Standoff length:** confirm the 10 mm body against the MSP2807's back-side
  components before ordering.
- **Boss heights:** confirm all four read `Height = 4`, `z = −4`. Earlier, one
  looked prouder in an isometric view. Still unchecked: a proud boss tilts the
  board, and the in-tray calibration would bake that tilt in as "flat", so it
  cannot reveal one. Measure the heights once the standoffs clamp the board.

---

## 10. Reference files

- MSP2807 mechanical drawing — display PCB 50 × 86, hole pattern 44 × 76.08,
  4 × Ø3.20
- GCT USB4930 specification and USB4940 drawing — receptacle and mating depth
- BME680 datasheet — sensor position and venting

The datasheets are not in the repository; the README lists them. The board
itself is `hardware/pcb/in_car_dashboard.kicad_pcb` (KiCad STEP export).
