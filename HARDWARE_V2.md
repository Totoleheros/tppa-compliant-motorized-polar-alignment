# Polar Align System – Hardware Setup (V2)

> 🔧 **This document covers the V2 hardware revision**, which uses a custom CNC ALT bielle mechanism and an RU42 crossed roller bearing for the AZM axis. This configuration is **currently under fabrication** — not yet field-validated.
>
>Use `PolarAlign_V2.ino` for this hardware.
>  
> For a proven, ready-to-build starting point, see [`HARDWARE_Prototype.md`](./HARDWARE_Prototype.md). 

---

## ⚠️ Disclaimer

> I'm just an enthusiast sharing this open hardware project, **with no guarantee of success**.
> I'll do my best to support others trying this build, but my **time is limited**, and my **skills are not professional-grade**.
> V2 is a **work in progress** — design validated in simulation, fabrication in progress. Field results and corrections will be published here once complete.

---

## 🧭 What's Different in V2?

V2 addresses the two main limitations of the Prototype:

| Axis | Prototype | V2 | Benefit |
|------|-----------|-----|---------|
| **AZM bearing** | igus PRT-02 LC — tilting moment unknown (⚠️ assumed weakest link) | **RU42 crossed roller** — tilting moment ~153 Nm (4× margin at 25 kg) | Known, published margins. No more uncertainty. |
| **ALT mechanism** | Commercial tilt plate (5.6 mm non-standard shaft, custom coupler) | **Custom CNC ALT bielle** — pivot below T8, optimized kinematics | Full 12° travel range, clean geometry, no off-spec shafts. |
| **Base plate** | Monolithic 15180 aluminium profiles | **Two-piece CNC aluminium** — assembly tolerance on 4 pillars | Adjustable at assembly, lower JLCPCB cost. |

All electronics, firmware logic, MPU wiring, and GRBL protocol are **identical to the Prototype**. Only the hardware constants change (see [`README.md` — Configuration Knobs](./README.md)).

---

## 🧩 3D Model & Files

- The full 3D design (STEP format) is included in the downloadable archive:
  📦 `PolarALIGN_V2_STEP.zip`

> 💡 All CNC part dimensions are directly readable from the STEP file. The sections below describe the architecture and design rationale; refer to the STEP for fabrication tolerances and exact dimensions.

- Parts in **green** in the model: CNC aluminium (JLCPCB recommended).
- Parts in **blue**: 3D-printed (PLA+CF or PETG).

---

## ⚖️ Mechanical Design — Load Path

### AZM Axis (Azimuth)

```
Telescope + EQ Mount
       ↓
  ALT CNC mechanism (tilt plate + bielle)
       ↓
  Two-piece CNC base plate
       ↓
  RU42 crossed roller bearing (inner ring fixed, outer ring rotates)
       ↓
  MiniF harmonic drive 100:1 (drives outer ring via 3D-printed adapter)
       ↓
  Angeleyes 400 tripod extension / pier
```

**Every component in the load path is metal.** The only 3D-printed part in this chain is the harmonic drive adapter (motor torque only — no telescope load).

---

## 🔩 AZM Axis: RU42 Crossed Roller Bearing

### Why crossed roller instead of slewing ring?

The igus PRT-02 LC (Prototype) is a polymer slewing ring with unpublished tilting moment for its LC variant. This was the single biggest uncertainty in the Prototype's payload analysis. The RU42 replaces it with a precision steel crossed roller bearing with known, published ratings.

| Parameter | igus PRT-02 LC | RU42 (V2) |
|---|---|---|
| Type | Polymer slewing ring | Steel crossed roller |
| OD | 80 mm | 70 mm |
| Dynamic axial | 4,000 N (16×) | 7,350 N **(30×)** |
| Tilting moment | ⚠️ **Unknown** (LC variant) | ~153 Nm **(4.1× at 25 kg)** |
| Axial play | ±0.25 mm | ≤0.03 mm (P5 precision) |
| Maintenance | None (polymer) | Grease via lubrication hole (Ø3.1mm) |
| Cost | ~63€ | **~38€** |

> **Tilting moment calculation:** The RU42's dynamic radial load rating C = 7,350 N applied at the roller pitch circle radius dp/2 = 20.75 mm gives an estimated tilting moment capacity of 7,350 × 0.02075 ≈ **153 Nm**. At 25 kg with a 150 mm eccentric arm (worst case), the load is ~37 Nm — **4.1× safety factor**. This is a standard approximation for crossed roller bearings under pure moment loading; the static rating (C₀ = 8,350 N) gives 173 Nm (**4.7×**).

> **Comparison:** The igus PRT-01-20 (aluminium housing, the published upgrade path for the Prototype) is rated at 120 Nm tilting moment (3.3× SF). The RU42 exceeds this at lower cost and smaller footprint.

### RU42 Full Specifications

| Parameter | Value |
|---|---|
| **Model** | RU42UUCCO (P5 precision, sealed both sides) |
| **Inner diameter (d)** | 20 mm |
| **Outer diameter (D)** | 70 mm |
| **Width (B)** | 12 mm |
| **Roller pitch circle (dp)** | 41.5 mm |
| **Dynamic radial rating (C)** | 7.35 kN |
| **Static radial rating (C₀)** | 8.35 kN |
| **Mass** | 0.29 kg |
| **Precision** | P5 |
| **Lubrication hole** | Ø3.1 mm |

**Mounting holes:**

| Ring | PCD | Holes | Type |
|---|---|---|---|
| **Inner ring** | 28 mm | 6× M3 | Threaded through |
| **Outer ring** | 57 mm | 6× Ø3.4 | Through + Ø6.5 counterbore, depth 3.3 mm |

> 💡 Inner ring = fixed to base plate via 6×M3. Outer ring = rotating platform (telescope side), driven by harmonic drive adapter via 6×Ø3.4 through-holes.

### Harmonic Drive Adapter

A 3D-printed PLA+CF adapter bridges the harmonic drive output (PCD = 20.5 mm) to the RU42 outer ring (PCD = 57 mm, mounting holes at Ø3.4). The adapter carries motor torque only — no telescope load passes through it.

---

## 🔩 AZM Axis: Base Plate (Two-Piece CNC)

The V2 base replaces the Prototype's monolithic 15180 aluminium profiles with two separate CNC-machined aluminium plates joined by four precision pillars.

**Design rationale:**
- **JLCPCB cost:** Two separate small blocks are significantly cheaper than a large monolithic part.
- **Assembly tolerance:** The four pillars allow for fine parallelism adjustment during assembly, compensating for minor CNC flatness variation.
- **Anti-tip gap:** A 1 mm gap between the two plates on the anti-tip side prevents binding during AZM rotation under eccentric load.

**AZM adjustment interface:**
- Central M8 bolt fixes the Angeleyes 400 tripod extension (universal mount interface).
- A radial sliding slot accommodates an 8×16 shoulder screw (M6×8) as the AZM fine-adjustment pin during polar alignment.

> 📐 Refer to `PolarALIGN_V2_STEP.zip` for all CNC dimensions, tolerances, and hole patterns.

---

## 🔧 ALT Axis: CNC Bielle Mechanism

The V2 ALT axis replaces the commercial tilt plate with a fully custom CNC aluminium bielle (crank-arm) mechanism. The key design change is the **pivot repositioned below the T8 lead screw**, which recovers the full travel range that was reduced in earlier geometry iterations.

### Geometry (validated in simulation)

| Parameter | Value |
|---|---|
| Pivot position | (0, 0) — reference origin |
| T8 screw position | Y = +17.5 mm above pivot |
| Ball joint (BH) | (58, 40.25) mm |
| T8 nut attachment (BB) | (97.54, 17.5) mm |
| Bielle length (L) | 45.6 mm |
| Left bearing (Palier G) | X = −19 mm |
| Right bearing (Palier D) | X = +127.07 mm |
| Travel range | −2° to +10° |
| Linear course | 17.59 mm over 12° |
| Effective mm/° (linearized) | 1.466 mm/° at mid-travel |

### Structural margins

| Component | Specification | Actual load | Safety Factor |
|---|---|---|---|
| T8 axial force | ~500–1,000 N (bronze nut) | **245 N @ −2°** | **2–4×** |
| Bielle (rod) | Steel, 5.6 mm² section | ~11 N compression | **20.7×** |
| UMOT 30:1 output torque | ~2–4 Nm | ~0.04 Nm required | **51×** |

### Right bearing (Palier D): 626ZZ

The right-side bearing seat carries the T8 axial reaction force.

| Parameter | Value |
|---|---|
| Type | 626ZZ (6×19×6 mm) |
| Shaft | Ø6 mm, with Ø8→Ø6 shoulder |
| Axial stop | Shoulder against bearing inner ring (metal-on-metal — not aluminium) |
| Axial clearance | 0.5 mm |
| Through-hole in aluminium | Ø6.5 mm |
| Combined axial/radial SF | 5.3× |

> ⚠️ **Critical assembly note:** The Ø8→Ø6 shoulder must bear against the **inner ring** of the 626ZZ, not against the aluminium housing. Aluminium is too soft to serve as an axial stop under repeated T8 load cycles.

### Chariot (T8 nut carrier): Anti-backlash system

The T8 nut carrier uses a **three-nut anti-backlash system**:

| Position | Nut | Role |
|---|---|---|
| Left | Small nut + spring + large anti-backlash nut | Active spring pre-load, removes T8 backlash |
| Right | Large simple nut | Axial reaction stop |

- Distance between the two large nuts (inner faces): **19 mm** — satisfies the N×2+1 mm opposition-of-phase rule for anti-backlash nut geometry.
- Left bearing housing is **blind (borgne)** — serves as the axial stop for leftward T8 forces.

> 📐 Logement bore diameter: sized to accommodate the M3 fixing hole PCD (16 mm) of the large anti-backlash nut — do not use the nut OD alone as the bore spec.

### Firmware constant for V2

The bielle geometry gives a different effective crank ratio than the Prototype's commercial tilt plate:

```cpp
// V2 — ALT CNC bielle (UMOT 30:1 × 6.94 linearized crank factor)
constexpr float ALT_MOTOR_GEARBOX = 208.3f;
```

The MPU-6500 learning system will converge to the true value within 2–3 ALT jogs regardless of this starting estimate. The ±20% acceptance band means the firmware will tolerate a ~40 mm/° error before rejecting the update — well within the V2 geometry's expected range.

---

## ⚡ Electronics & Wiring

**Identical to the Prototype.** Refer to [`HARDWARE_Prototype.md` — Electronics & Wiring section](./HARDWARE_Prototype.md#-electronics--control) for:

- FYSETC E4 V1.0 board
- MPU-6500 via SD card sniffer (GPIO 18/19)
- UART jumper setup
- Full GPIO map
- Motor wiring
- Safety inputs

The only firmware difference is `ALT_MOTOR_GEARBOX = 208.3` in `PolarAlign_V2.ino`.

---

## ⚖️ Payload Rating (V2)

The RU42 crossed roller bearing eliminates the tilting moment uncertainty that capped the Prototype at 20 kg.

| Rating | Max Payload | Notes |
|---|---|---|
| **Recommended** | **25 kg** | Full 4.1× safety margin on tilting moment. |
| **Advanced** | **30+ kg** | Requires centered payload; ALT mechanism margins remain >20×. Bearing is not the limiting factor. |

**Full margin table at 25 kg:**

| Component | Capacity | Load @25 kg | Margin | Limiting? |
|---|---|---|---|---|
| RU42 – Axial dynamic | 7,350 N | ~245 N | **30×** | No |
| RU42 – Tilting moment (est.) | ~153 Nm | ~37 Nm @150mm arm | **4.1×** | ⚠️ Weakest (but known & comfortable) |
| T8 lead screw (bronze nut) | 500–1,000 N | ~25–40 N @2° tilt | **15–25×** | No |
| UMOT 30:1 output torque | ~2–4 Nm | ~0.04 Nm @operating angle | **51×** | No |
| Bielle (CNC rod) | Steel, SF 20.7× | 11 N compression | **20.7×** | No |
| 626ZZ right bearing | Combined SF 5.3× | Axial reaction only | **5.3×** | No |
| 3D-printed parts | Non-structural | Motor/sensor weight only | N/A | No |

> **Note:** These are simulation-derived values. Field validation with the fabricated hardware is pending and will update this table.

---

## 🖨️ 3D Printing & CNC Fabrication

### CNC Parts (aluminium, recommended JLCPCB)

| Part | Role | Structural? |
|---|---|---|
| Base plate — piece 1 | Lower platform, tripod interface, RU42 outer ring mount | ✅ Yes |
| Base plate — piece 2 | Upper platform, RU42 inner ring mount, pillar tops | ✅ Yes |
| ALT tilt plate | Bielle mechanism, pivot, 626ZZ seats, chariot guide | ✅ Yes |

> All CNC dimensions in `PolarALIGN_V2_STEP.zip`. Standard tap drill sizes: M3=2.5mm, M4=3.3mm, M5=4.2mm, M6=5.0mm.

- Estimated CNC cost (JLCPCB): **~90–110€**

### 3D Printed Parts

Same role and constraints as the Prototype — none are in the telescope load path.

| Part | Material | Note |
|---|---|---|
| Harmonic drive adapter (AZM) | PLA+CF | Motor torque only — no load |
| ALT motor cradle (UMOT) | PLA+CF (PETG if >300 mA) | Thermal concern at high current |
| MPU-6500 bracket | PLA | Negligible load |
| Homing switch bracket | PLA | Negligible load |
| FYSETC E4 enclosure | PLA | None |

---

## 🛒 Parts List (V2-specific changes vs Prototype)

| # | Component | Change | Est. Cost |
|---|---|---|---|
| — | ~~igus PRT-02 LC J4~~ | **Removed** | — |
| 1 | **RU42UUCCO** crossed roller bearing | New — AZM bearing | ~38€ |
| 2 | **CNC ALT** tilt plate assembly | New — replaces commercial tilt plate | ~90–110€ (JLCPCB) |
| 3 | **626ZZ** bearing (6×19×6 mm) | New — right-side T8 axial stop | <2€ |
| — | ~~T8 tilt plate coupler (5.6mm hack)~~ | **Removed** — V2 is native T8 | — |

All other components (harmonic drive, NEMA 17 motors, FYSETC E4, T8 kit, MPU-6500, sniffer, tripod extension) are **identical to the Prototype** — see [`HARDWARE_Prototype.md`](./HARDWARE_Prototype.md) for links and details.

### 💰 V2 Total Budget Estimate

| Category | Cost |
|---|---|
| Prototype hardware (minus igus + tilt plate) | ~**270€** |
| RU42 bearing | ~**38€** |
| CNC ALT + base plates (JLCPCB) | ~**100€** |
| 626ZZ bearing + fasteners | ~**5€** |
| **Total** | ~**413€** |

---

## 📄 License

**MIT License** — do whatever you want, just keep the header.

---

*Maintained by Antonino Nicoletti — clear skies!*
