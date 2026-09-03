# ASTERIX in COMPASS

## What ASTERIX is

**ASTERIX** = **All Purpose STructured EUROCONTROL SuRveillance Information eXchange**. It is a binary data format standard maintained by EUROCONTROL for the exchange of air traffic surveillance information between sensors, processing systems, and ATC centers. It is the lingua franca of European ATC surveillance: every radar, MLAT, ADS-B ground station and most ATC tracker systems output ASTERIX.

The standard is split into **Categories** (CAT000-CAT255), each governing a specific kind of information (one radar's target reports, ground tracks, status messages, etc.). Each category is published in successive **Editions** (versioned PDFs), and inside a category the message layout is defined by a **UAP** (User Application Profile) that lists the data items, plus optional **REF** (Reserved Expansion Field) and **SPF** (Special Purpose Field) for vendor extensions.

**Wire structure** (from `data/jasterix_definitions/data_block_definition.json`):

```
[CAT byte][LEN 2 bytes][...records...]   <- "data block"
```

Each record begins with a **FSPEC** bitfield indicating which UAP slots are present, followed by the corresponding data items in UAP order. Recordings are usually wrapped in a **framing** layer (IOSS / IOSS-seq / RFF in `data/jasterix_definitions/framings/`) that adds per-frame timing/board metadata.

## Main categories for ATC surveillance

The categories COMPASS supports (`data/jasterix_definitions/categories/categories.json`, mirrored by `db_content_cat*.json` and `task_import_asterix_cat*.json` in `conf/default/`) cover the full surveillance chain:

| CAT | Role | Notes |
|---|---|---|
| 001 / 002 | Legacy monoradar target reports / service messages | superseded by 048/034, still seen in old recordings |
| 004 | Safety net messages (STCA, MSAW, APW, ...) | output of safety net function |
| 010 | Monosensor surface movement (A-SMGCS, MLAT-on-airport) | airport surface tracks |
| 019 | MLAT system status | health/config of an MLAT ground network |
| 020 | MLAT target reports | one report per MLAT update |
| 021 | **ADS-B target reports** | air-derived position/velocity from Mode S ES |
| 023 | CNS/ATM ground station status | ADS-B/VDL ground station health |
| 025 | CNS/ATM ground system status | service/system status of a ground system (e.g. MLAT, ADS-B) - imported as status-only DBContent |
| 034 | **Monoradar service messages** (sector crossings, north markers, status) | the sibling of 048 |
| 048 | **Monoradar target reports** (PSR/SSR/Mode S) | one plot per radar return - the workhorse |
| 062 | **System track messages** (multi-sensor tracker output, e.g. ARTAS) | smoothed tracks fused from many sensors |
| 063 | Sensor status messages from a tracker | which sensors the tracker sees as healthy |
| 065 | SDPS service messages | service-level status of the system data processor |
| 247 | Version number message | identifies ASTERIX version of the stream |

Categories COMPASS does **not** import but jASTERIX can decode (030/252) are ARTAS-internal formats described in the ARTAS interface spec.

The natural processing chain in an ATC system is therefore: **048 / 021 / 020 -> 062**, with 034 / 019 / 023 / 063 / 065 as the status side-channel and 004 as the safety-net output.

## Framings, REFs and SPFs

Three orthogonal extension mechanisms wrap or extend the basic ASTERIX record:

- **Framings** are recording-side wrappers around the raw CAT/LEN/records data block, added by the recorder (not by the sensor). They prepend per-frame metadata such as a recording-board timestamp, a packet sequence number, or line/source identification. The framings COMPASS knows about are **IOSS**, **IOSS-seq** and **RFF** (`data/jasterix_definitions/framings/`); the active framing is selected in `task_import_asterix.json`. Live network feeds carry no framing (records arrive in raw UDP datagrams), file recordings almost always do, and picking the wrong framing produces silent garbage rather than a parser error - the bytes still decode "into something".
- **REF (Reserved Expansion Field)** is a per-category, EUROCONTROL-defined extension slot (one item number per category) used to introduce new fields without breaking older parsers. Decoder definitions live in `data/jasterix_definitions/categories/<NNN>/cat<NNN>_ref_*.json`. Notably, the **CAT020 REF carries a Position Accuracy item that supersedes the legacy I020/500** (uses covariance instead of correlation, adds WGS-84 standard deviations, plus GVA for velocity). I020/500 is kept only for backwards compatibility - on a modern MLT feed the REF version is the authoritative one.
- **SPF (Special Purpose Field)** is a per-category, *vendor-defined* extension slot. Because it is vendor-specific, each SPF needs its own decoder definition (`cat<NNN>_spf_*.json`). The most prominent SPF in COMPASS is the **ARTAS Track Reference Information (TRI)** in CAT062, used to trace which contributing plots make up each system track.

## Information hierarchy in target reports

Across the target-report categories (048 monoradar, 021 ADS-B, 020 MLAT, 010 surface, 062 system tracks) the data items follow a recognisable **layered hierarchy**. The lower layers are present in essentially every report, the upper layers depend on what the target carries (transponder, Mode S, ADS-B avionics) and on what the sensor type measures or computes. Item numbers below refer to the latest editions implemented in jASTERIX (CAT048 1.23, CAT021 2.4, CAT020 1.8, CAT010 0.31, CAT062 1.21).

### Layer 0 - Origin and time (always present)

Identifies who produced the report and when it was valid.

- **Data source identifier** SAC/SIC: 048/010, 021/010, 020/010, 010/010, 062/010. Two bytes that uniquely name the producing sensor or tracker.
- **Time of day / applicability**: 048/140, 020/140, 010/140, 062/070. CAT021 distinguishes time of applicability for position (071), velocity (072), reception (073-076) and ASTERIX transmission (077) - this fine-grained timing is needed because ADS-B position and velocity originate as separate downlink messages.
- **Target report descriptor / message type**: 048/020, 020/020, 021/040, 010/000+020, 062/080. Tells the consumer what kind of report this is (PSR-only, SSR-only, PSR+SSR, Mode S all-call/roll-call, ADS-B air/surface, MLAT, simulated, test target, etc.).

### Layer 1 - Position (always present in target reports)

Different sensor families express position natively in different frames; categories therefore offer different position items:

- **Polar (RHO/THETA, slant range + azimuth from sensor)**: 048/040 (radar plot in measured polar), 010/040 (surface measured polar).
- **Cartesian (X/Y in local sensor frame)**: 048/042 (calculated), 020/042, 010/042, 062/100 (track position).
- **Geodetic (WGS-84 lat/lon)**: 020/041, 010/041, 021/130 (+021/131 high-resolution), 062/105.
- **Vertical**: barometric Mode-C / flight level (see Layer 2), measured 3D radar height 048/110, geometric height 010/091 + 010/105 + 021/140, measured flight level 062/136, calculated geometric/barometric track altitude 062/130 + 062/135.

For any radar-derived report, polar is the *measured* form and Cartesian/WGS-84 is *calculated* from it via the sensor's projection. ADS-B is geodetic-native.

#### Position accuracy information

Each category expresses position uncertainty differently - from "not transmitted at all" (CAT048) through compound standard-deviation items (CAT010, CAT020, CAT062) up to a multi-extension catalogue of integrity/accuracy categories (CAT021). The latest editions in `data/jasterix_definitions/` are summarized below; item references are to the most recent editions in the spec folder (see "Reference documents").

**CAT048 - monoradar (ed 1.28)**

CAT048 has **no native position-accuracy data item**. Standard deviations were briefly carried in I048/130 in early editions and were *removed* in edition 0.8 (May 1998). What is available is qualitative:

- **I048/030 Warning/Error Conditions and Target Classification**: enumerated codes (e.g. "plot may be in error", "RF detected", "split plot", reflection / range ambiguity flags, target classification). Codes 0-34 are defined as of edition 1.28.
- **I048/130 Radar Plot Characteristics**: SRL/SRR/SAM (SSR run-length, number of received replies, reply amplitude), PRL/PAM (PSR run-length and amplitude), RPD/APD (range and azimuth difference between PSR and SSR plot). These describe extraction quality, not σ.

Per-plot σ for CAT048 is therefore **derived inside COMPASS** from per-radar accuracy parameters (range/azimuth std dev for PSR/SSR/Mode S) configured in the data context, not read from the message.

**CAT021 - ADS-B (ed 2.6)**

Quality is reported in **I021/090 Quality Indicators**, a variable-length item with up to three one-octet extensions whose meaning depends on the MOPS version reported in I021/210/VN:

- *Primary subfield* (always present): **NUCp** / **NIC** (Navigation Uncertainty Category for Position / Navigation Integrity Category) and **NUCr** / **NACv** (Navigation Uncertainty Category for velocity / Navigation Accuracy Category for Velocity). NUC* are DO-260; NIC / NACv are DO-260A and later. Kept for backwards compatibility, so MOPS 2+ NIC values are mapped into this field as well.
- *1st extension*: **NACp** (Navigation Accuracy Category for Position), **SIL** (Surveillance Integrity Level / Source Integrity Level), **NICBARO** (Navigation Integrity Category for Barometric Altitude).
- *2nd extension*: **SDA** (Horizontal Position System Design Assurance Level, MOPS 2+), **GVA** (Geometric Vertical Accuracy), **SILS** (SIL-Supplement: per-flight-hour vs per-sample probability basis).
- *3rd extension*: **PIC** (Position Integrity Category, 0-15) - a single field that encodes the integrity containment bound directly. The spec carries a conversion table: PIC=14 → < 0.004 NM, PIC=11 → < 0.1 NM, PIC=8 → < 0.5 NM, PIC=1 → < 20 NM, PIC=0 → no integrity (≥ 20 NM).

For ADS-B the position itself in I021/130/131 is geodetic, the altitudes are in I021/140 (geometric) and I021/145 (flight level), and the per-item *ages* in I021/295 (which includes age of the QI block) tell the consumer how stale each accuracy figure is - important because ADS-B QIs update asynchronously from position.

**CAT020 - MLAT (ed 1.8 + REF 1.3)**

Two redundant position-accuracy items, the REF version being the recommended one:

- **I020/500 Position Accuracy** (compound):
  - Subfield 1 - **DOP of Position** (Cartesian): DOP-x, DOP-y, DOP-xy, all LSB 0.25.
  - Subfield 2 - **Standard Deviation of Position** (Cartesian): σx, σy (LSB 0.25 m), and the *correlation coefficient* ρ(X,Y) (LSB 0.25, two's complement).
  - Subfield 3 - **Standard Deviation of Geometric Height** σGH, LSB 0.5 m.
- **REF Item PA, Position Accuracy** (recommended over I020/500 - uses covariance instead of correlation, and adds WGS-84):
  - Subfield 1 - DOP of Position (Cartesian) with DOP-xy as a covariance-style component (sign·√|HDOPxy|).
  - Subfield 2 - **SDC** Standard Deviation of Position (Cartesian).
  - Subfield 3 - **SDH** Standard Deviation of Geometric Height (WGS-84).
  - Subfield 4 - **SDW** Standard Deviation of Position (WGS-84): σ(lat), σ(lon), Cov(lat,lon).

The REF additionally carries GVV (Ground Velocity Vector) and **GVA** (Ground Velocity Accuracy) for the velocity layer, plus TRT/DA for time-of-applicability and per-item ages.

**CAT010 - surface (ed 1.1)**

Single item, **I010/500 Standard Deviation of Position** (Cartesian only - the surface frame is a local airport projection):

- σx, σy: LSB 0.25 m.
- σxy: covariance in two's complement, LSB 0.25 m².

There is no native WGS-84 position accuracy in CAT010.

**CAT062 - system tracks (ed 1.21)**

The tracker has explicit covariance estimates from its filter, so **I062/500 Estimated Accuracies** carries a one-stop catalogue of every per-component standard deviation:

| Subfield | Quantity | LSB |
|---|---|---|
| 1 - APC | Track position σ (Cartesian X, Y) | 0.5 m |
| 2 - COV | XY covariance component (Cartesian, two's complement, sign·√\|Cov(X,Y)\|) | 0.5 m |
| 3 - APW | Track position σ (WGS-84 latitude, longitude) | 180/2²⁵ ° |
| 4 - AGA | Geometric altitude σ | 6.25 ft |
| 5 - ABA | Barometric altitude σ | 1/4 FL |
| 6 - ATV | Cartesian velocity σ (vx, vy) | 0.25 m/s |
| 7 - AA | Cartesian acceleration σ (ax, ay) | 0.25 m/s² |
| 8 - ARC | Rate of climb/descent σ | 6.25 ft/min |

I062/340 (Measured Information from the last contributing plot) does *not* carry an accuracy of the contributing measurement itself - only the measured quantities (3D height, polar position, Mode-3/A, Mode-C). The contributing-sensor σ has to be looked up in the corresponding monosensor record (or, for CAT048, derived as above) if needed.

**Quick reference**

| CAT | Native position accuracy | What it carries |
|---|---|---|
| 048 | none (qualitative I048/030, plot characteristics I048/130) | σ derived from per-radar parameters in COMPASS |
| 021 | I021/090 multi-extension QIs | NUCp/NUCr, NIC, NACp/NACv, SIL/SILS, NICBARO, SDA, GVA, PIC |
| 020 | I020/500 + REF PA | DOP, σ Cartesian (+ correlation or covariance), σ Geometric Height, σ WGS-84 (REF only), GVA |
| 010 | I010/500 | σx, σy, Cov(x,y) Cartesian |
| 062 | I062/500 | σ Cartesian, Cov(x,y), σ WGS-84, σ Geometric/Barometric Altitude, σ velocity, σ acceleration, σ ROCD |

### Layer 2 - Classic ("SSR") secondary information

What a non-Mode-S transponder downlinks - present whenever a target is equipped, irrespective of sensor type.

- **Mode 3/A code** (squawk, octal): 048/070, 020/070, 010/060, 062/060. Plus per-bit confidence on monoradar: 048/080.
- **Mode C / Flight Level** (barometric altitude reported by the transponder): 048/090 + 048/100, 020/090 + 020/100, 010/090, 062/135 (+062/136 measured). Detailed altitude variants below.
- **Mode 1 / Mode 2 (military)**: 048/050+055+060+065, 020/050+055.

Validity / garbled / low-confidence flags (V/G/L bits) are part of these items so a downstream consumer can reject suspect codes.

#### Altitude variants and reference frames

Across the categories COMPASS imports, vertical position is encoded under several different physical references. The transponder-derived (barometric) altitude lives in this layer; the geometric measurements are listed alongside because the *same* report often carries both, and a downstream consumer needs to know which is which.

| Reference frame | Source / how it is obtained | Items |
|---|---|---|
| Barometric, **1013.25 hPa standard** (uncorrected Mode-C / Mode-S altitude code) | Aircraft transponder, derived from static pressure | I048/090, I048/100, I020/090, I020/100, I010/090, I021/145, I062/135 (when QNH=0), I062/136 |
| Barometric, **QNH-corrected** | Tracker applies local QNH; flagged via QNH bit | I062/135 (when QNH=1); the *other* variant is then carried in I062/REF/MOI/CTBA, and a QNH-corrected measured FL in I062/REF/MOI/ALTQCMFL |
| **Geometric height above MSL** (3D radar physical measurement) | 3D radar height-finding, two's complement | I048/110 (25 ft LSB) - only present when the radar is a 3D radar |
| **Geometric height above WGS-84 ellipsoid** | Aircraft GPS (ADS-B) or computed by tracker | I021/140, I062/130 (both 6.25 ft LSB, ICAO requires < 10 ft LSB) |
| **Local Cartesian height** above an airport / MLAT reference frame (NOT barometric, NOT MSL) | Direct measurement by surface or MLT sensor | I010/091 (above local airport ref), I020/110 (above MLT system reference point given in I019/610) |

Per category, the altitude items are:

**CAT048 - monoradar**
- *I048/090 Flight Level* - the standard transponder altitude in 1/4 FL binary, with V (validated) and G (garbled) flags.
- *I048/100 Mode-C Code and Code Confidence* - raw 28-bit Gray-coded reply with per-bit confidence; sent when the binary FL in 048/090 is not validated or not decodable.
- *I048/110 Height Measured by 3D Radar* - geometric height referenced to **mean sea level**, two's complement, 25 ft LSB. Optional and only emitted by height-finding 3D radars.

**CAT021 - ADS-B**
- *I021/145 Flight Level* - barometric, **NOT QNH-corrected** (1013.25 hPa standard). 1/4 FL LSB.
- *I021/140 Geometric Height* - height above the **WGS-84 ellipsoid**, 6.25 ft LSB. The 0x7FFF code means "greater than" (avionics saturation).
- No raw Mode-C item: ADS-B carries the decoded altitude only, never the unprocessed transponder reply.

**CAT020 - MLAT**
- *I020/090 Flight Level* - Mode S altitude in binary, 1/4 FL LSB, V/G validity bits.
- *I020/100 Mode-C Code* - raw Gray-coded reply with confidence (in line with CAT048).
- *I020/110 Measured Height (Local Cartesian Coordinates)* - **direct multilateration height**, not barometric, referenced to the MLT system reference point defined in I019/610. 6.25 ft LSB, range ±204 800 ft.

**CAT010 - surface**
- *I010/090 Flight Level* - Mode-C / Mode S altitude in binary, 1/4 FL LSB, V/G flags.
- *I010/091 Measured Height* - **height above the local 2D airport coordinate system**, not barometric. 6.25 ft LSB, ±204 800 ft. There is no raw Mode-C item in CAT010.

**CAT062 - system tracks**
- *I062/130 Calculated Track Geometric Altitude* - vertical distance from target to its projection on the **WGS-84 ellipsoid**, 6.25 ft LSB. The source of this geometric altitude (which contributing sensor / how computed) is identified in I062/080 (SRC bits).
- *I062/135 Calculated Track Barometric Altitude* - 1/4 FL LSB, two's complement, with a **QNH bit** indicating whether QNH correction has been applied. Each record carries either the QNH-corrected or the non-QNH variant; the other is provided in I062/REF/MOI/CTBA when needed.
- *I062/136 Measured Flight Level* - the last valid and credible Flight Level used to update the track. Includes barometric altitudes received from ADS-B. The QNH-corrected variant of this measured FL is in I062/REF/MOI/ALTQCMFL.

The practical consequence: barometric and geometric altitude can differ by hundreds to thousands of feet for the same aircraft (weather, transponder pressure drift, geoid undulation, ellipsoid-vs-MSL), and within "barometric" the QNH-corrected value can differ from the standard-pressure value by tens of feet. Evaluation requirements in COMPASS distinguish between them so that, for example, a CAT021 geometric height is not compared against a CAT048 barometric Flight Level without an explicit conversion.

### Layer 3 - Mode S secondary information

Available when the target carries a Mode S transponder and is interrogated as such (or, for ADS-B, broadcasts ES). Conceptually this is the "who" layer.

**CAT001 has no Layer 3 items at all.** It is the legacy monoradar predecessor of CAT048, defined before Mode S existed: the item set stops at Mode-C (I001/100) and goes straight into plot characteristics and tracker fields - there is no aircraft address, no aircraft identification (callsign), no Mode S BDS / MB, and no ACAS. Any Mode S information for a CAT001 plot is therefore lost on the wire, so when modern Mode-S-equipped traffic appears in a CAT001 recording it can only be associated by Mode 3/A code and position.

- **24-bit ICAO aircraft address** (a.k.a. target address): 048/220, 020/220, 010/220, 021/080. Globally unique per airframe.
- **Aircraft / target identification** (callsign, 8 chars): 048/240, 020/245, 010/245, 021/170, 062/245. From Mode S BDS 2,0 or directly from the ADS-B identification message.
- **Communications / ACAS capability and flight status**: 048/230, 020/230.
- **Mode S MB data** (raw downlinked Comm-B registers, e.g. selected altitude, magnetic heading, IAS/Mach, vertical rate): 048/250, 020/250, 010/250, 021/250.
- **ACAS resolution advisory**: 048/260, 020/260, 021/260.
- **Mode 5 (military Mode S equivalent) and extended Mode 1**: 062/110.

For ADS-B (CAT021) the Mode S BDS contents are also broken out as **dedicated items** in Layer 4 (selected altitude 146/148, magnetic heading 152, airspeeds 150/151, vertical rates 155/157, roll angle 230) so that a consumer does not have to decode raw BDS frames.

### Layer 4 - Category-specific extensions

What each category adds beyond the common stack, driven by the sensor type.

**CAT048 - monoradar specific**
- 030 Warning/error conditions (radar-side quality flags)
- 110 3D height (3D radar only)
- 120 Radial Doppler speed
- 130 Radar plot characteristics (SRL/SRR/SAM/PRL/PAM/RPD/APD - run-length, amplitude, etc.)
- 161/170/200/210 Track number, status, calculated velocity, track quality (when the radar runs an internal monoradar tracker)

**CAT021 - ADS-B specific**
- 008 Aircraft operational status, 271 Surface capabilities
- 020 Emitter category (light, heavy, rotorcraft, vehicle, ...)
- 090 Quality indicators (NUCp / NACp / NIC / SIL / SDA / GVA) - the figures of merit that drive ADS-B trust
- 110 Trajectory intent
- 132 Message amplitude (1090ES RF level), 400 Receiver ID
- 145 Flight level, 146 Selected altitude, 148 Final state selected altitude
- 150 IAS/Mach, 151 True airspeed, 152 Magnetic heading
- 155 Barometric vertical rate, 157 Geometric vertical rate
- 160 Airborne ground vector, 165 Track angle rate
- 200 Target status, 210 MOPS version
- 220 Met information, 230 Roll angle
- 295 Data ages (per-item age, important for asynchronous ADS-B updates)

**CAT020 - MLAT specific**
- 030 Warning/error conditions
- 110 Measured height
- 202 Calculated Cartesian velocity, 210 Calculated acceleration
- 300 Vehicle fleet ID, 310 Pre-programmed message (for surface vehicles)
- 400 Contributing receivers (which ground stations participated in the multilateration solution)
- 500 Position accuracy (covariance)
- REF (1.3) carries DOP, station lists, additional position covariance details

**CAT010 - surface (SMR / surface MLAT) specific**
- 131 Amplitude of primary plot
- 270 Target size & orientation, 280 Presence (extent/footprint of the surface object)
- 300 Vehicle fleet ID, 310 Pre-programmed message
- 500 Position accuracy
- 550 System status

**CAT062 - system track specific** (output of a multi-sensor tracker)
- 040 Track number, 080 Track status, 290 System track update ages, 295 Track data ages
- 100/105 Calculated track position (Cartesian + WGS-84)
- 130/135/136 Calculated geometric / barometric / measured altitudes
- 185 Calculated Cartesian velocity, 210 Calculated acceleration, 220 Rate of climb/descent
- 200 Mode of movement
- 270 Target size & orientation
- 340 Measured information (last contributing plot)
- 380 Aircraft derived data (selected altitude, heading, airspeeds, ACAS, ADS-B QIs, ... fused from contributing reports)
- 390 Flight plan related data (callsign, dep/dest, type, RFL, SSR, runway, ...)
- 500 Estimated accuracies (per-component covariance)
- 510 Composed track number
- SPF carries the ARTAS Track Reference Information (TRI), used to trace which plots contributed to the track

### Layer-by-category quick reference

| Layer | CAT048 (radar) | CAT021 (ADS-B) | CAT020 (MLAT) | CAT010 (surface) | CAT062 (system track) |
|---|---|---|---|---|---|
| 0 origin/time | 010, 140 | 010, 071-077 | 010, 140 | 010, 140 | 010, 070 |
| 1 position | 040 polar, 042 Cart, 110 3D | 130/131 WGS-84, 140 geom h | 041 WGS-84, 042 Cart, 110 h | 041 WGS-84, 042 Cart, 040 polar | 100 Cart, 105 WGS-84, 130/135/136 alt |
| 2 SSR (Mode A/C) | 070, 090, 100 (+050/055) | 070, 145 | 070, 090, 100 (+050/055) | 060, 090 | 060, 135/136 |
| 3 Mode S id | 220 addr, 240 callsign, 230 caps, 250 BDS | 080 addr, 170 callsign, 250 BDS | 220 addr, 245 callsign, 230 caps, 250 BDS | 220 addr, 245 callsign, 250 BDS | 245 callsign (380 carries fused Mode S) |
| 4 cat-specific | 030, 120, 130, 161/170/200/210 | 008, 020, 090 QI, 110, 146-160, 200, 210 MOPS, 295 ages | 202, 210, 300/310, 400, 500 | 131, 270, 280, 300/310, 500, 550 | 040, 080, 185, 210, 220, 290/295, 340, 380, 390, 500, 510 |

The same conceptual layer thus shows up under different item numbers in different categories because the item numbering is per-category and grew historically. COMPASS hides this by mapping all of them onto a unified set of `DBContent` variables ("Time of Day", "Mode 3/A Code", "Aircraft Address", "Aircraft Identification", "Position Latitude", ...), so that downstream views, filters, evaluation requirements and the reconstructor work on the abstract layer regardless of which category supplied the report.

## How COMPASS uses and imports ASTERIX

COMPASS does not parse ASTERIX bytes itself - it links against **jASTERIX**, an OpenATS-maintained C++ library that turns ASTERIX bytes into JSON (and can encode JSON back to bytes) using **definition files only** - no recompile is needed to support a new edition. The library itself - repo layout, supported editions, definition file format, output formats, decode/encode API, CLI client - is documented in [readme_jasterix.md](/home/sk/workspace/jasterix/readme_jasterix.md) (jasterix skill).

### Two layers of definitions

1. **jASTERIX side** - byte-level decoding rules in `data/jasterix_definitions/` (a copy of `~/workspace/jasterix/definitions/`, kept in sync): the CAT/LEN envelope, framings, the `categories.json` edition registry, and per-edition item layouts plus REF/SPF definitions. Format details in [readme_jasterix.md](/home/sk/workspace/jasterix/readme_jasterix.md).

2. **COMPASS side** - JSON-to-DB-column mapping in `conf/default/`:
   - `task_import_asterix.json` - master config: framing, chunk sizes, packet overload thresholds, geo/time/Mode-C filters, SAC/SIC/ToD overrides, and one `ASTERIXCategoryConfig` sub-config per category selecting **edition / REF / SPF**
   - `task_import_asterix_cat<NNN>.json` - an `ASTERIXJSONParser` with many `JSONDataMapping` entries that bind a jASTERIX JSON key (e.g. `"140.Time-of-Day"`) to a `db_content_variable_name` (e.g. `"Time of Day"`) plus dimension and unit. This is what fills the columnar `Buffer` / `NullableVector<T>` rows that COMPASS works on.
   - `db_content.json` and `db_content_cat<NNN>.json` define the destination DBContent tables.

### Runtime flow

Code in `src/task/import/asterix/` and `src/import/asterix/`:

1. `ASTERIXImportTask` (`asteriximporttask.h`) owns one `std::shared_ptr<jASTERIX::jASTERIX>` instance. `initjASTERIX()` constructs the decoder against `data/jasterix_definitions/`; `configurejASTERIX()` applies the active context's per-category edition/REF/SPF picks.
2. A decoder backend - file (`asterixfiledecoder.cpp`), pcap (`asterixpcapdecoder.h`), live network (`asterixnetworkdecoder.cpp`), or replayed JSON (`asterixjsondecoder.cpp`) - feeds bytes to jASTERIX, which emits JSON in chunks using jASTERIX's **flat (columnar) format**: per category, one array column per leaf field with one entry per record.
3. `ASTERIXJSONMappingJob` (`asterixjsonmappingjob.cpp`) maps the flat columns via the per-category `ASTERIXJSONParser` mappings (`parseFlatJSON` -> `JSONDataMapping::setFlatArrayValues`) and writes typed values into Buffers. Repetitive-item leaves (e.g. the CAT062 ARTAS SPF TRIs, flat column `SPF.Target Report Identifiers.TRI`) arrive as an array of scalars per record cell; mappings with `in_array` iterate these and, with `append_value`, join them into one value (TRIs become a semicolon-separated string in `target_report_identifiers`).
4. `ASTERIXPostProcessJob` (`asterixpostprocessjob.h`) fixes ToD wrap/offset, applies SAC/SIC overrides, computes derived fields, then hands buffers off for DuckDB insertion.
5. The pipeline runs in parallel via JobManager + TBB, with the `num_packets_overload` / `max_packets_in_processing` knobs in `task_import_asterix.json` keeping memory bounded for large files or sustained live feeds.

**Live simulation (network replay)**: the network import can be fed from IOSS-framed
recordings instead of real sensors, without leaving the production code path.
`ASTERIXNetworkReplaySender` (`asterixnetworkreplaysender.cpp`) walks the IOSS frames of a
recording, paces by the frame header times (speed factor supported) and sends each frame's
content as one UDP datagram to a configured network line endpoint of the active context.
Multiple recordings are replayed simultaneously (one sender each, sharing the earliest
first frame time as common pacing base so their relative timing is kept) -
from the socket onward everything (UDP receivers, decode batching, live insert, display) is
the normal live path. The context's network lines are used unchanged: multicast groups are
sent host-local (multicast loopback enabled, hops 0, so the replayed recording never reaches
a real network), non-multicast listen addresses (e.g. "0.0.0.0") are redirected to
127.0.0.1. Limitation: a line with a `sender_ip` filter drops the replayed data (source
address cannot be spoofed) - a warning is logged. Started via
`import_asterix_network --replay_file '<file1>;<file2>'` (plus
`replay_speed`, `replay_line`, `replay_stop_at_end`; also available as
`--import_asterix_network_replay_*` command line options). Record Time of Day values are
aligned to the current wall clock via the existing ToD override: unless `time_offset` is
given explicitly, the offset is computed as current UTC time minus the first frame time.
Note that faster-than-real-time replay pushes shifted ToD values into the future beyond the
network import's +3 min check, so speeds > 1 should be paired with `ignore_future_ts`.

The full chain is therefore:

**ASTERIX bytes -> jASTERIX (definitions) -> JSON -> ASTERIXJSONParser (mappings) -> Buffer columns -> DBContent in DuckDB**

with categories chosen at the top of that chain determining what kind of surveillance data ends up in which DBContent table.

What happens *after* the buffers reach DuckDB - the DBContent / Variable / MetaVariable model, the `db_content*.json` schema, ToD wrap and SAC/SIC handling, how loaded data feeds views/filters/eval/reconstruction - is documented in [readme_dbcontent.md](../../db/dbcontent/readme_dbcontent.md).

### Mapping coverage

Everything jASTERIX decodes is mapped into DBContent variables - coverage is complete, not a curated subset. Audited 2026-08-27 with [`scripts/check_asterix_mapping_coverage.py`](../../../scripts/check_asterix_mapping_coverage.py), which reconstructs the flat JSON leaf-key space from the definition files (default edition + default REF edition per category, ~1300 leaf fields across the 14 imported categories) and compares it against the `JSONDataMapping` json_keys:

- **Data item level**: every item of every default edition has at least one active mapping, REFs included.
- **Leaf field level**: zero unmapped decoded fields. The audit found exactly one gap - the legacy `I020/500.SDH.sigma-gh` (Standard Deviation of Geometric Height) was decoded but only its `REF.PA.SDH` variant was mapped - closed same day by adding the fallback mapping to `Geometric Height StdDev`.
- Spare bits and FX/extension markers carry no information and are excluded.
- A small number of mapped keys (~14) lie *outside* the default-edition key space: deliberate multi-edition fallback mappings (e.g. deprecated `I021/030 Time of Day`, DO-260 `I021/090.PA` vs. DO-260A quality indicators, pre-1.21 CAT062 item shapes). They fire only when data in that shape arrives.

Maintenance rules:

- Adding a new edition, REF, or category, or changing mappings: re-run the checker; it exits non-zero when a decoded field has no mapping.
- The checker audits the **default** editions from `categories.json`. When a category is switched to a non-default edition in the import configuration, the mapping set for that edition is not separately verified - extend the checker if that becomes a recurring case.
- The two inactive placeholder mappings in `task_import_asterix_cat062.json` (`artas_md5`, `510.Composed Track Number.extend`, empty variable name, `active: false`) are intentional and ignored by the checker.

## Import result report

Every successful ASTERIX import appends to a **single, persistent `TaskResult`** named `"ASTERIX Import"` (type `Generic`), stored in the task results browser and exportable to DOCX/LaTeX/JSON. Canceled or errored imports leave the result untouched (the task manager is opened with `clear_existing=false` only inside the success branch of `checkAllDone()`). One `"ASTERIX Import"` result lives per DB; opening another DB loads its own result from `db_info`-adjacent storage.

Structure:

- **Overview / Imported Files**: one row per file ever imported into this DB, in chronological order. Columns: `#`, `Begin`, `Elapsed`, `File`, `Size (bytes)`, `Source`, `Framing`, `Errors`, `Warnings`. Each row links to the corresponding per-file section under `Files`.
- **Files / #N <basename>**: one section per imported file, with `N` a globally unique 1-based index across all imports into this DB. Inside:
  - `Info` table: filename, full path, size, content info, source type, framing, import timestamp, elapsed, PCAP section count.
  - `Records per Data Source` table: one row per data source seen in the file, with per-CAT record counts from probe-time data, linking to the per-DS sub-sections below.
  - `Skipped Categories` table (only when present): categories found in the recording but not decoded, one row per CAT with columns `Category`, `Data Blocks`, `Size`, `Reason`. Reason is "no specification" (jASTERIX has no definition for the category) or "decoding disabled" (category disabled in the import configuration). Since skipped data blocks cannot be split into records without a UAP, only data block counts and bytes are available - no record counts and no SAC/SIC attribution. Source: jASTERIX emits a `skipped_categories` object in its analysis result (counted from the raw CAT/LEN data block envelopes during `analyzeFile`/`analyzeData`), parsed via `ASTERIXSkippedCategoryInfo::fromAnalysisInfo(...)` into `ASTERIXImportFileInfo`/`ASTERIXImportFileSection`. Skipped categories also show up in the import dialog's file tree ("048, 030 (skipped)") and tooltips; a recording containing only skipped categories fails the decode check with "Only skipped categories: ..." instead of a generic decoding error.
  - One sub-section per **DS type** seen in this file (`Radar`, `MLAT`, `ADSB`, `Tracker`, `Other`), each containing one sub-section per **data source** of that type (heading: `<DS name> (<sac>/<sic>)`). Each data-source sub-section contains one **table per CAT** (no further nesting). Tables have columns `Item`, `Count`, `Min`, `Max`, `Description`. The count cell is formatted `"<n> (<pct.1f> %)"` against the per-(DS,CAT) total. Items defined in the active edition but never seen in the file are listed with count 0. Source: `ASTERIXImportProbeAggregator::aggregateFile(file_info)`. DS type comes from the registered `DataSource::dsType()` when known; otherwise inferred via `ASTERIXImportProbeAggregator::inferDsType(...)`.
  - `Errors / Warnings` text: from `ASTERIXImportFileError` plus per-PCAP-section errors/warnings, when any.

The cumulative cross-import per-(DS, CAT, item) summary persists separately in `DBContextManager::asterixInfo()` (db_info key `"asterix_info"`, see [readme_context.md](../../../core/context/readme_context.md)) and is refined on every import via `DBContextManager::mergeAsterixInfo(...)`. The report itself only renders per-file probe data - for the cumulative store, query the `DBContextManager` directly.

## Reference documents

The original EUROCONTROL specification PDFs are kept locally under `~/Nextcloud/documents/asterix/`, organized by category. The most recent editions present (and used as the basis for the per-category descriptions above) are:

| CAT | Latest edition on disk | File |
|---|---|---|
| 010 | 1.1 (March 2007) | `010/cat010-asterix-monoradar-surface-movement-data-part-7.pdf` |
| 020 | 1.8 (December 2010) | `020/cat020-asterix-p14-v1.8-20101201.pdf` |
| 020 REF | 1.3 (April 2010) | `020/asterix-cat020-appendix-a-coding-rule-for-reserved-expansion-field-part14-v1.2-042010.pdf` |
| 025 | 1.5 (July 2021) | `025/eurocontrol-asterix-cat025-pt26-ed15.pdf` (1.1 in `025/20151015-asterix-cat025-part26-v1.1.pdf`) |
| 021 | 2.6 | `021/asterix-adsbtr-cat021-part12-v2-6.pdf` |
| 048 | 1.28 | `048/eurocontrol-cat048-pt4-ed128.pdf` |
| 062 | 1.21 | `062/asterix-cat062-system-track-data-p9-ed1-21.pdf` |
| ARTAS (CAT030/031/032/252) | - | `cats-30-31-32-252-interface-specification-application-of-asterix-to-artas.pdf` |
| Part 1 (general) | ed 3.1 / ed 2.4 | `eurocontrol-specification-asterix-part1-ed-3-1.pdf`, `part_1_-_eurocontrol_specification_asterix_spec-149_ed_2.4.pdf` |
| Categories overview | 2021-09-27 | `asterix-categories-and-statuses-20210927.pdf` |

Older editions of each category are kept alongside in the same folder for diffing against historical recordings. The jASTERIX-side per-edition decoders in `data/jasterix_definitions/categories/<NNN>/cat<NNN>_<edition>.json` are derived from these PDFs (see [readme_jasterix.md](/home/sk/workspace/jasterix/readme_jasterix.md) for the definition format).

In-project ASTERIX-related locations:

| Path | What lives there |
|---|---|
| `~/workspace/jasterix/` | jASTERIX library source (sibling repo), holding the canonical definition files under `definitions/` and the spec PDFs at the repo root. Documented in [readme_jasterix.md](/home/sk/workspace/jasterix/readme_jasterix.md) (jasterix skill) |
| `data/jasterix_definitions/` | The byte-level decoding rules shipped *with* COMPASS - a copy of `~/workspace/jasterix/definitions/`, kept in sync. This is what jASTERIX is constructed against at runtime; the definition file format is documented in [readme_jasterix.md](/home/sk/workspace/jasterix/readme_jasterix.md) |
| `conf/default/` | The COMPASS-side mappings: `task_import_asterix.json` (master config: framing, chunk sizes, packet overload thresholds, geo/time/Mode-C filters, SAC/SIC/ToD overrides, plus one `ASTERIXCategoryConfig` per category selecting the active edition / REF / SPF), `task_import_asterix_cat<NNN>.json` (one `ASTERIXJSONParser` per category with the JSON-to-`DBContent` `JSONDataMapping` entries), and `db_content.json` + `db_content_cat<NNN>.json` (destination DBContent table definitions) |
| [`readme_adsb_accuracy.md`](../../../readme_adsb_accuracy.md) | In-depth notes on ADS-B accuracy/integrity indicators: NUCp/NACp/NIC/NACv/SIL/SDA/GVA per MOPS version, the I021/090 → CAT021 DBContent variable mapping, conversion of QI values to position σ in COMPASS, and how `ADSBAccuracyEstimator` consumes them in the reconstructor. Read this whenever the Layer 1 / Layer 4 CAT021 quality fields above need actual numbers or COMPASS-side semantics |
