# Changes to port to the Go and Java versions

Sister ports: **Go** `github.com/gehtsoft-usa/go_ballisticcalc`, **Java**
`github.com/nikolaygekht/ballistic.calculator.java`.

This document is the **port hand-off** for the .NET engine work done **after the shot-angle
coordinate fix (`f3b62f8`, 2025‑10‑04) and the RA4 drag table (`b425f55`, 2025‑09‑16)** — the two
landmarks the ports were last synced against. It exists so the Go/Java maintainers can apply the
changes from **language-neutral specs (formulas, constants, conventions)** without re-reading the C#
diffs. The .NET commit range covered is **`f3b62f8..b2d1c46`** (HEAD as of 2026‑07‑19).

Everything is grouped by **what a port must do**, most important first:
- §1 Physics / behavior changes — **MUST port** (change trajectory numbers).
- §2 Data / serialization bug fixes — **MUST port** (silently lost or wrong data).
- §3 New feature — **SHOULD port** (new public API: multi-BC → custom drag curve).
- §4 The optimization rewrite — **NO port action**, but read the numeric-parity caveat.
- §5 Docs / cosmetic — **verify only / skip**.
- §6 Commit map, §7 per-language checklist, §8 reference data for validation.

> Convention note used throughout: **Windage sign is left +, right −**; **Drop** is vs line of sight
> and equals **−sightHeight at the muzzle**; angles + up / − down; `shotAngle`/`losAngle` is the
> line-of-sight inclination; `barrelElevation = sightAngle + shotAngle`.

---

## 1. Physics / behavior changes — MUST port

### 1.1 Sound-speed constants corrected (biggest accuracy fix) — `48d1209`
`Atmosphere.CalculateSoundVelocity`, temperature `T` in **Kelvin**, result m/s:

```
OLD:  soundVelocity = 331   * sqrt(T / 273)
NEW:  soundVelocity = 331.3 * sqrt(T / 273.15)
```

Why it matters: sound speed sets the Mach number, which drives **every** drag-table lookup, so the
error propagated into velocity along the whole trajectory (~6× tighter velocity match on the G7 case
after the fix). **Port:** change the two literals — multiplier `331 → 331.3`, divisor `273 → 273.15`.
No call-site changes; `AtAltitude(...)` returns this local Mach speed to both `SightAngle` and
`Calculate`. Ensure the temperature you pass is absolute (Kelvin).

### 1.2 Bidirectional drag-node walk — `48d1209`
The drag-curve node is cached across integration steps (a doubly-linked list node with `Mach`,
`Previous`, `Next`; `CalculateDrag(mach) = C + mach*(B + A*mach)`). Mach usually **falls**, but on
inclined/steep shots and transonic wobble the air-relative speed can **rise** between steps. The old
code only walked backwards, leaving the node stale and selecting the wrong polynomial.

```
OLD:  while (node.Mach > currentMach)                        node = node.Previous;
NEW:  while (node.Mach > currentMach)                        node = node.Previous;
      while (node.Next != null && node.Next.Mach <= currentMach)  node = node.Next;
```

**Port:** add the forward-advance loop, in **both** the `Calculate` loop **and** the `SightAngle`
zeroing iteration.

### 1.3 Steep-shot termination margin — `48d1209`
The loop's range guard compares against the **line-of-sight** distance `distanceMeters = rx / cos(losAngle)`,
so the end margin must be in the same frame or steep shots stop one output step early.

```
OLD:  maximumRangeMeters = rangeToMeters + calcStepMeters
NEW:  maximumRangeMeters = rangeToMeters + calcStepMeters / cos(losAngle)
```
(`cos(losAngle) = 1` when level.) **Port:** divide the one-step end margin by `cos(shotAngle)`.

### 1.4 Flight-time = integrated time, not re-derived — `48d1209`
Reported time must be the **sum of the integration timesteps**, not re-derived from displacement/speed
(which diverges on angled shots).

```
OLD:  time += Duration( drMag / velocityMag_mps )    # drMag = |Δposition| in meters
NEW:  timeSeconds += dt                               # per step, plain double
      ... output row uses Duration(timeSeconds)
```
**Port:** keep a `double timeSeconds` accumulator, add `dt` each step, convert to your time type only
when writing an output row. (`dt` derivation itself is unchanged — see §4 for the tick-truncation
parity note.)

### 1.5 Spin drift projected by cos(shot angle) — `48d1209`
Spin drift is horizontal; on an inclined shot only its cos-projection lies in the reported windage
plane.

```
OLD:  windage += driftFactor * time^1.83 * driftDirection * inchToMeter
NEW:  windage += driftFactor * time^1.83 * driftDirection * inchToMeter * cos(losAngle)
```
`driftFactor = 1.25·(Sg + 1.2)` (Miller stability `Sg`), `driftDirection = −1` right twist / `+1`
left, exponent `1.83`, all unchanged. **Port:** multiply the spin-drift term by `cos(shotAngle)`.
(Still no aerodynamic / crosswind-jump term — absent by design.)

### 1.6 Inclined-fire drop rotation — **verify only** (documented in `48d1209`, logic pre-existing)
The perpendicular-rotation drop formula already existed at the `f3b62f8` baseline; `48d1209` only
added a comment locking it in. Applied only when a shot/LoS angle is present:

```
if (hasShotAngle):
    y         = ry + sightHeight                          # lift into vertical frame
    y_rotated = -rx*sin(losAngle) + y*cos(losAngle)       # rotate into LoS-perpendicular frame
    drop      = y_rotated - sightHeight                   # subtract sightHeight back in ROTATED frame
else:
    drop = ry
```
`rx` = flat down-range (m), `ry` = raw vertical (m, starts at −sightHeight). **Intentional quirk — do
NOT "simplify":** sightHeight is added in the vertical frame but subtracted back in the rotated frame;
the residual `sightHeight·(cos(losAngle) − 1)` deliberately pins the **muzzle drop to exactly
−sightHeight** while every downrange point uses the true perpendicular rotation. **Port:** confirm the
Go/Java angled-drop code is exactly this (not a plain `ry·cos`); if so, only port the comment/intent.

### 1.7 Coriolis / Eötvös deflection + azimuth decoupling — `b2d1c46` (NEW capability)
Full rationale and reference data: `CLAUDE/CORIOLIS.md`. Implemented as **per-output-point closed-form
corrections** (NOT per-step −2Ω×v integration): time-of-flight, velocity and range are untouched, so
this is a surgical post-step correction on each output row.

**New public field:** `ShotParameters.Latitude` — nullable angle, **N +, S −**; `null` ⇒ Coriolis off.
Serialized as optional `latitude` (XML) / omit-when-null (JSON).

**Azimuth semantics change (breaking, but alpha):** `ShotParameters.BarrelAzimuth` (0° = North,
clockwise → 90° = East) **no longer tilts the trajectory**. The bullet is always integrated along the
line of fire (initial `vz = 0`); azimuth is now a pure scalar into the Coriolis terms. This also
**removes the old `BarrelAzimuth ≈ 90°` divide-by-zero** (where the down-range velocity component, and
thus `dt = step/vx`, collapsed). For `Az ∈ {0, null}` the trajectory is bit-identical to before.

Constants: `Ω = 7.2921159e-5 rad/s`, `g = 9.80665 m/s²`. `φ = Latitude`, `AZ = BarrelAzimuth`,
`V₀` = muzzle velocity (m/s). Per output point with down-range `Range` (m) and time-of-flight `TOF` (s):

- **Horizontal** (fold into `Windage`, alongside spin drift), azimuth-independent:
  ```
  Windage -= Ω · sin(φ) · Range · TOF          # deflects right in N hemisphere ⇒ negative (left +/right −)
  ```
- **Vertical Eötvös**, modelled as modified gravity, scales the **gravitational fall below the vacuum
  bore line** (NOT the raw vertical position — see the muzzle caveat below):
  ```
  vRatio  = 1 − 2·Ω·cos(φ)·sin(AZ)·V₀ / g               # dimensionless, constant per shot
  boreLine(rx) = rx·tan(barrelElevation) − sightHeight  # == LineOfDepartureElevation (vacuum trajectory)
  ry_eff  = boreLine + (ry − boreLine)·vRatio            # scale only the fall
  ```
  Then compute `Drop`/`DropFlat` from `ry_eff` (feed `ry_eff` into the §1.6 rotation for angled fire).
  East (sin AZ > 0) ⇒ less drop; West ⇒ more; hemisphere-symmetric (cos φ is even).

  **Muzzle caveat (important):** scale the *fall* `(ry − boreLine)`, not raw `ry`. At the muzzle
  `ry = boreLine = −sightHeight`, so the fall is 0 and the muzzle drop stays exactly −sightHeight.
  Scaling raw `ry` instead shifts the muzzle row by ~0.02″ and is wrong.

- **Do NOT apply Coriolis in `SightAngle`** (zeroing stays purely ballistic; effect at a 100 yd zero is
  <0.02 MOA). Record as a deliberate choice.

Reference match (Ballistic Explorer + Kestrel, which agree ~0.4%): windage ≈ exact; drop within ~0.16%
(drag-baseline-limited). No fudge/scale factor is used. Sign check: N hemisphere windage < 0 (right),
S > 0 (left), equator ≈ 0; East less drop, West more.

---

## 2. Data / serialization bug fixes — MUST port

### 2.1 `Mach` not saved — `c7850f0`
`TrajectoryPoint` constructor accepted a `mach` argument but **never assigned the field** (every point
had `Mach = 0`). Not an attribute problem — a missing assignment. **Port:** confirm the Go/Java
`TrajectoryPoint` builder/constructor actually stores `mach`.

### 2.2 Linear `Windage` not saved — `ca399f3`
Same class, same root cause: the constructor took `windage` but never assigned it, so `Windage` was
always default 0 — losing **both** linear wind drift **and** spin drift on every point. **Port:**
confirm the `Windage` field is assigned from the constructor argument.

### 2.3 `BallisticCoefficient` equality is now tolerance-based — `07e22fe`
```
OLD:  equal  ⇔  ValueType==other.ValueType && Value==other.Value && Table==other.Table
NEW:  equal  ⇔  ValueType==other.ValueType && Table==other.Table && |Value − other.Value| < 1e-7
```
Exact `double ==` on the BC value was replaced with a 1e-7 absolute tolerance (avoids round-trip /
parse mismatches). **Port:** if your `BallisticCoefficient` implements value equality / hashing, use
the same 1e-7 tolerance on the numeric value (and keep hashcode consistent, e.g. don't hash the raw
double).

### 2.4 `TimeSpan` deserialization uses invariant culture — `07e22fe`
Deserializer changed `TimeSpan.TryParse(text, out ts)` → `TimeSpan.TryParse(text, InvariantCulture, out ts)`.
**Port:** parse serialized time/duration values with invariant/`en-US`-style formatting, independent of
the host locale (matters where `.` vs `,` decimal separators differ).

---

## 3. New feature — SHOULD port: multi-BC → custom drag curve (`552ac97`, tests in `8c2a1e4`)

Lets a caller synthesize a custom `Cd(Mach)` curve from a standard base table plus an effective-BC
profile that varies with Mach, then run it through the existing custom-table (`GC`) path.

**New value type `BcAtMach`** — one knot of the profile. Immutable, two `double`s:
- `Mach` (≥ 0) — Mach number at which the BC applies.
- `Bc` (> 0) — effective BC at that Mach, relative to the base curve.
- Constructor `BcAtMach(mach, bc)`.

**New factory `DrgDragTableFactory.Build(ammunition, baseTable, bcCurve)` → custom drag table:**
- `ammunition` — metadata (name/weight/diameter) attached to the result; non-null.
- `baseTable` — a standard `DragTableId` (G1, G7, …); must **not** be `GC`.
- `bcCurve` — `IEnumerable<BcAtMach>`, ≥ 1 knot, any order (sorted internally).
- Guards (throw): null ammunition/curve; `baseTable == GC`; empty curve; any `Bc ≤ 0`.

**Algorithm (exact):**
1. Sort knots ascending by `Mach`.
2. `baseCurve = standard table for baseTable` (hardcoded `(Mach, Cd)` points).
3. On the base curve's **own Mach grid** (same count, same Mach values — do NOT resample):
   ```
   Cd_custom(M) = Cd_base(M) / BC(M)
   ```
4. Result is a custom table with `TableId = GC`.

**`BC(M)` interpolation** — piecewise-linear between Mach-sorted knots, **flat extrapolation** outside:
```
M ≤ knots[0].Mach      → knots[0].Bc
M ≥ knots[last].Mach    → knots[last].Bc
else, bracket a,b with M ≤ b.Mach:  BC = a.Bc + (M − a.Mach)/(b.Mach − a.Mach) · (b.Bc − a.Bc)
```

**Usage contract:** run the resulting `GC` table with `BallisticCoefficient(value = 1.0, GC)` so the
engine's per-step `(1/BC)` factor is 1.0 and `Cd_custom` is applied exactly. (Flat BC profile + BC 1.0
reproduces the standard-table trajectory to ~0.05 fps / 0.02 in — the identity check.)

**`DrgDragTable.Save(stream|fileName, encoding = ASCII)`** — writes the `.drg` file (leave-open on the
stream overload). Symmetric with the existing `Open`. Format:
- Header (comma-separated, always 6 fields): `CFM,<name>,<weightKg>,<diameterM>,0,0`
  - field 0 literal `CFM`; `<name>` = ammo name or `"custom"`, commas replaced by spaces;
    weight in **kg**, diameter in **meters**, both round-trip formatted, invariant culture; last two
    fields literal `0,0`.
- One line per point: `<Cd> <Mach>` — **space-separated, Cd first, Mach second**, round-trip invariant.
- Reader (`Open`, unchanged): header field 0 ∈ {`CFM`,`BRL`}; weight kg / diameter m; point lines split
  on whitespace, exactly 2 tokens (else skipped), token0 = Cd, token1 = Mach.

---

## 4. Optimization rewrite — NO port action, but read this (`a8caf6d`, `9a1ac85`, `7a0e609`, PR `50fbe9e`)

The .NET `TrajectoryCalculator` hot loop was rewritten from `Measurement<T>` (typed-unit struct) math
into **raw doubles** for performance. It is a **pure performance rewrite** — validated bit-equivalent
(1e-7) against the pre-optimization C# engine across all 7 reference cases. The Go/Java ports almost
certainly already use plain floats/doubles, so **there is nothing to port here** and you should **not**
try to reproduce the C# ULP behavior. Two takeaways only:

1. **Numeric conventions the algorithm assumes** (worth matching for cross-port agreement):
   velocity in the **muzzle-velocity's native unit** (not m/s); position in **meters** (x toward
   target, y drop, z windage); altitude accumulated **per step in its native unit**; density &
   sound-speed refreshed only when altitude moves **> 1 m** (a deliberate, result-affecting
   approximation); `GetCalculationStep` = halve the output step, then if still > max step divide by
   `10^(floor(log10 step) − floor(log10 maxStep) + 1)`.
2. **Bit-for-bit cross-language equality is not achievable** and should not be a goal. The C# output
   depends on (a) .NET `TimeSpan` 100 ns **tick truncation** of `dt` and the reported time, and (b)
   the Gehtsoft.Measurements conversion constants (distance base unit = **inch**, velocity base =
   m/s). No other language reproduces these exactly. Port the **algorithm and conventions**, and set
   cross-port test tolerances realistically (looser than 1e-7) rather than chasing bit-equality.

Minor cleanup rolled in here: in `SightAngle` the drag-node walk guard was simplified from
`while (node.Previous != null && node.Previous.Mach > currentMach)` to `while (node.Mach > currentMach)`
(now consistent with `Calculate`, and combined with the §1.2 forward walk). Treat as intended.

---

## 5. Docs / cosmetic — verify only / skip

- **Wind direction convention** (`cd81cb0`, doc-only in .NET): the authoritative convention is
  **0° = toward the target (tailwind), 90° = from the right, 270°/−90° = from the left, 180° = toward
  the shooter (headwind)**. The .NET *logic* was always correct; only its doc was wrong. **Port action:**
  verify the Go/Java wind-vector decomposition matches this (if a port copied the *old wrong doc* into
  its sign logic, that is a real bug to fix) and correct the port's doc comment.
- **Reticle** (`07e22fe`): pure cosmetic (extracted `const "black"` / title strings; Sonar pragmas).
  Mil-dot geometry unchanged (circle r=6 mil, cross ±5 mil, ticks ±5→±6 at width 0.2, dots r=0.1 mil at
  integer mils −4..4 skipping 0). **No behavior to port.**
- **`Rifle.cs`** (`07e22fe`): doc-comment cross-reference only. No behavior.
- Docgen/skill/guide/build-script/nuget commits (`09cef14`, `646c403`, `ce0b760`, `d4e02f0`, `20b7315`,
  `2396d06`, `9882877`, `998ad3a`, `376c3df` fluent→awesome assertions): **.NET-only** tooling/tests/docs.
  Not applicable to Go/Java.

---

## 6. Commit map (.NET, `f3b62f8..b2d1c46`)

| Commit | Date | Port? | What |
|---|---|---|---|
| `c7850f0` | 2025‑10‑07 | §2.1 | Mach not saved (ctor assignment) |
| `ca399f3` | 2025‑10‑07 | §2.2 | Linear windage not saved (ctor assignment) |
| `367eff8` | 2025‑10‑07 | no | test assertion-lib rollback (.NET) |
| `376c3df` | 2025‑11‑11 | no | FluentAssertions → AwesomeAssertions (.NET tests) |
| `cd81cb0` | 2025‑11‑11 | §5 | wind-direction **doc** fix (verify port logic) |
| `07e22fe` | 2026‑03‑05 | §2.3, §2.4 | cleanup: BC equality tolerance + TimeSpan invariant parse (+ cosmetic reticle/serializer) |
| `a8caf6d`,`9a1ac85`,`7a0e609`,`50fbe9e` | 2026‑03/04 | §4 | hot-loop optimization (no port action) |
| `998ad3a` | 2026‑05‑07 | no | docgen deps (.NET) |
| `48d1209` | 2026‑07‑13 | §1.1–1.6 | 3DOF correctness + sound-speed correction |
| `552ac97` | 2026‑07‑13 | §3 | DrgDragTableFactory / BcAtMach / DrgDragTable.Save |
| `8c2a1e4` | 2026‑07‑13 | §8 | multi-BC→custom-drag tests + 4DOF references (data to reuse) |
| `09cef14`…`9882877` | 2026‑07‑13 | no | docs / guide / SKILL / build (.NET) |
| `b2d1c46` | 2026‑07‑19 | §1.7 | Coriolis / Eötvös + azimuth decoupling |

---

## 7. Per-language checklist

Apply §1–§3 in this order; skip §4/§5.

1. [ ] Sound speed `331.3·√(T/273.15)` (§1.1).
2. [ ] Forward drag-node advance in `Calculate` **and** `SightAngle` (§1.2).
3. [ ] End-range margin `/cos(shotAngle)` (§1.3).
4. [ ] Time = accumulated `Σ dt`, converted only at output (§1.4).
5. [ ] Spin drift `× cos(shotAngle)` (§1.5).
6. [ ] Verify angled-drop rotation matches §1.6 (keep the `sightHeight·(cos−1)` residual).
7. [ ] Coriolis: add `ShotParameters.Latitude`; decouple azimuth from the muzzle vector (`vz=0`);
       add horizontal + Eötvös-fall corrections; skip in `SightAngle` (§1.7).
8. [ ] `TrajectoryPoint` assigns `Mach` and `Windage` from its constructor (§2.1, §2.2).
9. [ ] `BallisticCoefficient` equality uses 1e-7 tolerance (§2.3).
10. [ ] Time/duration deserialization uses invariant culture (§2.4).
11. [ ] `BcAtMach` + `DrgDragTableFactory.Build` + `DrgDragTable.Save` (§3).
12. [ ] Verify wind-direction decomposition matches the convention (§5).

---

## 8. Reference data for validation (in this repo)

- **Coriolis (BE + Kestrel):** `CLAUDE/data/SET0.txt … SET16.txt` (Ballistic Explorer captures, 50‑yd
  granularity to 2000 yd; SET0 = Coriolis off baseline) and the Kestrel table in `CLAUDE/CORIOLIS.md`
  §6.5. The .NET tests derived from these: `BallisticCalculator.Test/resources/be_coriolis_*.txt`
  (BE config, drop = BE "Path", windage = −(BE "Drift"); azimuth/latitude in the `shot` row) and
  `CoriolisTest.cs`. Config: BE §6.6 (65 gr, 0.365 G1, 2600 fps, sight 2.50 in, zero 100 yd,
  0 ft / 29.53 inHg / 59 °F / 0.78 RH); Kestrel §6.5 (69 gr, 0.365 G1, 2600 fps, sight 3.2 in,
  0 ft / 29.92 inHg / 59 °F / 0 RH).
- **Angled fire / 4DOF (Hornady):** references added in `48d1209` and `8c2a1e4`; see `CLAUDE/hornady/`
  and the `g1_nowind_up*` templates. Expected accuracy: within ~0.1 MOA drop while supersonic,
  relaxing to ~0.2 MOA transonic/subsonic (G1-drag-shape divergence, not algorithm error).
- **Test-template format** (reused by the port if helpful): `BallisticCalculator.Test/Calculator/TableLoader.cs`.
  The `shot` row is `shot;<shotAngle>;<cantAngle>;<azimuth>;<latitude>` (azimuth/latitude optional; a
  blank field ⇒ null).

Expected post-port accuracy vs the field references: **windage ≈ exact**, **drop within ~0.16%**
(drag-baseline-limited through the transonic zone), Coriolis inert when `Latitude` is unset.
