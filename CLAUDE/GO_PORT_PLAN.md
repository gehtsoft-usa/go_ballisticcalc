# Go port plan — applying `CHANGES_TO_PORT.md` (.NET `f3b62f8..b2d1c46`)

Companion to `CLAUDE/CHANGES_TO_PORT.md`. This records the **Go-specific** decisions made
after triaging every checklist item against the current Go code, plus the agreed scope.

## Triage: what does NOT need porting

| Doc item | Go status | Reason |
|---|---|---|
| §1.2 bidirectional drag-node walk | **N/A** | `Drag.go:CalculateByCurve` does a fresh binary search per call — no cached node, so the stale-node bug cannot occur. |
| §1.6 inclined-drop rotation | **Already present** | `TrajectoryCalculator.go` (`hasShotAngle` branch) already uses the `sightHeight·(cos−1)` form. Verify + comment only. |
| §2.1 Mach saved / §2.2 Windage saved | **Already correct** | `TrajectoryData` is built with named struct fields, not a positional constructor — no missing assignment. |
| §2.3 BC equality tolerance / §2.4 invariant TimeSpan parse | **N/A** | Go has no serialization layer (no JSON/XML, no equality method) in this repo. |
| §3 `.drg` **write** (`Save`) | **Excluded by decision** | Not needed. `.drg` **read** IS in scope (see below). |
| §4 optimization rewrite, §5 docs | **Skip** | Per doc. |

## Sound-speed constant (§1.1)

Go uses an Imperial form `c = cSpeedOfSound·√(°R)`. Numbers at 59 °F:

| | constant (fps/√°R) | c at 59 °F |
|---|---|---|
| Old C# `331·√(T/273)` | 48.9885 | 1115.66 fps |
| **New C# `331.3·√(T/273.15)`** | **49.0195** | **1116.37 fps** |
| Go current | 49.0223 | 1116.44 fps |

Go's existing constant was already ~90 % of the way to the corrected C# value (0.07 fps / 0.006 %
off), i.e. Go never used the old `331/273` formula. **Decision: sync to C#** — set
`cSpeedOfSound = 49.0195` (= `331.3/√(273.15·1.8)/0.3048`, to be pinned against C# numeric output).
The shift is far inside existing test tolerances (5 fps, 0.005 mach), so no path test re-baselining is
expected. Also audit other shared constants (gravity `cGravityConstant`=32.17405 ft/s² == 9.80665 m/s²;
Ω; drag `PIR`) for Go↔C# agreement.

## GC / custom drag (§3) — "Path B minus write"

**Decision: full data-backed GC support, everything except `.drg` writing.**

Current Go GC state: `DragTableGC` exists, but the only way to a GC coefficient is
`CreateBallisticCoefficientForCustomDragFunction` storing an **opaque closure**;
`dragFunctionFactory` has no GC case; and `CreateBallisticCoefficient(value, DragTableGC)`
**panics** (passes the range check, then hits `default: panic`). Latent bug to fix.

In scope:
- First-class data-backed custom drag table type (`[]DataPoint` + curve via existing
  `CalculateCurve`/`CalculateByCurve`), used through the normal `Drag()` path.
- Fix `CreateBallisticCoefficient(DragTableGC)` to return an error rather than panic.
- `BcAtMach{Mach, Bc}` + multi-BC factory: `Cd_custom(M) = Cd_base(M)/BC(M)` on the base table's own
  Mach grid; piecewise-linear `BC(M)` with flat extrapolation; guards (base≠GC, ≥1 knot, all Bc>0).
- `.drg` **read** (CFM/BRL `Open` → `[]DataPoint`).

Out of scope: `.drg` **write** (`Save`).

## Coriolis / Eötvös (§1.7)

Go's `ShotParameters` has **no azimuth at all** (`barrelAzimuth` hardcoded 0). Add nullable
`latitude` + `azimuth`; unset ⇒ Coriolis off, bit-identical to today; azimuth stays a scalar (muzzle
`vz` unchanged). All corrections done in Go's feet/fps frame (units verified consistent — no meter
conversion needed):
- Horizontal: `windage -= Ω·sin(φ)·distance·time` (Ω = 7.2921159e-5).
- Vertical Eötvös: `vRatio = 1 − 2·Ω·cos(φ)·sin(Az)·V₀fps/cGravityConstant`;
  `boreLine = lineOfDepartureElevation`; `ryEffective = boreLine + (ry − boreLine)·vRatio`, fed into
  the flat drop and the §1.6 rotation.
- **Skip Coriolis in `SightAngle`.**

## Naming: match existing Go `CreateXxx` idiom (no special preferences).

## Implementation workstreams

1. **Physics fixes** (`Atmosphere.go`, `TrajectoryCalculator.go`): §1.1 constant, §1.4 `time += deltaTime`,
   §1.5 spin drift `× lineOfSightCos`, §1.3 end margin `/lineOfSightCos`, §1.6 verify+comment. Re-run tests.
2. **Coriolis** (§1.7): add `latitude`/`azimuth` to `ShotParameters`; per-output-point corrections; skip in `SightAngle`.
3. **GC / custom drag** (§3): data-backed GC type, fix panic, `BcAtMach` + factory, `.drg` read.
4. **Test data + tests**: copy .NET refs (Coriolis `SET*.txt` + config, angled-fire/multi-BC) into the
   Go repo; mirrored Go tests (Coriolis, custom-drag identity, `.drg` read, angled fire). `go build/test` green.

## Validation targets
Windage ≈ exact; drop within ~0.16 % (drag-baseline-limited); Coriolis inert when latitude unset;
custom-drag identity (flat BC + BC 1.0 reproduces the standard table).

## Outcome (implemented)

All four workstreams complete; `go build/vet/test ./...` green (16 tests), staticcheck clean apart from
one pre-existing unused-func finding (`Atmosphere.getDensity`).

- WS1 physics: `cSpeedOfSound 49.0223→49.0195`; `time += deltaTime`; spin drift `× lineOfSightCos`;
  end margin `/lineOfSightCos`; §1.6 comment. Existing path/angled tests stay green (shifts sub-tolerance).
- WS2 Coriolis: `latitude`/`azimuth` + `CreateShotParametersCoriolis`; per-output-point windage +
  Eötvös-fall corrections; skipped in `SightAngle`. Signs verified (N→right, S→left, East less drop,
  West more), muzzle drop pinned to −sightHeight.
- WS3 GC: `BcAtMach`, `BuildDrgDragTable`, `DrgDragTable` (+ `CreateDrgDragTable`, `.drg` read via
  `OpenDrgDragTable`/`ReadDrgDragTable`), `CreateBallisticCoefficient(GC)` now errors instead of panicking.
  Custom-drag identity is **exact** (Go reuses the same curve machinery). No `.drg` write, by decision.
- WS4 tests: reference data copied to `testdata/coriolis/` (6 `be_coriolis_*.txt`) and `testdata/drg/`
  (`sierra_168_brl.drg`). New tests: `TestCoriolisTrajectory`, `TestDrg*`.
- WS5 (below): SI humid-air density model + integration-step default aligned to .NET.

## WS5 — root-cause fix for the long-range Go↔.NET divergence

During validation Go's absolute drop diverged from BE/.NET beyond ~1150 yd (~0.8 % / 1.7 MOA at
2000 yd). Investigation (ruling out drag-curve construction — identical; node selection — negligible;
PIR / standard density — identical; sound speed — synced; step size and dt — negligible) found **two**
real Go↔.NET differences:

1. **Humid-air density model.** Go used the old imperial JBM approximation, whose humidity correction
   under-weights water vapor; at 78 % RH it produced ~0.5 % denser air than .NET's SI model (dry-air +
   vapor partial pressures, `287.058`/`461.495`, Wobus SVP polynomial), i.e. ~0.5 % excess drag. Ported
   the SI model into `Atmosphere.go` (`calculateDensitySI`, `saturatedVapourPressure`, SI barometric
   temperature/pressure at altitude; sound speed unchanged at `331.3·√(T_K/273.15)`). Dry-air results
   are unchanged; the humid divergence is closed.
2. **Integration step default.** Go's default `MaximumCalculatorStepSize` was 1 ft vs .NET's **0.1 m**;
   combined with the scale-sensitive `log10` flooring in `getCalculationStep`, this made Go's derived
   step ~10× coarser at a 50 yd output step (0.25 yd vs 0.025 yd), overshooting reported distances and
   costing long-range accuracy. Set the Go default to 0.1 m to match.

**Result:** Go now matches .NET/BE within the .NET tests' own tolerances. `TestCoriolisTrajectory` was
switched from the earlier swing-vs-baseline workaround to .NET's **absolute** assertions (distance/
velocity ≤0.5 %, drop ≤0.4 MOA, windage ≤0.05 MOA) to 2000 yd and passes. New `TestTrajectoryTemplates`
runs the **same .NET resource templates** (`testdata/trajectory/g1_*`, `g7_nowind`) with the **same
per-file tolerances** as `TrajectoryCalculatorTest.TrajectoryTest` and passes.

The old bespoke `TestPathG1`/`TestPathG7` (.308/168gr, no .NET counterpart, pre-SI imperial values) were
removed — superseded by `TestTrajectoryTemplates`, which validates G1/G7 against the shared .NET data.

### Note: azimuth-without-latitude
In Go, azimuth is only settable via `CreateShotParametersCoriolis` (which enables Coriolis), so the .NET
"azimuth set but latitude null ⇒ identical to azimuth 0" case is not expressible and needs no test — a
cleaner API than the .NET separate-`BarrelAzimuth` design.
