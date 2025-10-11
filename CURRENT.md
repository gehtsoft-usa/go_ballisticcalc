# Current Status - Go Ballistic Calculator Sync

## Project Overview
Syncing the Go port of the C# Ballistic Calculator library with changes from the C# version since July 2022.

C# Source: `/mnt/d/develop/components/BusinessSpecificComponents/BallisticCalculator.Net`
Go Target: `/mnt/d/develop/go/go_ballisticcalc`

## Completed Work

### 1. ✅ Added RA4 Drag Table
- **File**: `Drag.go`
- Added RA4 drag table constant (DragTableRA4 = 9)
- Added ra4Table data (87 data points)
- Added ra4Curve calculation
- Updated dragFunctionFactory switch case
- Fixed CreateBallisticCoefficient validation
- **Test**: TestPathRA4 - PASSING

### 2. ✅ Updated TrajectoryData Structure
- **File**: `TrajectoryData.go`
- Added new fields:
  - `travelDistanceFlat` - horizontal distance
  - `dropFlat` - drop relative to muzzle
  - `lineOfSightElevation` - line of sight elevation
  - `lineOfDepartureElevation` - line of departure elevation
- Added corresponding getter methods

### 3. ✅ Implemented Angle Calculation for Inclined/Declined Shots
- **File**: `TrajectoryCalculator.go`
- Added angle calculation variables (lines 145-154):
  - `lineOfSight`, `lineOfSightTan`, `lineOfSightCos`, `lineOfSightSin`
  - `lineOfDepartureTan`
  - `hasShotAngle` flag
- Implemented coordinate rotation for angled shots (lines 247-256)
- Added slant distance calculation: `distance = rangeVector.X / lineOfSightCos`
- Updated windToVector to account for shot angle

### 4. ✅ Fixed Critical Issues
- **deltaRangeVector.X calculation** (line 286): Changed from `calculationStep` to `velocityVector.X * deltaTime`
- **Altitude tracking** (line 289): Added `alt += deltaRangeVector.Y`
- **lastAtAltitude initialization** (line 200): Set to -3280839895.0 (equivalent to -1000000000 meters in feet)
- **Atmosphere caching**: Only update when altitude changes > 1 meter (3.28084 feet)

### 5. ✅ Created Test Suite
- **File**: `go_ballisticcalc_test.go`
- TestPathRA4 - Tests RA4 drag table with 40gr @ 2000fps
- TestAngledShotG1 - Tests 10° upward angle with 65gr @ 2600fps
- TestAngledShotG1Debug - Debug version with full trajectory output
- Adjusted test tolerances to isolate velocity issue

### 6. ✅ All Tests Passing Except One
- TestZero1 - PASSING
- TestZero2 - PASSING
- TestPathG1 - PASSING
- TestPathG7 - PASSING
- TestPathRA4 - PASSING
- TestAngledShotG1 - **FAILING** (velocity discrepancy only)

## Current Problem

### Velocity Discrepancy in Angled Shots

**Symptom**: When shooting at 10° upward angle, velocity is consistently ~40-44 fps too low at long range.

**Test Results**:
```
Angle = 0°:  VELOCITY CORRECT
  - At 500yd: 1537.4 fps (expected 1537.4) ✓
  - At 1000yd: 982.8 fps (expected 982.8) ✓

Angle = 10°: VELOCITY TOO LOW
  - At 500yd: 1495.2 fps (expected 1537.4) ✗ 42.2 fps low
  - At 1000yd: 939.1 fps (expected 982.8) ✗ 43.7 fps low
```

**Key Finding**:
- User confirmed: "I did simple test: I put angle in angled shot test to 0 and velocity calculation starts working properly."
- **This proves the issue is specifically related to how the angle affects the calculation**

### Cascading Effects
The velocity error causes:
- Mach (calculated from velocity)
- Energy (calculated from velocity)
- OGV (calculated from velocity)
- Drop (projectile falls more due to lower velocity)
- Hold (due to increased drop)

## What We Know

### Code Matches C# Implementation
- deltaTime calculation: `calculationStep / velocityVector.X` (same in both)
- Velocity update formula matches C# (line 285)
- Altitude adjustment matches C# (line 289)
- Atmosphere density/mach calculation matches C#
- All drag calculations match (PIR constant confirmed in Drag.go line 161)

### What Should Happen with Upward Angle
1. Initial velocity vector has larger Y component (upward)
2. Bullet travels to higher altitude
3. Air density decreases with altitude → LESS drag
4. Velocity should be HIGHER or same, not LOWER

### What We're Seeing
- MORE drag is being applied during angled shots
- The excess drag is consistent throughout the trajectory
- The issue only manifests when shot angle ≠ 0

## Investigation Focus Areas

### Possible Causes to Investigate

1. **deltaTime Calculation**
   - Line 281: `deltaTime = calculationStep / velocityVector.X`
   - When shooting upward, `velocityVector.X` is smaller (velocity distributed across X and Y)
   - Smaller denominator → larger deltaTime → more drag applied per step
   - **This could be the root cause**

2. **Velocity Magnitude vs Components**
   - Line 283: `velocity = velocityAdjusted.Magnitude()`
   - Check if drag should be applied to magnitude or components
   - C# line 346: `velocity = velocityAdjusted.Magnitude`

3. **Atmosphere Density Updates**
   - Altitude is tracked correctly (confirmed)
   - Density/mach updated when altitude changes > 1 meter
   - Need to verify density is actually DECREASING with altitude gain

4. **Wind Vector Interaction**
   - windToVector includes shot angle (lines 312-315)
   - Even with no wind, this could affect velocityAdjusted
   - Need to verify windVector = (0,0,0) when windInfo is nil

## Test Configuration

### Current Test Setup (TestAngledShotG1)
```go
BC: 0.365 (G1)
Weight: 65 grains
Muzzle Velocity: 2600 fps
Zero: 100 yards
Sight Height: 2.5 inches
Shot Angle: 10° upward
Maximum Distance: 1000 yards
Step: 50 yards
Wind: None (nil)
```

### Expected Values (from C# implementation and 3rd party calculator)
```
Distance  Velocity  Mach   Energy  Drop      Hold
0 yd      2600.0    2.329  975.5   -2.5"     0
100 yd    2360.1    2.114  803.8   0.0"      0
500 yd    1537.4    1.378  341.1   -67.4"    -12.9 MOA
1000 yd   982.8     0.882  139.4   -527.4"   -50.4 MOA
```

## Next Steps

1. **Add detailed debug output** to compare step-by-step:
   - velocityVector components (X, Y, Z)
   - deltaTime values
   - drag values
   - altitude changes
   - density factor / mach at each step
   - Compare angle=0 vs angle=10 side by side

2. **Verify deltaTime calculation**:
   - Check if C# uses different approach for angled shots
   - Investigate if calculationStep should be adjusted for angle

3. **Check velocity magnitude calculation**:
   - Ensure drag is calculated on correct velocity value
   - Verify velocityAdjusted when no wind

4. **Examine atmosphere density progression**:
   - Confirm density actually decreases as altitude increases
   - Check if density updates are happening at correct intervals

5. **Review C# implementation line-by-line**:
   - Compare TrajectoryCalculator.cs lines 343-375 with Go lines 281-291
   - Look for any subtle differences in order of operations

## Files Modified

- `Drag.go` - Added RA4 drag table
- `TrajectoryData.go` - Added new fields for flat distances and line elevations
- `TrajectoryCalculator.go` - Implemented angle calculations and fixes
- `ShotParameters.go` - Added CreateShotParametersEx function
- `go_ballisticcalc_test.go` - Added RA4 and angled shot tests

## Commands to Run Tests

```bash
# Run all tests
go test -v

# Run only angled shot test
go test -v -run TestAngledShotG1

# Run debug test with full output
go test -v -run TestAngledShotG1Debug

# Run all tests except angled shot
go test -v -run "^Test.*" | grep -v TestAngledShotG1
```

## Key Code Locations

### TrajectoryCalculator.go
- Line 145-154: Angle calculation variables
- Line 178: Initial velocity vector with angle
- Line 204: Initialize atmosphere for altitude
- Line 214-217: Update atmosphere when altitude changes
- Line 247-256: Coordinate rotation for angled shots
- Line 281: **deltaTime calculation** (potential issue)
- Line 283: **velocity magnitude calculation**
- Line 284: **drag calculation**
- Line 285: **velocity update** (critical formula)
- Line 286: deltaRangeVector calculation
- Line 288: Slant distance calculation
- Line 289: Altitude update

### Test Data Files
- Test expected values from: `/mnt/d/develop/components/BusinessSpecificComponents/BallisticCalculator.Net` tests
- RA4 data from: `/mnt/d/ra4.pdf`

## Resources

- C# Implementation: `/mnt/d/develop/components/BusinessSpecificComponents/BallisticCalculator.Net/BallisticCalculator/Calculations/TrajectoryCalculator.cs`
- PIR constant verified in: `Drag.go` line 161 (2.08551e-04)
- Gravity constant: `TrajectoryCalculator.go` line 14 (-32.17405)
