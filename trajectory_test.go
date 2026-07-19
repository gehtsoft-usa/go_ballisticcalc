package externalballistics_test

import (
	"bufio"
	"math"
	"os"
	"strings"
	"testing"

	externalballistics "github.com/gehtsoft-usa/go_ballisticcalc"
	"github.com/gehtsoft-usa/go_ballisticcalc/bmath/unit"
)

// TestTrajectoryTemplates mirrors the .NET TrajectoryCalculatorTest.TrajectoryTest: it runs the same
// resource templates (testdata/trajectory/*.txt) through the Go engine with Step 50 yd / Max 1000 yd
// and asserts Distance, Velocity, Drop and Windage against the baseline within the SAME per-file
// tolerances the C# test defines. leadingNumber / dragTableFromSuffix / inchTol are shared with
// coriolis_test.go (same package).
func TestTrajectoryTemplates(t *testing.T) {
	cases := []struct {
		name    string
		velPct  float64 // distance & velocity, fraction
		dropMOA float64
		windMOA float64
	}{
		{"g1_nowind", 0.0015, 0.10, 0.05},
		{"g1_nowind_up", 0.0015, 0.20, 0.05},
		{"g1_nowind_up_supersonic", 0.0015, 0.10, 0.05},
		{"g1_twist", 0.0015, 0.10, 0.015},
		{"g7_nowind", 0.0015, 0.10, 0.05},
		{"g1_wind", 0.0015, 0.10, 0.10},
		{"g1_wind_hot", 0.0015, 0.10, 0.10},
		{"g1_wind_cold", 0.0015, 0.10, 0.05},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			runTrajectoryTemplate(t, "testdata/trajectory/"+tc.name+".txt", tc.velPct, tc.dropMOA, tc.windMOA)
		})
	}
}

func runTrajectoryTemplate(t *testing.T, path string, velPct, dropMOA, windMOA float64) {
	t.Helper()
	f, err := os.Open(path)
	if err != nil {
		t.Fatalf("open %s: %v", path, err)
	}
	defer f.Close()

	var ammo externalballistics.Ammunition
	var weapon externalballistics.Weapon
	var atmosphere externalballistics.Atmosphere
	var wind []externalballistics.WindInfo
	var shotAngleDeg, cantAngleDeg float64
	type refRow struct{ yd, dropIn, windIn, velFps float64 }
	var rows []refRow

	var bc externalballistics.BallisticCoefficient
	var weightGrain, velocityFps, diameterIn, lengthIn float64
	var hasDimensions bool
	var sightIn, zeroYd, twistIn float64
	var twistDir byte
	var hasTwist bool

	scanner := bufio.NewScanner(f)
	for scanner.Scan() {
		line := strings.TrimSpace(scanner.Text())
		if line == "" {
			continue
		}
		fld := strings.Split(line, ";")
		switch fld[0] {
		case "ammo":
			v, suffix := leadingNumber(fld[1])
			bc, err = externalballistics.CreateBallisticCoefficient(v, dragTableFromSuffix(t, suffix))
			if err != nil {
				t.Fatalf("CreateBallisticCoefficient: %v", err)
			}
			weightGrain, _ = leadingNumber(fld[2])
			velocityFps, _ = leadingNumber(fld[3])
			if len(fld) >= 6 {
				diameterIn, _ = leadingNumber(fld[4])
				lengthIn, _ = leadingNumber(fld[5])
				hasDimensions = true
			}
		case "rifle":
			sightIn, _ = leadingNumber(fld[1])
			zeroYd, _ = leadingNumber(fld[2])
			if len(fld) >= 5 {
				twistIn, _ = leadingNumber(fld[3])
				if strings.TrimSpace(fld[4]) == "left" {
					twistDir = externalballistics.TwistLeft
				} else {
					twistDir = externalballistics.TwistRight
				}
				hasTwist = true
			}
		case "wind":
			wv, _ := leadingNumber(fld[1])
			wd, _ := leadingNumber(fld[2])
			if wv != 0 {
				wind = externalballistics.CreateOnlyWindInfo(
					unit.MustCreateVelocity(wv, unit.VelocityMPH),
					unit.MustCreateAngular(wd, unit.AngularDegree))
			}
		case "atmosphere":
			tempF, _ := leadingNumber(fld[1])
			humidityPct, _ := leadingNumber(fld[2])
			pressInHg, _ := leadingNumber(fld[3])
			altFt, _ := leadingNumber(fld[4])
			atmosphere, err = externalballistics.CreateAtmosphere(
				unit.MustCreateDistance(altFt, unit.DistanceFoot),
				unit.MustCreatePressure(pressInHg, unit.PressureInHg),
				unit.MustCreateTemperature(tempF, unit.TemperatureFahrenheit),
				humidityPct)
			if err != nil {
				t.Fatalf("CreateAtmosphere: %v", err)
			}
		case "shot":
			shotAngleDeg, _ = leadingNumber(fld[1])
			cantAngleDeg, _ = leadingNumber(fld[2])
		default:
			if fld[0][0] >= '0' && fld[0][0] <= '9' {
				var r refRow
				r.yd, _ = leadingNumber(fld[0])
				r.dropIn, _ = leadingNumber(fld[1])
				r.windIn, _ = leadingNumber(fld[3])
				r.velFps, _ = leadingNumber(fld[5])
				rows = append(rows, r)
			}
		}
	}
	if err := scanner.Err(); err != nil {
		t.Fatalf("scan: %v", err)
	}

	var projectile externalballistics.Projectile
	if hasDimensions {
		projectile = externalballistics.CreateProjectileWithDimensions(bc,
			unit.MustCreateDistance(diameterIn, unit.DistanceInch),
			unit.MustCreateDistance(lengthIn, unit.DistanceInch),
			unit.MustCreateWeight(weightGrain, unit.WeightGrain))
	} else {
		projectile = externalballistics.CreateProjectile(bc, unit.MustCreateWeight(weightGrain, unit.WeightGrain))
	}
	ammo = externalballistics.CreateAmmunition(projectile, unit.MustCreateVelocity(velocityFps, unit.VelocityFPS))
	zero := externalballistics.CreateZeroInfo(unit.MustCreateDistance(zeroYd, unit.DistanceYard))
	if hasTwist {
		twist := externalballistics.CreateTwist(twistDir, unit.MustCreateDistance(twistIn, unit.DistanceInch))
		weapon = externalballistics.CreateWeaponWithTwist(unit.MustCreateDistance(sightIn, unit.DistanceInch), zero, twist)
	} else {
		weapon = externalballistics.CreateWeapon(unit.MustCreateDistance(sightIn, unit.DistanceInch), zero)
	}

	calc := externalballistics.CreateTrajectoryCalculator()
	sightAngle := calc.SightAngle(ammo, weapon, atmosphere)
	shot := externalballistics.CreateShotParametersEx(sightAngle,
		unit.MustCreateDistance(1000, unit.DistanceYard),
		unit.MustCreateDistance(50, unit.DistanceYard),
		unit.MustCreateAngular(shotAngleDeg, unit.AngularDegree),
		unit.MustCreateAngular(cantAngleDeg, unit.AngularDegree))

	data := calc.Trajectory(ammo, weapon, atmosphere, shot, wind)

	if len(data) != len(rows) {
		t.Fatalf("point count: got %d want %d", len(data), len(rows))
	}
	for i, r := range rows {
		p := data[i]
		gotDist := p.TravelledDistance().In(unit.DistanceYard)
		gotVel := p.Velocity().In(unit.VelocityFPS)
		gotDrop := p.Drop().In(unit.DistanceInch)
		gotWind := p.Windage().In(unit.DistanceInch)

		if math.Abs(gotDist-r.yd) > r.yd*velPct {
			t.Errorf("@%.0f distance: got %.2f want %.2f (±%.3f)", r.yd, gotDist, r.yd, r.yd*velPct)
		}
		if math.Abs(gotVel-r.velFps) > r.velFps*velPct {
			t.Errorf("@%.0f velocity: got %.2f want %.2f (±%.3f)", r.yd, gotVel, r.velFps, r.velFps*velPct)
		}
		if math.Abs(gotDrop-r.dropIn) > inchTol(dropMOA, r.yd) {
			t.Errorf("@%.0f drop: got %.2f want %.2f (±%.3f)", r.yd, gotDrop, r.dropIn, inchTol(dropMOA, r.yd))
		}
		if math.Abs(gotWind-r.windIn) > inchTol(windMOA, r.yd) {
			t.Errorf("@%.0f windage: got %.3f want %.3f (±%.3f)", r.yd, gotWind, r.windIn, inchTol(windMOA, r.yd))
		}
	}
}
