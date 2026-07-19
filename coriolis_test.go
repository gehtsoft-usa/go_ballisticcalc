package externalballistics_test

import (
	"bufio"
	"math"
	"os"
	"strconv"
	"strings"
	"testing"

	externalballistics "github.com/gehtsoft-usa/go_ballisticcalc"
	"github.com/gehtsoft-usa/go_ballisticcalc/bmath/unit"
)

// Coriolis / Eötvös acceptance ported from the .NET TrajectoryCalculatorTest.CoriolisTrajectoryTest.
// The reference data (Ballistic Explorer captures) lives in testdata/coriolis/be_coriolis_*.txt.
// Column mapping (per the .NET test): drop = BE "Path" (our Drop), windage = -(BE "Drift"), left +/right -.

type coriolisRefRow struct {
	yards    float64
	dropIn   float64
	windIn   float64
	velFps   float64
}

type coriolisRef struct {
	bcValue     float64
	dragTable   byte
	weightGrain float64
	velocityFps float64
	sightIn     float64
	zeroYard    float64
	tempF       float64
	humidityPct float64
	pressInHg   float64
	altFt       float64
	shotAngle   float64
	cantAngle   float64
	azimuthDeg  float64
	latitudeDeg float64
	hasLatitude bool
	rows        []coriolisRefRow
}

// leadingNumber parses the leading signed decimal of s and returns it plus the (unit) remainder.
func leadingNumber(s string) (float64, string) {
	i := 0
	for i < len(s) {
		c := s[i]
		if (c >= '0' && c <= '9') || c == '.' || c == '-' || c == '+' {
			i++
		} else {
			break
		}
	}
	f, _ := strconv.ParseFloat(s[:i], 64)
	return f, s[i:]
}

func dragTableFromSuffix(t *testing.T, suffix string) byte {
	switch strings.ToUpper(strings.TrimSpace(suffix)) {
	case "G1":
		return externalballistics.DragTableG1
	case "G2":
		return externalballistics.DragTableG2
	case "G5":
		return externalballistics.DragTableG5
	case "G6":
		return externalballistics.DragTableG6
	case "G7":
		return externalballistics.DragTableG7
	case "G8":
		return externalballistics.DragTableG8
	case "GS":
		return externalballistics.DragTableGS
	case "RA4":
		return externalballistics.DragTableRA4
	default:
		t.Fatalf("unknown drag table suffix %q", suffix)
		return 0
	}
}

func loadCoriolisRef(t *testing.T, path string) coriolisRef {
	t.Helper()
	f, err := os.Open(path)
	if err != nil {
		t.Fatalf("open %s: %v", path, err)
	}
	defer f.Close()

	var ref coriolisRef
	scanner := bufio.NewScanner(f)
	for scanner.Scan() {
		line := strings.TrimSpace(scanner.Text())
		if line == "" {
			continue
		}
		fields := strings.Split(line, ";")
		switch fields[0] {
		case "ammo":
			bc, suffix := leadingNumber(fields[1])
			ref.bcValue = bc
			ref.dragTable = dragTableFromSuffix(t, suffix)
			ref.weightGrain, _ = leadingNumber(fields[2])
			ref.velocityFps, _ = leadingNumber(fields[3])
		case "rifle":
			ref.sightIn, _ = leadingNumber(fields[1])
			ref.zeroYard, _ = leadingNumber(fields[2])
		case "atmosphere":
			ref.tempF, _ = leadingNumber(fields[1])
			ref.humidityPct, _ = leadingNumber(fields[2])
			ref.pressInHg, _ = leadingNumber(fields[3])
			ref.altFt, _ = leadingNumber(fields[4])
		case "shot":
			ref.shotAngle, _ = leadingNumber(fields[1])
			ref.cantAngle, _ = leadingNumber(fields[2])
			if len(fields) > 3 && strings.TrimSpace(strings.TrimSuffix(fields[3], "°")) != "" {
				ref.azimuthDeg, _ = leadingNumber(fields[3])
			}
			if len(fields) > 4 && strings.TrimSpace(strings.TrimSuffix(fields[4], "°")) != "" {
				ref.latitudeDeg, _ = leadingNumber(fields[4])
				ref.hasLatitude = true
			}
		default:
			// data rows start with a digit; header row (yd;in;...) is skipped here
			if fields[0][0] >= '0' && fields[0][0] <= '9' {
				var row coriolisRefRow
				row.yards, _ = leadingNumber(fields[0])
				row.dropIn, _ = leadingNumber(fields[1])
				row.windIn, _ = leadingNumber(fields[3])
				row.velFps, _ = leadingNumber(fields[5])
				ref.rows = append(ref.rows, row)
			}
		}
	}
	if err := scanner.Err(); err != nil {
		t.Fatalf("scan %s: %v", path, err)
	}
	return ref
}

// inchTol converts an MOA tolerance at the given range to inches, floored at 0.001 in
// (matching the .NET test's InchTol helper).
func inchTol(moa, yards float64) float64 {
	tol := moa * 1.0471996 * yards / 100.0
	if tol < 0.001 {
		tol = 0.001
	}
	return tol
}

// coriolisModelRun runs a be_coriolis reference case through the Go engine and returns the model
// drop/windage/velocity indexed by whole yard, plus the reference rows for the same case.
type coriolisModelRun struct {
	ref       coriolisRef
	dropByYd  map[int]float64
	windByYd  map[int]float64
	velByYd   map[int]float64
	refByYd   map[int]coriolisRefRow
	muzzleIn  float64 // reported drop at the muzzle row
	sightInch float64
}

func runCoriolisCase(t *testing.T, name string) coriolisModelRun {
	t.Helper()
	ref := loadCoriolisRef(t, "testdata/coriolis/"+name+".txt")

	bc, err := externalballistics.CreateBallisticCoefficient(ref.bcValue, ref.dragTable)
	if err != nil {
		t.Fatalf("CreateBallisticCoefficient: %v", err)
	}
	projectile := externalballistics.CreateProjectile(bc, unit.MustCreateWeight(ref.weightGrain, unit.WeightGrain))
	ammo := externalballistics.CreateAmmunition(projectile, unit.MustCreateVelocity(ref.velocityFps, unit.VelocityFPS))
	zero := externalballistics.CreateZeroInfo(unit.MustCreateDistance(ref.zeroYard, unit.DistanceYard))
	weapon := externalballistics.CreateWeapon(unit.MustCreateDistance(ref.sightIn, unit.DistanceInch), zero)
	atmosphere, err := externalballistics.CreateAtmosphere(
		unit.MustCreateDistance(ref.altFt, unit.DistanceFoot),
		unit.MustCreatePressure(ref.pressInHg, unit.PressureInHg),
		unit.MustCreateTemperature(ref.tempF, unit.TemperatureFahrenheit),
		ref.humidityPct)
	if err != nil {
		t.Fatalf("CreateAtmosphere: %v", err)
	}

	calc := externalballistics.CreateTrajectoryCalculator()
	sightAngle := calc.SightAngle(ammo, weapon, atmosphere)

	maxDist := unit.MustCreateDistance(2000, unit.DistanceYard)
	step := unit.MustCreateDistance(50, unit.DistanceYard)
	var shot externalballistics.ShotParameters
	if ref.hasLatitude {
		shot = externalballistics.CreateShotParametersCoriolis(sightAngle, maxDist, step,
			unit.MustCreateAngular(ref.shotAngle, unit.AngularDegree),
			unit.MustCreateAngular(ref.cantAngle, unit.AngularDegree),
			unit.MustCreateAngular(ref.azimuthDeg, unit.AngularDegree),
			unit.MustCreateAngular(ref.latitudeDeg, unit.AngularDegree))
	} else {
		shot = externalballistics.CreateShotParametersEx(sightAngle, maxDist, step,
			unit.MustCreateAngular(ref.shotAngle, unit.AngularDegree),
			unit.MustCreateAngular(ref.cantAngle, unit.AngularDegree))
	}

	data := calc.Trajectory(ammo, weapon, atmosphere, shot, nil)

	run := coriolisModelRun{
		ref:       ref,
		dropByYd:  map[int]float64{},
		windByYd:  map[int]float64{},
		velByYd:   map[int]float64{},
		refByYd:   map[int]coriolisRefRow{},
		sightInch: ref.sightIn,
	}
	for i, p := range data {
		yd := int(math.Round(p.TravelledDistance().In(unit.DistanceYard)))
		run.dropByYd[yd] = p.Drop().In(unit.DistanceInch)
		run.windByYd[yd] = p.Windage().In(unit.DistanceInch)
		run.velByYd[yd] = p.Velocity().In(unit.VelocityFPS)
		if i == 0 {
			run.muzzleIn = p.Drop().In(unit.DistanceInch)
		}
	}
	for _, r := range ref.rows {
		run.refByYd[int(math.Round(r.yards))] = r
	}
	return run
}

// TestCoriolisTrajectory mirrors the .NET TrajectoryCalculatorTest.CoriolisTrajectoryTest: it runs
// the same be_coriolis_* Ballistic Explorer captures (to 2000 yd through the transonic zone) and
// asserts Distance/Velocity within 0.5%, Drop within 0.4 MOA and Windage within 0.05 MOA — absolute,
// the same tolerances the C# test uses. (With the SI density model and matched integration step, Go's
// absolute long-range drop now tracks BE within the .NET window; the earlier swing-vs-baseline
// workaround is no longer needed.) It also checks the muzzle drop stays pinned to -sightHeight.
func TestCoriolisTrajectory(t *testing.T) {
	cases := []string{
		"be_coriolis_off",
		"be_coriolis_north",
		"be_coriolis_east",
		"be_coriolis_west",
		"be_coriolis_pole",
		"be_coriolis_south",
	}
	const velTolPct = 0.005 // distance & velocity, ±0.5%
	const dropTolMOA = 0.4
	const windTolMOA = 0.05

	for _, name := range cases {
		t.Run(name, func(t *testing.T) {
			run := runCoriolisCase(t, name)

			// Muzzle drop must stay exactly -sightHeight (Eötvös scales the fall, not raw ry).
			if math.Abs(run.muzzleIn-(-run.sightInch)) > 0.01 {
				t.Errorf("muzzle drop: got %.4f want %.4f", run.muzzleIn, -run.sightInch)
			}

			for yd, refRow := range run.refByYd {
				fyd := float64(yd)

				if math.Abs(run.velByYd[yd]-refRow.velFps) > refRow.velFps*velTolPct {
					t.Errorf("%d yd velocity: got %.1f want %.1f (±%.1f)", yd, run.velByYd[yd], refRow.velFps, refRow.velFps*velTolPct)
				}
				if math.Abs(run.dropByYd[yd]-refRow.dropIn) > inchTol(dropTolMOA, fyd) {
					t.Errorf("%d yd drop: got %.2f want %.2f (±%.2f)", yd, run.dropByYd[yd], refRow.dropIn, inchTol(dropTolMOA, fyd))
				}
				if math.Abs(run.windByYd[yd]-refRow.windIn) > inchTol(windTolMOA, fyd) {
					t.Errorf("%d yd windage: got %.3f want %.3f (±%.3f)", yd, run.windByYd[yd], refRow.windIn, inchTol(windTolMOA, fyd))
				}
			}
		})
	}
}
