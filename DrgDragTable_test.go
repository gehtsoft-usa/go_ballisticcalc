package externalballistics_test

import (
	"math"
	"testing"

	externalballistics "github.com/gehtsoft-usa/go_ballisticcalc"
	"github.com/gehtsoft-usa/go_ballisticcalc/bmath/unit"
)

// buildFlat builds a custom drag table from a base table with a single flat BC knot of value k.
func buildFlat(t *testing.T, base byte, k float64) externalballistics.DrgDragTable {
	t.Helper()
	table, err := externalballistics.BuildDrgDragTable(base, []externalballistics.BcAtMach{
		externalballistics.CreateBcAtMach(0, k),
	})
	if err != nil {
		t.Fatalf("BuildDrgDragTable: %v", err)
	}
	return table
}

// TestDrgFlatBcScalesBaseCurve checks Cd_custom(M) = Cd_base(M) / BC(M): a flat-k table's drag
// coefficients are the base curve's divided by k, on the base curve's own Mach grid. A flat-1.0
// table reproduces the base curve exactly, so it stands in for Cd_base.
func TestDrgFlatBcScalesBaseCurve(t *testing.T) {
	base := buildFlat(t, externalballistics.DragTableG1, 1.0)
	scaled := buildFlat(t, externalballistics.DragTableG1, 0.5)

	bp := base.DataPoints()
	sp := scaled.DataPoints()
	if len(bp) != len(sp) {
		t.Fatalf("point count mismatch: base %d scaled %d", len(bp), len(sp))
	}
	for i := range bp {
		if bp[i].A != sp[i].A {
			t.Errorf("point %d mach mismatch: base %.4f scaled %.4f", i, bp[i].A, sp[i].A)
		}
		if math.Abs(sp[i].B*0.5-bp[i].B) > 1e-12 {
			t.Errorf("point %d: scaled*0.5=%.8f base=%.8f", i, sp[i].B*0.5, bp[i].B)
		}
	}

	// Spot-check that flat-1.0 reproduces known G1 base coefficients.
	known := map[float64]float64{0.00: 0.2629, 0.50: 0.2032, 1.00: 0.4805}
	for _, p := range bp {
		if want, ok := known[p.A]; ok {
			if math.Abs(p.B-want) > 1e-9 {
				t.Errorf("G1 base at mach %.2f: got %.4f want %.4f", p.A, p.B, want)
			}
		}
	}
}

// TestDrgBcInterpolation checks the piecewise-linear BC(M) with flat extrapolation, by comparing a
// two-knot table against a flat-1.0 (base) table: Cd_custom(M) == Cd_base(M) / BC(M).
func TestDrgBcInterpolation(t *testing.T) {
	base := buildFlat(t, externalballistics.DragTableG1, 1.0)
	table, err := externalballistics.BuildDrgDragTable(externalballistics.DragTableG1, []externalballistics.BcAtMach{
		externalballistics.CreateBcAtMach(1.0, 0.2),
		externalballistics.CreateBcAtMach(2.0, 0.4),
	})
	if err != nil {
		t.Fatalf("BuildDrgDragTable: %v", err)
	}

	expectedBc := func(m float64) float64 {
		switch {
		case m <= 1.0:
			return 0.2
		case m >= 2.0:
			return 0.4
		default:
			return 0.2 + (m-1.0)*0.2
		}
	}

	bp := base.DataPoints()
	tp := table.DataPoints()
	for i := range tp {
		want := bp[i].B / expectedBc(tp[i].A)
		if math.Abs(tp[i].B-want) > 1e-12 {
			t.Errorf("mach %.3f: Cd got %.8f want %.8f", tp[i].A, tp[i].B, want)
		}
	}
}

// TestDrgFlatBcReproducesStandardTrajectory is the identity check: a flat-BC custom (GC) table run
// with BC 1.0 reproduces the standard-table trajectory run with that BC. Config mirrors the .NET
// DrgDragTableFactoryTest.FlatBc_ReproducesStandardTrajectory.
func TestDrgFlatBcReproducesStandardTrajectory(t *testing.T) {
	const k = 0.475
	stdBc, err := externalballistics.CreateBallisticCoefficient(k, externalballistics.DragTableG7)
	if err != nil {
		t.Fatalf("CreateBallisticCoefficient: %v", err)
	}
	table := buildFlat(t, externalballistics.DragTableG7, k)
	gcBc, err := table.BallisticCoefficient(1.0, externalballistics.BC)
	if err != nil {
		t.Fatalf("BallisticCoefficient: %v", err)
	}

	run := func(b externalballistics.BallisticCoefficient) []externalballistics.TrajectoryData {
		projectile := externalballistics.CreateProjectile(b, unit.MustCreateWeight(168, unit.WeightGrain))
		ammo := externalballistics.CreateAmmunition(projectile, unit.MustCreateVelocity(2700, unit.VelocityFPS))
		zero := externalballistics.CreateZeroInfo(unit.MustCreateDistance(100, unit.DistanceYard))
		weapon := externalballistics.CreateWeapon(unit.MustCreateDistance(1.5, unit.DistanceInch), zero)
		atmo := externalballistics.CreateDefaultAtmosphere()
		calc := externalballistics.CreateTrajectoryCalculator()
		sa := calc.SightAngle(ammo, weapon, atmo)
		shot := externalballistics.CreateShotParameters(sa,
			unit.MustCreateDistance(1000, unit.DistanceYard),
			unit.MustCreateDistance(100, unit.DistanceYard))
		return calc.Trajectory(ammo, weapon, atmo, shot, nil)
	}

	std := run(stdBc)
	gc := run(gcBc)
	if len(std) != len(gc) {
		t.Fatalf("length mismatch: std %d gc %d", len(std), len(gc))
	}
	for i := range std {
		dv := math.Abs(std[i].Velocity().In(unit.VelocityFPS) - gc[i].Velocity().In(unit.VelocityFPS))
		dd := math.Abs(std[i].Drop().In(unit.DistanceInch) - gc[i].Drop().In(unit.DistanceInch))
		if dv > 0.05 {
			t.Errorf("point %d velocity delta %.4f fps > 0.05", i, dv)
		}
		if dd > 0.02 {
			t.Errorf("point %d drop delta %.4f in > 0.02", i, dd)
		}
	}
}

func TestDrgBuildGuards(t *testing.T) {
	okKnots := []externalballistics.BcAtMach{externalballistics.CreateBcAtMach(0, 0.5)}

	if _, err := externalballistics.BuildDrgDragTable(externalballistics.DragTableGC, okKnots); err == nil {
		t.Error("expected error for GC base table")
	}
	if _, err := externalballistics.BuildDrgDragTable(externalballistics.DragTableG1, nil); err == nil {
		t.Error("expected error for empty knots")
	}
	badKnots := []externalballistics.BcAtMach{externalballistics.CreateBcAtMach(1.0, 0)}
	if _, err := externalballistics.BuildDrgDragTable(externalballistics.DragTableG1, badKnots); err == nil {
		t.Error("expected error for non-positive BC")
	}
	// CreateBallisticCoefficient must reject GC (previously panicked).
	if _, err := externalballistics.CreateBallisticCoefficient(1.0, externalballistics.DragTableGC); err == nil {
		t.Error("expected error creating a standard BC with the GC table")
	}
}

// TestDrgOpenFile reads a real .drg (BRL/CFM) file and checks the points and metadata.
func TestDrgOpenFile(t *testing.T) {
	table, err := externalballistics.OpenDrgDragTable("testdata/drg/sierra_168_brl.drg")
	if err != nil {
		t.Fatalf("OpenDrgDragTable: %v", err)
	}
	points := table.DataPoints()
	if len(points) < 2 {
		t.Fatalf("expected multiple points, got %d", len(points))
	}
	// Points must be sorted ascending by Mach.
	for i := 1; i < len(points); i++ {
		if points[i].A < points[i-1].A {
			t.Errorf("points not sorted at %d: %.3f < %.3f", i, points[i].A, points[i-1].A)
		}
	}
	if !table.HasMetadata() {
		t.Fatal("expected metadata from the BRL header")
	}
	// weightKg 0.01089 ~= 168 gr; diameterM 0.00782 ~= 0.308 in
	if got := table.Weight().In(unit.WeightGrain); math.Abs(got-168) > 1 {
		t.Errorf("weight: got %.2f gr want ~168", got)
	}
	if got := table.Diameter().In(unit.DistanceInch); math.Abs(got-0.308) > 0.002 {
		t.Errorf("diameter: got %.4f in want ~0.308", got)
	}

	// The loaded table must be usable as a ballistic coefficient.
	if _, err := table.BallisticCoefficient(1.0, externalballistics.BC); err != nil {
		t.Errorf("BallisticCoefficient from loaded table: %v", err)
	}
}
