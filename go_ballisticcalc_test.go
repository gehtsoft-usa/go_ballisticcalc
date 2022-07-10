package externalballistics_test

import (
	"math"
	"testing"

	externalballistics "github.com/gehtsoft-usa/go_ballisticcalc"
	"github.com/gehtsoft-usa/go_ballisticcalc/bmath/unit"
)

func TestZero1(t *testing.T) {
	bc, _ := externalballistics.CreateBallisticCoefficient(0.365, externalballistics.DragTableG1)
	projectile := externalballistics.CreateProjectile(bc, unit.MustCreateWeight(69, unit.WeightGrain))
	ammo := externalballistics.CreateAmmunition(projectile, unit.MustCreateVelocity(2600, unit.VelocityFPS))
	zero := externalballistics.CreateZeroInfo(unit.MustCreateDistance(100, unit.DistanceYard))
	weapon := externalballistics.CreateWeapon(unit.MustCreateDistance(3.2, unit.DistanceInch), zero)
	atmosphere := externalballistics.CreateDefaultAtmosphere()
	calc := externalballistics.CreateTrajectoryCalculator()

	sightAngle := calc.SightAngle(ammo, weapon, atmosphere)
	if math.Abs(sightAngle.In(unit.AngularRadian)-0.001651) > 1e-6 {
		t.Errorf("TestZero1 failed %.10f", sightAngle.In(unit.AngularRadian))
	}
}

func TestZero2(t *testing.T) {
	bc, _ := externalballistics.CreateBallisticCoefficient(0.223, externalballistics.DragTableG7)
	projectile := externalballistics.CreateProjectile(bc, unit.MustCreateWeight(168, unit.WeightGrain))
	ammo := externalballistics.CreateAmmunition(projectile, unit.MustCreateVelocity(2750, unit.VelocityFPS))
	zero := externalballistics.CreateZeroInfo(unit.MustCreateDistance(100, unit.DistanceYard))
	weapon := externalballistics.CreateWeapon(unit.MustCreateDistance(2, unit.DistanceInch), zero)
	atmosphere := externalballistics.CreateDefaultAtmosphere()

	calc := externalballistics.CreateTrajectoryCalculator()

	sightAngle := calc.SightAngle(ammo, weapon, atmosphere)
	if math.Abs(sightAngle.In(unit.AngularRadian)-0.001228) > 1e-6 {
		t.Errorf("TestZero1 failed %.10f", sightAngle.In(unit.AngularRadian))
	}
}

func assertEqual(t *testing.T, a, b, accuracy float64, name string) {
	if math.Abs(a-b) > accuracy {
		t.Errorf("Assertion %s failed (%f/%f)", name, a, b)
	}
}

func validateOneImperial(t *testing.T, data externalballistics.TrajectoryData,
	distance, velocity, mach, energy, path, hold, windage, windAdjustment, time, ogv float64,
	adjustmentUnit byte) {
	assertEqual(t, distance, data.TravelledDistance().In(unit.DistanceYard), 0.001, "Distance")
	assertEqual(t, velocity, data.Velocity().In(unit.VelocityFPS), 5, "Velocity")
	assertEqual(t, mach, data.MachVelocity(), 0.005, "Mach")
	assertEqual(t, energy, data.Energy().In(unit.EnergyFootPound), 5, "Energy")
	assertEqual(t, time, data.Time().TotalSeconds(), 0.06, "Time")
	assertEqual(t, ogv, data.OptimalGameWeight().In(unit.WeightPound), 1, "OGV")

	if distance >= 800 {
		assertEqual(t, path, data.Drop().In(unit.DistanceInch), 4, "Drop")
	} else if distance >= 500 {
		assertEqual(t, path, data.Drop().In(unit.DistanceInch), 1, "Drop")
	} else {
		assertEqual(t, path, data.Drop().In(unit.DistanceInch), 0.5, "Drop")
	}

	if distance > 1 {
		assertEqual(t, hold, data.DropAdjustment().In(adjustmentUnit), 0.5, "Hold")
	}

	if distance >= 800 {
		assertEqual(t, windage, data.Windage().In(unit.DistanceInch), 1.5, "Windage")
	} else if distance >= 500 {
		assertEqual(t, windage, data.Windage().In(unit.DistanceInch), 1, "Windage")
	} else {
		assertEqual(t, windage, data.Windage().In(unit.DistanceInch), 0.5, "Windage")
	}

	if distance > 1 {
		assertEqual(t, windAdjustment, data.WindageAdjustment().In(adjustmentUnit), 0.5, "WAdj")
	}
}

func TestPathG1(t *testing.T) {
	bc, _ := externalballistics.CreateBallisticCoefficient(0.223, externalballistics.DragTableG1)
	projectile := externalballistics.CreateProjectile(bc, unit.MustCreateWeight(168, unit.WeightGrain))
	ammo := externalballistics.CreateAmmunition(projectile, unit.MustCreateVelocity(2750, unit.VelocityFPS))
	zero := externalballistics.CreateZeroInfo(unit.MustCreateDistance(100, unit.DistanceYard))
	weapon := externalballistics.CreateWeapon(unit.MustCreateDistance(2, unit.DistanceInch), zero)
	atmosphere := externalballistics.CreateDefaultAtmosphere()
	shotInfo := externalballistics.CreateShotParameters(unit.MustCreateAngular(0.001228, unit.AngularRadian),
		unit.MustCreateDistance(1000, unit.DistanceYard),
		unit.MustCreateDistance(100, unit.DistanceYard))
	wind := externalballistics.CreateOnlyWindInfo(unit.MustCreateVelocity(5, unit.VelocityMPH),
		unit.MustCreateAngular(-45, unit.AngularDegree))

	calc := externalballistics.CreateTrajectoryCalculator()
	data := calc.Trajectory(ammo, weapon, atmosphere, shotInfo, wind)

	assertEqual(t, float64(len(data)), 11, 0.1, "Length")

	validateOneImperial(t, data[0], 0, 2750, 2.463, 2820.6, -2, 0, 0, 0, 0, 880, unit.AngularMOA)
	validateOneImperial(t, data[1], 100, 2351.2, 2.106, 2061, 0, 0, -0.6, -0.6, 0.118, 550, unit.AngularMOA)
	validateOneImperial(t, data[5], 500, 1169.1, 1.047, 509.8, -87.9, -16.8, -19.5, -3.7, 0.857, 67, unit.AngularMOA)
	validateOneImperial(t, data[10], 1000, 776.4, 0.695, 224.9, -823.9, -78.7, -87.5, -8.4, 2.495, 20, unit.AngularMOA)
}

func TestPathG7(t *testing.T) {
	bc, _ := externalballistics.CreateBallisticCoefficient(0.223, externalballistics.DragTableG7)
	projectile := externalballistics.CreateProjectileWithDimensions(bc, unit.MustCreateDistance(0.308, unit.DistanceInch),
		unit.MustCreateDistance(1.282, unit.DistanceInch), unit.MustCreateWeight(168, unit.WeightGrain))
	ammo := externalballistics.CreateAmmunition(projectile, unit.MustCreateVelocity(2750, unit.VelocityFPS))
	zero := externalballistics.CreateZeroInfo(unit.MustCreateDistance(100, unit.DistanceYard))
	twist := externalballistics.CreateTwist(externalballistics.TwistRight, unit.MustCreateDistance(11.24, unit.DistanceInch))
	weapon := externalballistics.CreateWeaponWithTwist(unit.MustCreateDistance(2, unit.DistanceInch), zero, twist)
	atmosphere := externalballistics.CreateDefaultAtmosphere()
	shotInfo := externalballistics.CreateShotParameters(unit.MustCreateAngular(4.221, unit.AngularMOA),
		unit.MustCreateDistance(1000, unit.DistanceYard),
		unit.MustCreateDistance(100, unit.DistanceYard))
	wind := externalballistics.CreateOnlyWindInfo(unit.MustCreateVelocity(5, unit.VelocityMPH),
		unit.MustCreateAngular(-45, unit.AngularDegree))

	calc := externalballistics.CreateTrajectoryCalculator()
	data := calc.Trajectory(ammo, weapon, atmosphere, shotInfo, wind)

	assertEqual(t, float64(len(data)), 11, 0.1, "Length")

	validateOneImperial(t, data[0], 0, 2750, 2.463, 2820.6, -2, 0, 0, 0, 0, 880, unit.AngularMil)
	validateOneImperial(t, data[1], 100, 2544.3, 2.279, 2416, 0, 0, -0.35, -0.09, 0.113, 698, unit.AngularMil)
	validateOneImperial(t, data[5], 500, 1810.7, 1.622, 1226, -56.3, -3.18, -9.96, -0.55, 0.673, 252, unit.AngularMil)
	validateOneImperial(t, data[10], 1000, 1081.3, 0.968, 442, -401.6, -11.32, -50.98, -1.44, 1.748, 55, unit.AngularMil)
}

func TestAmmunictionReturnBC(t *testing.T) {
	bc, _ := externalballistics.CreateBallisticCoefficient(0.223, externalballistics.DragTableG7)
	var projectile = externalballistics.CreateProjectile(bc, unit.MustCreateWeight(69, unit.WeightGrain))
	assertEqual(t, bc.Value(), 0.223, 0.0005, "BC")
	assertEqual(t, projectile.GetBallisticCoefficient(), 0.223, 0.0005, "BC Calculated")
}

func TestAmmunictionReturnFF(t *testing.T) {
	bc, _ := externalballistics.CreateBallisticCoefficientForCustomDragFunction(1.184, externalballistics.FF,
		func(mach float64) float64 {
			return 0
		})
	var projectile = externalballistics.CreateProjectileWithDimensions(bc, unit.MustCreateDistance(0.204, unit.DistanceInch), unit.MustCreateDistance(1, unit.DistanceInch), unit.MustCreateWeight(40, unit.WeightGrain))
	assertEqual(t, bc.Value(), 1.184, 0.0005, "ff")
	assertEqual(t, projectile.GetBallisticCoefficient(), 0.116, 0.0005, "BC Calculated")
}

var customTable = []externalballistics.DataPoint{
	{B: 0.119, A: 0},
	{B: 0.119, A: 0.7},
	{B: 0.12, A: 0.85},
	{B: 0.122, A: 0.87},
	{B: 0.126, A: 0.9},
	{B: 0.148, A: 0.93},
	{B: 0.182, A: 0.95},
}

var customCurve = externalballistics.CalculateCurve(customTable)

func customDragFunction(mach float64) float64 {
	return externalballistics.CalculateByCurve(customTable, customCurve, mach)
}

func validateOneMetric(t *testing.T, data externalballistics.TrajectoryData, distance float64, drop float64, velocity float64, time float64) {
	assertEqual(t, distance, data.TravelledDistance().In(unit.DistanceMeter), 0.1, "Distance")
	//be accurate within 1/3 of moa
	var vac = unit.MustCreateAngular(0.3, unit.AngularMOA).In(unit.AngularCmPer100M) * distance / 100
	assertEqual(t, drop, data.Drop().In(unit.DistanceCentimeter), vac, "Drop")
	assertEqual(t, velocity, data.Velocity().In(unit.VelocityMPS), 5, "Velocity")
	assertEqual(t, time, data.Time().TotalSeconds(), 0.05, "Time")
}

func TestCustomCurve(t *testing.T) {
	bc, _ := externalballistics.CreateBallisticCoefficientForCustomDragFunction(1, externalballistics.FF, customDragFunction)
	var projectile = externalballistics.CreateProjectileWithDimensions(bc, unit.MustCreateDistance(119.56, unit.DistanceMillimeter), unit.MustCreateDistance(20, unit.DistanceInch), unit.MustCreateWeight(13585, unit.WeightGram))
	var ammo = externalballistics.CreateAmmunition(projectile, unit.MustCreateVelocity(555, unit.VelocityMPS))
	var zero = externalballistics.CreateZeroInfo(unit.MustCreateDistance(100, unit.DistanceMeter))
	var weapon = externalballistics.CreateWeapon(unit.MustCreateDistance(40, unit.DistanceMillimeter), zero)
	var atmosphere = externalballistics.CreateDefaultAtmosphere()

	var calc = externalballistics.CreateTrajectoryCalculator()
	var sightAngle = calc.SightAngle(ammo, weapon, atmosphere)
	var shotInfo = externalballistics.CreateShotParameters(sightAngle, unit.MustCreateDistance(1500, unit.DistanceMeter), unit.MustCreateDistance(100, unit.DistanceMeter))
	var data = calc.Trajectory(ammo, weapon, atmosphere, shotInfo, nil)
	validateOneMetric(t, data[1], 100, 0, 550, 0.182)
	validateOneMetric(t, data[2], 200, -28.4, 544, 0.364)
	validateOneMetric(t, data[15], 1500, -3627.8, 486, 2.892)
}
