package externalballistics

import (
	"math"

	"github.com/gehtsoft-usa/go_ballisticcalc/bmath/unit"
	"github.com/gehtsoft-usa/go_ballisticcalc/bmath/vector"
)

const cZeroFindingAccuracy float64 = 0.000005
const cMinimumVelocity float64 = 50.0
const cMaximumDrop float64 = -15000
const cMaxIterations int = 10
const cGravityConstant float64 = 32.17405
const cCoriolisOmega float64 = 7.2921159e-5 // Earth rotation rate, rad/s (§1.7)

// TrajectoryCalculator table is used to calculate the trajectory of a projectile shot with the parameters specified
type TrajectoryCalculator struct {
	maximumCalculatorStepSize unit.Distance
}

// MaximumCalculatorStepSize returns the maximum size of one calculation iteration.
func (v TrajectoryCalculator) MaximumCalculatorStepSize() unit.Distance {
	return v.maximumCalculatorStepSize
}

// SetMaximumCalculatorStepSize sets the maximum size of one calculation iteration.
//
// As the generic rule, the maximum step of the calculation must not be greater than
// a half of the step used in the short parameter. The smaller value is, the calculation is more precise but
// takes more time to calculate. From practical standpoint the value in range from 0.5 to 5 feet produces
// good enough accuracy.
func (v *TrajectoryCalculator) SetMaximumCalculatorStepSize(x unit.Distance) {
	v.maximumCalculatorStepSize = x
}

func (v TrajectoryCalculator) getCalculationStep(step float64) float64 {
	step = step / 2 //do it twice for increased accuracy of velocity calculation and 10 times per step
	var maximumStep = v.maximumCalculatorStepSize.In(unit.DistanceFoot)

	if step > maximumStep {
		var stepOrder = int(math.Floor(math.Log10(step)))
		var maximumOrder = int(math.Floor(math.Log10(maximumStep)))

		step = step / math.Pow(10, float64(stepOrder-maximumOrder+1))
	}
	return step
}

// CreateTrajectoryCalculator creates and instance of the trajectory calculator
func CreateTrajectoryCalculator() TrajectoryCalculator {
	return TrajectoryCalculator{
		// 0.1 m matches the .NET engine's default (MaximumCalculationStepSize). The old 1 ft default
		// made the derived integration step ~10x coarser at a 50 yd output step (getCalculationStep's
		// log10 flooring is scale-sensitive), which overshot reported distances and cost long-range
		// accuracy relative to .NET.
		maximumCalculatorStepSize: unit.MustCreateDistance(0.1, unit.DistanceMeter),
	}
}

// SightAngle calculates the sight angle for a rifle with scope height specified and zeroed using the ammo specified at
// the range specified and under the conditions (atmosphere) specified.
//
// The calculated value is to be used as sightAngle parameter of the ShotParameters structure
func (v TrajectoryCalculator) SightAngle(ammunition Ammunition, weapon Weapon, atmosphere Atmosphere) unit.Angular {
	var calculationStep = v.getCalculationStep(unit.MustCreateDistance(10, weapon.Zero().ZeroDistance().Units()).In(unit.DistanceFoot))

	var deltaRangeVector, rangeVector, velocityVector, gravityVector vector.Vector
	var muzzleVelocity, velocity, barrelAzimuth, barrelElevation float64
	var densityFactor, mach, drag, zeroFindingError float64
	var time, deltaTime float64
	var maximumRange float64

	mach = atmosphere.Mach().In(unit.VelocityFPS)
	densityFactor = atmosphere.getDensityFactor()
	muzzleVelocity = ammunition.MuzzleVelocity().In(unit.VelocityFPS)
	barrelAzimuth = 0.0
	barrelElevation = 0

	zeroFindingError = cZeroFindingAccuracy * 2
	var iterationsCount int
	var bullet = ammunition.Bullet()
	var ballisticFactor = 1 / bullet.GetBallisticCoefficient()

	gravityVector = vector.Create(0, -cGravityConstant, 0)
	for zeroFindingError > cZeroFindingAccuracy && iterationsCount < cMaxIterations {
		velocity = muzzleVelocity
		time = 0.0

		//x - distance towards target,
		//y - drop and
		//z - windage
		rangeVector = vector.Create(0.0, -weapon.SightHeight().In(unit.DistanceFoot), 0)
		velocityVector = vector.Create(math.Cos(barrelElevation)*math.Cos(barrelAzimuth), math.Sin(barrelElevation), math.Cos(barrelElevation)*math.Sin(barrelAzimuth)).MultiplyByConst(velocity)
		var zeroDistance = weapon.Zero().ZeroDistance().In(unit.DistanceFoot)
		maximumRange = zeroDistance + calculationStep

		for rangeVector.X <= maximumRange {
			if velocity < cMinimumVelocity || rangeVector.Y < cMaximumDrop {
				break
			}

			deltaTime = calculationStep / velocityVector.X
			velocity = velocityVector.Magnitude()
			drag = ballisticFactor * densityFactor * velocity * bullet.BallisticCoefficient().Drag(velocity/mach)
			velocityVector = velocityVector.Subtract((velocityVector.MultiplyByConst(drag).Subtract(gravityVector)).MultiplyByConst(deltaTime))
			deltaRangeVector = vector.Create(velocityVector.X*deltaTime, velocityVector.Y*deltaTime, velocityVector.Z*deltaTime)
			rangeVector = rangeVector.Add(deltaRangeVector)
			velocity = velocityVector.Magnitude()
			time = time + deltaRangeVector.Magnitude()/velocity

			if math.Abs(rangeVector.X-zeroDistance) < 0.5*calculationStep {
				zeroFindingError = math.Abs(rangeVector.Y)
				barrelElevation = barrelElevation - rangeVector.Y/rangeVector.X
				break
			}
		}
		iterationsCount++
	}
	return unit.MustCreateAngular(barrelElevation, unit.AngularRadian)
}

// Trajectory calculates the trajectory with the parameters specified
func (v TrajectoryCalculator) Trajectory(ammunition Ammunition, weapon Weapon, atmosphere Atmosphere, shotInfo ShotParameters, windInfo []WindInfo) []TrajectoryData {
	var rangeTo = shotInfo.MaximumDistance().In(unit.DistanceFoot)
	var step = shotInfo.Step().In(unit.DistanceFoot)

	var calculationStep = v.getCalculationStep(step)

	var deltaRangeVector, rangeVector, velocityAdjusted, velocityVector, windVector, gravityVector vector.Vector
	var muzzleVelocity, velocity, barrelAzimuth, barrelElevation float64
	var densityFactor, mach, drag float64
	var time, deltaTime float64
	var maximumRange, nextRangeDistance float64
	var bulletWeight = ammunition.Bullet().BulletWeight().In(unit.WeightGrain)
	var stabilityCoefficient = 1.0
	var calculateDrift bool

	if weapon.HasTwist() && ammunition.Bullet().HasDimensions() {
		stabilityCoefficient = calculateStabilityCoefficient(ammunition, weapon, atmosphere)
		calculateDrift = true
	}

	var rangesLength = int(math.Floor(rangeTo/step)) + 1
	var ranges = make([]TrajectoryData, rangesLength)

	barrelAzimuth = 0.0
	barrelElevation = shotInfo.SightAngle().In(unit.AngularRadian)

	var lineOfSight = shotInfo.ShotAngle().In(unit.AngularRadian)
	var hasShotAngle = shotInfo.ShotAngle().In(unit.AngularRadian) != 0
	if hasShotAngle {
		barrelElevation = barrelElevation + shotInfo.ShotAngle().In(unit.AngularRadian)
	}

	var lineOfSightTan = math.Tan(lineOfSight)
	var lineOfDepartureTan = math.Tan(barrelElevation)
	var lineOfSightCos = math.Cos(lineOfSight)
	var lineOfSightSin = math.Sin(lineOfSight)

	var alt0 = atmosphere.Altitude().In(unit.DistanceFoot)
	var currentWind int
	var nextWindRange = 1e7

	if len(windInfo) < 1 {
		windVector = vector.Create(0, 0, 0)
	} else {
		if len(windInfo) > 1 {
			nextWindRange = windInfo[0].untilDistance.In(unit.DistanceFoot)
		}
		windVector = windToVector(shotInfo, windInfo[0])
	}

	muzzleVelocity = ammunition.MuzzleVelocity().In(unit.VelocityFPS)
	gravityVector = vector.Create(0, -cGravityConstant, 0)
	velocity = muzzleVelocity
	time = 0.0

	//x - distance towards target,
	//y - drop and
	//z - windage
	rangeVector = vector.Create(0.0, -weapon.SightHeight().In(unit.DistanceFoot), 0)
	velocityVector = vector.Create(math.Cos(barrelElevation)*math.Cos(barrelAzimuth), math.Sin(barrelElevation), math.Cos(barrelElevation)*math.Sin(barrelAzimuth)).MultiplyByConst(velocity)

	var currentItem int
	maximumRange = rangeTo
	nextRangeDistance = 0
	var distance = 0.0 // slant distance along line of sight

	var twistCoefficient float64

	if calculateDrift {
		if weapon.Twist().Direction() == TwistLeft {
			twistCoefficient = 1
		} else {
			twistCoefficient = -1
		}
	}

	var bullet = ammunition.Bullet()
	var ballisticFactor = 1 / bullet.GetBallisticCoefficient()

	// §1.7 Coriolis / Eötvös: closed-form per-output-point corrections (Earth rotation), applied only
	// when a latitude is provided. Time-of-flight, velocity and range are untouched. Azimuth is a pure
	// scalar here (0° = North, clockwise → 90° = East); it does not tilt the muzzle vector.
	var coriolis = shotInfo.HasLatitude()
	var coriolisHCoef float64
	var coriolisVRatio = 1.0
	if coriolis {
		var sinLat = math.Sin(shotInfo.Latitude().In(unit.AngularRadian))
		var cosLat = math.Cos(shotInfo.Latitude().In(unit.AngularRadian))
		var azSin = math.Sin(shotInfo.Azimuth().In(unit.AngularRadian))
		// Horizontal (windage) coefficient, azimuth-independent (sign carried by sin(latitude)).
		coriolisHCoef = cCoriolisOmega * sinLat
		// Vertical Eötvös as a gravity ratio g_eff/g (dimensionless). muzzleVelocity is fps and
		// cGravityConstant is ft/s², so muzzleVelocity/cGravityConstant is seconds — units cancel.
		coriolisVRatio = 1.0 - 2.0*cCoriolisOmega*cosLat*azSin*muzzleVelocity/cGravityConstant
	}

	// Cache atmosphere calculations - only update when altitude changes by > 1 meter
	var altDelta = 3.28084             // 1 meter in feet
	var lastAtAltitude = -3280839895.0 // -1000000000 meters in feet
	var alt = alt0

	// Initialize density and mach for first iteration
	densityFactor, mach = atmosphere.getDensityFactorAndMachForAltitude(alt)
	lastAtAltitude = alt

	//run all the way down the range
	// §1.3: `distance` is the line-of-sight (slant) distance rangeVector.X/cos(losAngle), so the
	// one-step end margin must be in the same frame or steep shots stop one output step early.
	for distance <= maximumRange+calculationStep/lineOfSightCos {
		if velocity < cMinimumVelocity || rangeVector.Y < cMaximumDrop {
			break
		}

		// Update density and mach only when altitude changes by more than altDelta
		if math.Abs(lastAtAltitude-alt) > altDelta {
			densityFactor, mach = atmosphere.getDensityFactorAndMachForAltitude(alt)
			lastAtAltitude = alt
		}

		if distance >= nextWindRange {
			currentWind++
			windVector = windToVector(shotInfo, windInfo[currentWind])

			if currentWind == len(windInfo)-1 {
				nextWindRange = 1e7
			} else {
				nextWindRange = windInfo[currentWind].untilDistance.In(unit.DistanceFoot)
			}
		}

		if distance >= nextRangeDistance {
			var windage = rangeVector.Z
			if calculateDrift {
				// §1.5: spin drift is horizontal; on an inclined shot only its cos-projection lies
				// in the reported windage plane, hence the * lineOfSightCos factor.
				windage += (1.25 * (stabilityCoefficient + 1.2) * math.Pow(time, 1.83) * twistCoefficient) / 12.0 * lineOfSightCos
			}

			// Calculate line of sight and line of departure elevations
			var sightHeight = weapon.SightHeight().In(unit.DistanceFoot)
			var lineOfSightElevation = rangeVector.X * lineOfSightTan
			var lineOfDepartureElevation = rangeVector.X*lineOfDepartureTan - sightHeight

			// §1.7 Coriolis / Eötvös. Horizontal deflects right in the N hemisphere; our windage is
			// left +/right − so subtract. Eötvös modifies gravity, which acts only on the gravitational
			// fall below the vacuum bore line (== lineOfDepartureElevation), so scale that fall by
			// g_eff/g and rebuild; the bore line (hence the exact −sightHeight muzzle drop where the
			// fall is 0) is left untouched. The integrator state (rangeVector.Y) is never mutated.
			var ryEffective = rangeVector.Y
			if coriolis {
				windage -= coriolisHCoef * distance * time
				var boreLine = lineOfDepartureElevation
				ryEffective = boreLine + (rangeVector.Y-boreLine)*coriolisVRatio
			}

			// Calculate drop and adjustments
			var drop = ryEffective
			var dropFlat = ryEffective
			var dropAdjustment float64
			var windageAdjustment float64

			// If shot angle is present, rotate coordinates to account for incline/decline.
			// §1.6 (intentional quirk — do NOT "simplify"): sightHeight is added in the vertical
			// frame but subtracted back in the rotated frame; the residual sightHeight*(cos-1)
			// deliberately pins the muzzle drop to exactly -sightHeight while every downrange point
			// uses the true perpendicular rotation.
			if hasShotAngle {
				var y = ryEffective + sightHeight
				var yRotated = -rangeVector.X*lineOfSightSin + y*lineOfSightCos
				drop = yRotated - sightHeight
				dropAdjustment = getCorrection(distance, drop)
				windageAdjustment = getCorrection(distance, windage)
			} else {
				dropAdjustment = getCorrection(rangeVector.X, drop)
				windageAdjustment = getCorrection(rangeVector.X, windage)
			}
			ranges[currentItem] = TrajectoryData{
				time:                     Timespan{time: time},
				travelDistance:           unit.MustCreateDistance(distance, unit.DistanceFoot),
				travelDistanceFlat:       unit.MustCreateDistance(rangeVector.X, unit.DistanceFoot),
				drop:                     unit.MustCreateDistance(drop, unit.DistanceFoot),
				dropFlat:                 unit.MustCreateDistance(dropFlat, unit.DistanceFoot),
				dropAdjustment:           unit.MustCreateAngular(dropAdjustment, unit.AngularRadian),
				lineOfSightElevation:     unit.MustCreateDistance(lineOfSightElevation, unit.DistanceFoot),
				lineOfDepartureElevation: unit.MustCreateDistance(lineOfDepartureElevation, unit.DistanceFoot),
				windage:                  unit.MustCreateDistance(windage, unit.DistanceFoot),
				windageAdjustment:        unit.MustCreateAngular(windageAdjustment, unit.AngularRadian),
				velocity:                 unit.MustCreateVelocity(velocity, unit.VelocityFPS),
				mach:                     velocity / mach,
				energy:                   unit.MustCreateEnergy(calculateEnergy(bulletWeight, velocity), unit.EnergyFootPound),
				optimalGameWeight:        unit.MustCreateWeight(calculateOgv(bulletWeight, velocity), unit.WeightPound),
			}
			nextRangeDistance += step
			currentItem++
			if currentItem == len(ranges) {
				break
			}
		}

		velocityAdjusted = velocityVector.Subtract(windVector)
		velocity = velocityAdjusted.Magnitude()
		deltaTime = calculationStep / velocity
		drag = ballisticFactor * densityFactor * velocity * bullet.BallisticCoefficient().Drag(velocity/mach)
		velocityVector = velocityVector.Subtract(velocityAdjusted.MultiplyByConst(drag * deltaTime).Subtract(gravityVector.MultiplyByConst(deltaTime)))

		deltaRangeVector = vector.Create(velocityVector.X*deltaTime,
			velocityVector.Y*deltaTime,
			velocityVector.Z*deltaTime)

		rangeVector = rangeVector.Add(deltaRangeVector)
		distance = rangeVector.X / lineOfSightCos // Calculate slant distance
		alt += deltaRangeVector.Y
		velocity = velocityVector.Magnitude()
		// §1.4: reported flight time is the sum of the integration timesteps, not re-derived from
		// displacement/speed (which diverges on angled shots). deltaTime is this step's dt.
		time = time + deltaTime

	}
	return ranges
}

func calculateStabilityCoefficient(ammunitionInfo Ammunition, rifleInfo Weapon, atmosphere Atmosphere) float64 {
	var weight = ammunitionInfo.Bullet().BulletWeight().In(unit.WeightGrain)
	var diameter = ammunitionInfo.Bullet().BulletDiameter().In(unit.DistanceInch)
	var twist = rifleInfo.Twist().Twist().In(unit.DistanceInch) / diameter
	var length = ammunitionInfo.Bullet().BulletLength().In(unit.DistanceInch) / diameter
	var sd = 30 * weight / (math.Pow(twist, 2) * math.Pow(diameter, 3) * length * (1 + math.Pow(length, 2)))
	var fv = math.Pow(ammunitionInfo.MuzzleVelocity().In(unit.VelocityFPS)/2800, 1.0/3.0)

	var ft = atmosphere.Temperature().In(unit.TemperatureFahrenheit)
	var pt = atmosphere.Pressure().In(unit.PressureInHg)
	var ftp = ((ft + 460) / (59 + 460)) * (29.92 / pt)

	return sd * fv * ftp
}

func windToVector(shot ShotParameters, wind WindInfo) vector.Vector {
	var shotAngle = shot.SightAngle().In(unit.AngularRadian)
	if shot.ShotAngle().In(unit.AngularRadian) != 0 {
		shotAngle += shot.ShotAngle().In(unit.AngularRadian)
	}
	var sightCosine = math.Cos(shotAngle)
	var sightSine = math.Sin(shotAngle)
	var cantCosine = math.Cos(shot.CantAngle().In(unit.AngularRadian))
	var cantSine = math.Sin(shot.CantAngle().In(unit.AngularRadian))
	var rangeVelocity = wind.velocity.In(unit.VelocityFPS) * math.Cos(wind.direction.In(unit.AngularRadian))
	var crossComponent = wind.velocity.In(unit.VelocityFPS) * math.Sin(wind.direction.In(unit.AngularRadian))
	var rangeFactor = -rangeVelocity * sightSine
	return vector.Create(rangeVelocity*sightCosine, rangeFactor*cantCosine+crossComponent*cantSine, crossComponent*cantCosine-rangeFactor*cantSine)
}

func getCorrection(distance, offset float64) float64 {
	return math.Atan(offset / distance)
}

func calculateEnergy(bulletWeight, velocity float64) float64 {
	return bulletWeight * math.Pow(velocity, 2) / 450400
}

func calculateOgv(bulletWeight, velocity float64) float64 {
	return math.Pow(bulletWeight, 2) * math.Pow(velocity, 3) * 1.5e-12
}
