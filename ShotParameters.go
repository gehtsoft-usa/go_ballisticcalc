package externalballistics

import "github.com/gehtsoft-usa/go_ballisticcalc/bmath/unit"

//ShotParameters struct keeps parameters of the shot to be calculated
type ShotParameters struct {
	sightAngle      unit.Angular
	shotAngle       unit.Angular
	cantAngle       unit.Angular
	maximumDistance unit.Distance
	step            unit.Distance
	azimuth         unit.Angular
	latitude        unit.Angular
	hasLatitude     bool
}

//CreateShotParameters creates parameters of the shot
//
//sightAngle - is the angle between scope centerline and the barrel centerline
func CreateShotParameters(sightAngle unit.Angular, maxDistance unit.Distance, step unit.Distance) ShotParameters {
	return ShotParameters{
		sightAngle:      sightAngle,
		shotAngle:       unit.MustCreateAngular(0, unit.AngularRadian),
		cantAngle:       unit.MustCreateAngular(0, unit.AngularRadian),
		maximumDistance: maxDistance,
		step:            step,
	}
}

//SightAngle returns the angle of the sight
func (v ShotParameters) SightAngle() unit.Angular {
	return v.sightAngle
}

//ShotAngle returns the angle of the short
func (v ShotParameters) ShotAngle() unit.Angular {
	return v.shotAngle
}

//CantAngle returns the cant angle (the angle between centers of scope and the barrel projection and zenith line)
func (v ShotParameters) CantAngle() unit.Angular {
	return v.cantAngle
}

//MaximumDistance returns the maximum distance to be calculated
func (v ShotParameters) MaximumDistance() unit.Distance {
	return v.maximumDistance
}

//Step returns the step between calculation results
func (v ShotParameters) Step() unit.Distance {
	return v.step
}

//CreateShotParameterUnlevel creates the parameter of the shot aimed at the target which is not on th same level
//as the shooter
//
//sightAngle - is the angle between scope centerline and the barrel centerline
//
//shotAngle - is the angle between lines drawn from the shooter to the target and the horizon. The positive angle
//means that the target is higher and the negative angle means that the target is lower
func CreateShotParameterUnlevel(sightAngle unit.Angular, maxDistance unit.Distance, step unit.Distance, shotAngle unit.Angular, cantAngle unit.Angular) ShotParameters {
	return ShotParameters{
		sightAngle:      sightAngle,
		shotAngle:       shotAngle,
		cantAngle:       cantAngle,
		maximumDistance: maxDistance,
		step:            step,
	}
}

//CreateShotParametersEx creates the parameter of the shot with extended parameters including shot angle and cant angle
//
//sightAngle - is the angle between scope centerline and the barrel centerline
//
//shotAngle - is the angle between lines drawn from the shooter to the target and the horizon. The positive angle
//means that the target is higher and the negative angle means that the target is lower
//
//cantAngle - is the cant angle (the angle between centers of scope and the barrel projection and zenith line)
func CreateShotParametersEx(sightAngle unit.Angular, maxDistance unit.Distance, step unit.Distance, shotAngle unit.Angular, cantAngle unit.Angular) ShotParameters {
	return ShotParameters{
		sightAngle:      sightAngle,
		shotAngle:       shotAngle,
		cantAngle:       cantAngle,
		maximumDistance: maxDistance,
		step:            step,
	}
}

//CreateShotParametersCoriolis creates the parameters of the shot including the data required to account
//for the Coriolis/Eötvös effect (Earth rotation).
//
//sightAngle - is the angle between scope centerline and the barrel centerline
//
//shotAngle - is the angle between lines drawn from the shooter to the target and the horizon. The positive angle
//means that the target is higher and the negative angle means that the target is lower
//
//cantAngle - is the cant angle (the angle between centers of scope and the barrel projection and zenith line)
//
//azimuth - is the compass bearing of the line of fire (0° = North, clockwise → 90° = East). It orients the
//Coriolis/Eötvös deflection; it does not tilt the trajectory itself.
//
//latitude - is the geographic latitude of the shot (North positive, South negative). Using this constructor
//always enables the Coriolis/Eötvös correction; use the other constructors to leave it off.
func CreateShotParametersCoriolis(sightAngle unit.Angular, maxDistance unit.Distance, step unit.Distance, shotAngle unit.Angular, cantAngle unit.Angular, azimuth unit.Angular, latitude unit.Angular) ShotParameters {
	return ShotParameters{
		sightAngle:      sightAngle,
		shotAngle:       shotAngle,
		cantAngle:       cantAngle,
		maximumDistance: maxDistance,
		step:            step,
		azimuth:         azimuth,
		latitude:        latitude,
		hasLatitude:     true,
	}
}

//Azimuth returns the compass bearing of the line of fire used to orient the Coriolis/Eötvös deflection
func (v ShotParameters) Azimuth() unit.Angular {
	return v.azimuth
}

//Latitude returns the geographic latitude of the shot used for the Coriolis/Eötvös deflection
func (v ShotParameters) Latitude() unit.Angular {
	return v.latitude
}

//HasLatitude returns true when the latitude is set and the Coriolis/Eötvös effect must be accounted for
func (v ShotParameters) HasLatitude() bool {
	return v.hasLatitude
}
