package externalballistics

import (
	"fmt"
	"math"

	"github.com/gehtsoft-usa/go_ballisticcalc/bmath/unit"
)

const cIcaoStandardTemperatureR float64 = 518.67
const cIcaoFreezingPointTemperatureR float64 = 459.67
const cTemperatureGradient float64 = -3.56616e-03
const cIcaoStandardHumidity float64 = 0.0
const cPressureExponent float64 = -5.255876
const cStandardTemperature float64 = 59.0
const cStandardPressure float64 = 29.92

// SI humid-air density model, ported from the .NET engine's Atmosphere.cs. It replaces the older
// imperial JBM approximation so Go and .NET agree on air density — in particular the humidity
// correction, which the old model under-weighted (giving ~0.5% denser air at high relative humidity,
// hence ~0.5% excess drag and a long-range trajectory divergence from the .NET/Ballistic Explorer
// references). Sound speed uses the same c = 331.3*sqrt(T_K/273.15) m/s formula as .NET.
const cStandardDensityKgM3 float64 = 1.2249959 // 0.076474 lb/ft³ reference density, in kg/m³
const cDryAirGasConstant float64 = 287.058     // specific gas constant for dry air, J/(kg·K)
const cVaporGasConstant float64 = 461.495      // specific gas constant for water vapor, J/(kg·K)
const cVaporPressureES0 float64 = 6.1078       // saturated vapor pressure base, hPa
// Herman Wobus polynomial coefficients for saturated vapor pressure (temperature in °C)
const cSvpC0 float64 = 0.99999683
const cSvpC1 float64 = -0.90826951e-2
const cSvpC2 float64 = 0.78736169e-4
const cSvpC3 float64 = -0.61117958e-6
const cSvpC4 float64 = 0.43884187e-8
const cSvpC5 float64 = -0.29883885e-10
const cSvpC6 float64 = 0.21874425e-12
const cTemperatureLapse float64 = -0.0065     // temperature lapse rate, K/m
const cUniversalGasConstant float64 = 8.31432 // universal gas constant, J/(mol·K)
const cGravitySI float64 = 9.80665            // gravitational acceleration, m/s²
const cAirMolarMass float64 = 0.0289644       // molar mass of dry air, kg/mol
const cFootMeter float64 = 0.3048             // feet-to-meters

//Atmosphere describes the atmosphere conditions
type Atmosphere struct {
	altitude    unit.Distance
	pressure    unit.Pressure
	temperature unit.Temperature
	humidity    float64
	density     float64
	mach        unit.Velocity
	mach1       float64
}

//CreateDefaultAtmosphere creates a default atmosphere used in ballistic calculations
func CreateDefaultAtmosphere() Atmosphere {
	a := Atmosphere{altitude: unit.MustCreateDistance(0, unit.DistanceFoot),
		pressure:    unit.MustCreatePressure(cStandardPressure, unit.PressureInHg),
		temperature: unit.MustCreateTemperature(cStandardTemperature, unit.TemperatureFahrenheit),
		humidity:    0.78}
	a.calculate()
	return a
}

//CreateAtmosphere creates the atmosphere with the specified parameter
func CreateAtmosphere(altitude unit.Distance, pressure unit.Pressure, temperature unit.Temperature, humidity float64) (Atmosphere, error) {
	if humidity < 0 || humidity > 100 {
		return CreateDefaultAtmosphere(), fmt.Errorf("Atmosphere : humidity must be in 0..1 or 0..100 range")
	}

	if humidity > 1 {
		humidity = humidity / 100
	}

	a := Atmosphere{altitude: altitude,
		pressure:    pressure,
		temperature: temperature,
		humidity:    humidity}

	a.calculate()
	return a, nil
}

//CreateICAOAtmosphere creates default ICAO atmosphere for the specified altitude
func CreateICAOAtmosphere(altitude unit.Distance) Atmosphere {
	temperature := unit.MustCreateTemperature(
		cIcaoStandardTemperatureR+
			altitude.In(unit.DistanceFoot)*cTemperatureGradient-cIcaoFreezingPointTemperatureR,
		unit.TemperatureFahrenheit)

	pressure := unit.MustCreatePressure(
		cStandardPressure*
			math.Pow(cIcaoStandardTemperatureR/(temperature.In(unit.TemperatureFahrenheit)+
				cIcaoFreezingPointTemperatureR),
				cPressureExponent), unit.PressureInHg)

	a := Atmosphere{
		altitude:    altitude,
		temperature: temperature,
		pressure:    pressure,
		humidity:    cIcaoStandardHumidity,
	}

	a.calculate()
	return a
}

//Altitude returns the ground level altitude over the sea level
func (a Atmosphere) Altitude() unit.Distance {
	return a.altitude
}

//Temperature returns the temperature at the ground level
func (a Atmosphere) Temperature() unit.Temperature {
	return a.temperature
}

//Pressure returns the pressure at the ground level
func (a Atmosphere) Pressure() unit.Pressure {
	return a.pressure
}

//Humidity returns the relative humidity set in 0 to 1 coefficient
//
//multiply this value by 100 to get percents
func (a Atmosphere) Humidity() float64 {
	return a.humidity
}

//HumidityInPercents returns relative humidity in percents (0..100)
func (a Atmosphere) HumidityInPercents() float64 {
	return a.humidity * 100
}

func (a Atmosphere) String() string {
	return fmt.Sprintf("Altitude:%s,Pressure:%s,Temperature:%s,Humidity:%.2f%%",
		a.altitude, a.pressure, a.temperature, a.humidity*100)
}

func (a Atmosphere) getDensityFactor() float64 {
	return a.density / cStandardDensityKgM3
}

//Mach returns the speed of sound at the atmosphere with such parameters
func (a Atmosphere) Mach() unit.Velocity {
	return a.mach
}

// saturatedVapourPressure returns the 100%-saturated vapor pressure (hPa) for a temperature in °C,
// using the Herman Wobus polynomial.
func saturatedVapourPressure(tCelsius float64) float64 {
	pt := cSvpC0 + tCelsius*(cSvpC1+tCelsius*(cSvpC2+tCelsius*(cSvpC3+tCelsius*(cSvpC4+tCelsius*(cSvpC5+tCelsius*cSvpC6)))))
	return cVaporPressureES0 / math.Pow(pt, 8)
}

// calculateDensitySI returns the humid-air density (kg/m³) for a temperature in Kelvin, a pressure in
// pascals and a relative humidity (0..1), summing the dry-air and water-vapor partial contributions.
func calculateDensitySI(temperatureKelvin, pressurePascal, humidity float64) float64 {
	tCelsius := temperatureKelvin - 273.15
	vaporSaturation := saturatedVapourPressure(tCelsius) * 100 // hPa -> Pa
	actualVapourPressure := vaporSaturation * humidity
	dryPressure := pressurePascal - actualVapourPressure
	return dryPressure/(cDryAirGasConstant*temperatureKelvin) + actualVapourPressure/(cVaporGasConstant*temperatureKelvin)
}

// soundVelocityFps returns the speed of sound (ft/s) for a temperature in Kelvin.
func soundVelocityFps(temperatureKelvin float64) float64 {
	return (331.3 * math.Sqrt(temperatureKelvin/273.15)) / cFootMeter
}

// temperatureAtAltitudeKelvin applies the standard temperature lapse rate.
func temperatureAtAltitudeKelvin(baseKelvin, baseAltitudeMeter, altitudeMeter float64) float64 {
	return baseKelvin + cTemperatureLapse*(altitudeMeter-baseAltitudeMeter)
}

// pressureAtAltitudePascal applies the barometric formula (constant base temperature).
func pressureAtAltitudePascal(basePressurePascal, baseKelvin, baseAltitudeMeter, altitudeMeter float64) float64 {
	exponent := -cGravitySI * cAirMolarMass / (cUniversalGasConstant * cTemperatureLapse)
	return basePressurePascal * math.Pow(1+cTemperatureLapse/baseKelvin*(altitudeMeter-baseAltitudeMeter), exponent)
}

func (a *Atmosphere) calculate() {
	tempKelvin := a.temperature.In(unit.TemperatureKelvin)
	pressPascal := a.pressure.In(unit.PressureHP) * 100 // hectopascals -> pascals

	a.density = calculateDensitySI(tempKelvin, pressPascal, a.humidity)
	a.mach1 = soundVelocityFps(tempKelvin)
	a.mach = unit.MustCreateVelocity(a.mach1, unit.VelocityFPS)
}

func (a *Atmosphere) getDensityFactorAndMachForAltitude(altitude float64) (float64, float64) {
	orgAltitude := a.altitude.In(unit.DistanceFoot)

	if math.Abs(orgAltitude-altitude) < 30 {
		return a.density / cStandardDensityKgM3, a.mach1
	}

	baseKelvin := a.temperature.In(unit.TemperatureKelvin)
	basePressPascal := a.pressure.In(unit.PressureHP) * 100
	baseAltitudeMeter := orgAltitude * cFootMeter
	altitudeMeter := altitude * cFootMeter

	tempKelvin := temperatureAtAltitudeKelvin(baseKelvin, baseAltitudeMeter, altitudeMeter)
	pressPascal := pressureAtAltitudePascal(basePressPascal, baseKelvin, baseAltitudeMeter, altitudeMeter)
	density := calculateDensitySI(tempKelvin, pressPascal, a.humidity)
	return density / cStandardDensityKgM3, soundVelocityFps(tempKelvin)
}
