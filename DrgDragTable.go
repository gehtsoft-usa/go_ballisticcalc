package externalballistics

import (
	"bufio"
	"fmt"
	"io"
	"os"
	"sort"
	"strconv"
	"strings"

	"github.com/gehtsoft-usa/go_ballisticcalc/bmath/unit"
)

//BcAtMach is a single knot (a Mach number and the effective ballistic coefficient at it) of a
//multi-BC profile. It is consumed by BuildDrgDragTable to synthesize a custom (GC) drag curve.
type BcAtMach struct {
	// mach is the Mach number at which the ballistic coefficient is specified
	mach float64
	// bc is the effective ballistic coefficient at that Mach, relative to the base drag curve
	bc float64
}

//CreateBcAtMach creates a knot of a multi-BC profile.
//
//mach - the Mach number (must be non-negative).
//
//bc - the effective ballistic coefficient at that Mach (must be positive).
func CreateBcAtMach(mach float64, bc float64) BcAtMach {
	return BcAtMach{mach: mach, bc: bc}
}

//Mach returns the Mach number of the knot
func (v BcAtMach) Mach() float64 {
	return v.mach
}

//Bc returns the effective ballistic coefficient of the knot
func (v BcAtMach) Bc() float64 {
	return v.bc
}

//DrgDragTable holds a custom (GC) drag table: a set of (Mach, drag coefficient) data points plus
//optional projectile metadata. It can be synthesized from a multi-BC profile (BuildDrgDragTable) or
//loaded from a .drg file (OpenDrgDragTable / ReadDrgDragTable), and turned into a BallisticCoefficient.
type DrgDragTable struct {
	dataPoints  []DataPoint
	curve       []CurvePoint
	name        string
	weight      unit.Weight
	diameter    unit.Distance
	hasMetadata bool
}

//CreateDrgDragTable builds a custom drag table from a set of (Mach, drag coefficient) data points.
//The points are copied and sorted ascending by Mach. At least two points are required.
func CreateDrgDragTable(dataPoints []DataPoint) (DrgDragTable, error) {
	if len(dataPoints) < 2 {
		return DrgDragTable{}, fmt.Errorf("DrgDragTable: at least two data points are required")
	}
	points := make([]DataPoint, len(dataPoints))
	copy(points, dataPoints)
	sort.Slice(points, func(i, j int) bool { return points[i].A < points[j].A })
	return DrgDragTable{
		dataPoints: points,
		curve:      CalculateCurve(points),
	}, nil
}

//BuildDrgDragTable synthesizes a custom (GC) drag table from a standard base table and a multi-BC
//profile that varies with Mach: Cd_custom(M) = Cd_base(M) / BC(M), evaluated on the base table's own
//Mach grid. BC(M) is interpolated piecewise-linearly between the Mach-sorted knots, with flat
//extrapolation outside the knot range.
//
//baseTable - a standard drag table id (G1, G7, ...); it must not be GC.
//
//bcCurve - the multi-BC profile, at least one knot, in any order (sorted internally). Every Bc must be positive.
//
//Run the resulting table through BallisticCoefficient(1.0, BC) so the engine's per-step (1/BC)
//factor is 1.0 and Cd_custom is applied exactly.
func BuildDrgDragTable(baseTable byte, bcCurve []BcAtMach) (DrgDragTable, error) {
	if baseTable == DragTableGC {
		return DrgDragTable{}, fmt.Errorf("DrgDragTable: the base table must be a standard drag curve, not GC")
	}
	if len(bcCurve) < 1 {
		return DrgDragTable{}, fmt.Errorf("DrgDragTable: at least one BC knot is required")
	}

	base, err := standardDragTable(baseTable)
	if err != nil {
		return DrgDragTable{}, err
	}

	knots := make([]BcAtMach, len(bcCurve))
	copy(knots, bcCurve)
	sort.Slice(knots, func(i, j int) bool { return knots[i].mach < knots[j].mach })
	for _, k := range knots {
		if k.bc <= 0 {
			return DrgDragTable{}, fmt.Errorf("DrgDragTable: the ballistic coefficient must be positive")
		}
	}

	points := make([]DataPoint, len(base))
	for i := range base {
		mach := base[i].A
		bc := interpolateBc(knots, mach)
		points[i] = DataPoint{A: mach, B: base[i].B / bc}
	}
	return CreateDrgDragTable(points)
}

//interpolateBc evaluates BC(M) for a Mach-sorted set of knots: piecewise-linear between knots with
//flat extrapolation beyond the end knots.
func interpolateBc(knots []BcAtMach, mach float64) float64 {
	if mach <= knots[0].mach {
		return knots[0].bc
	}
	last := len(knots) - 1
	if mach >= knots[last].mach {
		return knots[last].bc
	}
	for i := 1; i < len(knots); i++ {
		if mach <= knots[i].mach {
			a := knots[i-1]
			b := knots[i]
			t := (mach - a.mach) / (b.mach - a.mach)
			return a.bc + t*(b.bc-a.bc)
		}
	}
	return knots[last].bc
}

//BallisticCoefficient creates a ballistic coefficient that uses this custom (GC) drag table.
//
//Typically called with value 1.0 and value type BC when the table was built from a multi-BC profile,
//so the engine's per-step (1/BC) factor is 1.0 and the table's drag coefficients apply exactly.
func (t DrgDragTable) BallisticCoefficient(value float64, valueType byte) (BallisticCoefficient, error) {
	if value <= 0 {
		return BallisticCoefficient{}, fmt.Errorf("BallisticCoefficient: Drag coefficient must be greater than zero")
	}
	if valueType != BC && valueType != FF {
		return BallisticCoefficient{}, fmt.Errorf("BallisticCoefficient: The value type must be either BC or FF")
	}
	points := t.dataPoints
	curve := t.curve
	drag := func(mach float64) float64 {
		return CalculateByCurve(points, curve, mach)
	}
	return BallisticCoefficient{
		value:     value,
		valueType: valueType,
		table:     DragTableGC,
		drag:      drag,
	}, nil
}

//DataPoints returns the (Mach, drag coefficient) data points of the table (sorted ascending by Mach)
func (t DrgDragTable) DataPoints() []DataPoint {
	return t.dataPoints
}

//HasMetadata returns true if the table carries projectile metadata (name/weight/diameter), e.g. when
//loaded from a .drg file with a valid header.
func (t DrgDragTable) HasMetadata() bool {
	return t.hasMetadata
}

//Name returns the projectile/table name from the metadata (empty when HasMetadata is false)
func (t DrgDragTable) Name() string {
	return t.name
}

//Weight returns the projectile weight from the metadata (zero when HasMetadata is false)
func (t DrgDragTable) Weight() unit.Weight {
	return t.weight
}

//Diameter returns the projectile diameter from the metadata (zero when HasMetadata is false)
func (t DrgDragTable) Diameter() unit.Distance {
	return t.diameter
}

//OpenDrgDragTable loads a custom drag table from a .drg file (CFM/BRL format).
func OpenDrgDragTable(fileName string) (DrgDragTable, error) {
	f, err := os.Open(fileName)
	if err != nil {
		return DrgDragTable{}, err
	}
	defer f.Close()
	return ReadDrgDragTable(f)
}

//ReadDrgDragTable loads a custom drag table from a reader over the .drg (CFM/BRL) format.
//
//The header line is comma-separated with the first field CFM or BRL, followed by name, weight (kg)
//and diameter (m). Each subsequent line holds two whitespace-separated numbers: the drag coefficient
//first and the Mach number second; lines that do not hold exactly two numbers are skipped.
func ReadDrgDragTable(reader io.Reader) (DrgDragTable, error) {
	scanner := bufio.NewScanner(reader)

	var points []DataPoint
	var name string
	var weight unit.Weight
	var diameter unit.Distance
	var hasMeta bool
	var headerParsed bool

	for scanner.Scan() {
		line := strings.TrimSpace(scanner.Text())
		if line == "" {
			continue
		}

		if !headerParsed {
			parts := strings.Split(line, ",")
			if len(parts) < 1 || (strings.TrimSpace(parts[0]) != "CFM" && strings.TrimSpace(parts[0]) != "BRL") {
				return DrgDragTable{}, fmt.Errorf("ReadDrgDragTable: invalid header, expected a line starting with CFM or BRL")
			}
			if len(parts) >= 4 {
				w, errW := strconv.ParseFloat(strings.TrimSpace(parts[2]), 64)
				d, errD := strconv.ParseFloat(strings.TrimSpace(parts[3]), 64)
				if errW == nil && errD == nil {
					name = strings.TrimSpace(parts[1])
					weight = unit.MustCreateWeight(w, unit.WeightKilogram)
					diameter = unit.MustCreateDistance(d, unit.DistanceMeter)
					hasMeta = true
				}
			}
			headerParsed = true
			continue
		}

		fields := strings.Fields(line)
		if len(fields) != 2 {
			continue
		}
		cd, errCd := strconv.ParseFloat(fields[0], 64)
		mach, errMach := strconv.ParseFloat(fields[1], 64)
		if errCd != nil || errMach != nil {
			continue
		}
		points = append(points, DataPoint{A: mach, B: cd})
	}

	if err := scanner.Err(); err != nil {
		return DrgDragTable{}, err
	}
	if !headerParsed {
		return DrgDragTable{}, fmt.Errorf("ReadDrgDragTable: empty or missing header")
	}
	if len(points) < 2 {
		return DrgDragTable{}, fmt.Errorf("ReadDrgDragTable: at least two data points are required, got %d", len(points))
	}

	table, err := CreateDrgDragTable(points)
	if err != nil {
		return DrgDragTable{}, err
	}
	if hasMeta {
		table.name = name
		table.weight = weight
		table.diameter = diameter
		table.hasMetadata = true
	}
	return table, nil
}
