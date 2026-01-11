package org.firstinspires.ftc.teamcode.util;

import java.util.Map;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

// TODO: Convert all instances of this class into LinearInterpolator
@Deprecated
public class RampAngleInterpolator extends LinearInterpolator implements AngleGetter {
    /**
     * The units the dists array was recorded in.
     */
    private final DistanceUnit distsUnit;

    private final DistanceGetter getter;

    public RampAngleInterpolator(final DistanceUnit units, Map<Double, Double> distAnglePairs, DistanceGetter distGetter) {
        super(distAnglePairs);
        this.distsUnit = units;
        this.getter = distGetter;
    }

    /**
     * Determines what angle the ramp would be at given the provided
     * distance.
     * 
     * @param dist The distance to use for calculation
     * @param units The Units of the provided distance. 
     * @return The calculated ramp angle, in radians.
     */
    public double calculateAngle(double dist, DistanceUnit units) {
        return calculate(units.fromUnit(this.distsUnit, dist));
    }

    /**
     * Alias for `LinearInterpolator.calculate()`
     * 
     * @param dist The measured distance, in the units provided at construction
     * @return The angle that can be interpolated from the data and the argument
     * @see LinearInterpolator.calculate()
     */
    public double calculateAngle(double dist) {
        return calculate(dist);
    }

    /**
     * Alias for `LinearInterpolator.getAsDouble().` This reevaluates the
     * expression of the getter provided at construction time. 
     * 
     * @return The angle, in the units, given for the current distance 
     */
    @Override
    public double getAngle(AngleUnit units) {
        return units.fromRadians(this.getAngle());
    }

    /**
     * @return The angle, in radians, for the current distance 
     */
    @Override
    public double getAngle() {
        return calculateAngle(getter.getDistance(distsUnit));
    }
}