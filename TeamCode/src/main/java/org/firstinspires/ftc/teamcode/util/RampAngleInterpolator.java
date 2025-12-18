package org.firstinspires.ftc.teamcode.util;

import java.util.Arrays;
import java.util.Map;
import java.util.Set;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.Util;

public class RampAngleInterpolator implements AngleGetter {
    private final DistanceGetter distGetter;
    
    /**
     * The distance the corresponding angle was at.
     */
    private final double[] dists;

    /**
     * The units the dists array was recorded in.
     */
    private final DistanceUnit distsUnit;

    /**
     * The measured angles in radians at the given distance.
     */
    private final double[] angles;

    public RampAngleInterpolator(DistanceUnit units, Map<Double, Double> distAnglePairs, DistanceGetter distGetter) {
        this.distsUnit = units; 
        this.distGetter = distGetter;

        // Initializing dists and sorting
        this.dists = new double[distAnglePairs.size()];
        final Double[] wrappedDists = distAnglePairs.keySet().toArray(new Double[dists.length]);

        for(int i = 0; i < dists.length; i++) {
            dists[i] = wrappedDists[i].doubleValue(); // Auto unboxing
        }

        Arrays.sort(this.dists);

        // Getting the angles
        this.angles = new double[distAnglePairs.size()];

        for(int i = 0; i < angles.length; i++) {
            angles[i] = distAnglePairs.get(dists[i]).doubleValue(); // Auto unboxing
        }
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
        return calculateAngle(units.fromUnit(this.distsUnit, dist));
    }

    /**
     * Determines what angle the ramp would be at given the provided 
     * distance. The units are assumed to be those provided at construction
     * 
     * @param dist The distance to use for calculation
     * @param units The Units of the provided distance. 
     * @return The calculated ramp angle, in radians.
     */
    public double calculateAngle(double dist) {
        final int i = -1 - Arrays.binarySearch(dists, dist);

        // If the distance was in the list, return the corresponding angle
        if(i < 0) {
            return angles[-1 - i];
        }

        // Checking that the distance is inside the ranges
        if(i <= 0 || i >= dists.length) {
            throw new IllegalArgumentException("Distance " + dist + " was out of bounds for " + this.toString());
        }

        // Interpolating the distance
        final double interpFactor = Util.invLerp(dists[i - 1], dist, dists[i]);
        return Util.lerp(angles[i - 1], interpFactor, angles[i]);
    }

    @Override
    public double getAngle(AngleUnit units) {
        return units.fromRadians(this.getAngle());
    }

    @Override
    public double getAngle() {
        return calculateAngle(distGetter.getDistance(distsUnit));
    }
}