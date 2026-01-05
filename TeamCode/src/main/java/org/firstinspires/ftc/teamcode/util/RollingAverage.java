package org.firstinspires.ftc.teamcode.util;

import java.util.LinkedList;
import java.util.function.DoubleSupplier;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class RollingAverage implements DoubleSupplier {
    private final double[] weights;
    private final DoubleSupplier valGetter;
    private final LinkedList<Double> vals = new LinkedList<>();
    private double val = 0;

    public RollingAverage(DoubleSupplier valGetter, double[] rollingWeights) {
        this.valGetter = valGetter;
        this.weights = rollingWeights;
    }

    /**
     * Updates all sensor readings. 
     */
    public void clearBulkCache() {
        addNew(valGetter.getAsDouble());
        val = 0;

        double totalWeights = 0;
        int i = 0;
        for(final double measuredVal : vals) {
            val += weights[i] * measuredVal;
            totalWeights += weights[i];
            i++;
        }

        if(totalWeights != 0) {
            val /= totalWeights;
        } 
    }

    private void addNew(double val) {
        vals.addFirst(val); // The first index is the most recent datum
        
        if(vals.size() > weights.length) {
            vals.removeLast(); // The last index is the oldest datum
        }
    }

    @Override
    public double getAsDouble() {
        // TODO: Bulk caching
        clearBulkCache();
        return val;
    }
}