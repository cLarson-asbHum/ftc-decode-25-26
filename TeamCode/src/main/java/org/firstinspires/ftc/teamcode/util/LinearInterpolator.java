package org.firstinspires.ftc.teamcode.util;

import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;
import java.util.Set;
import java.util.function.DoubleUnaryOperator;

import org.firstinspires.ftc.teamcode.util.Util;

public class LinearInterpolator implements DoubleUnaryOperator {
    /**
     * The input the corresponding output was at.
     */
    private final double[] ins;

    /**
     * The measured output for some given input (which is at the same index in `ins`).
     */
    private final double[] outs;

    public LinearInterpolator(Map<Double, Double> coordinatePairs) {
        // Initializing ins and sorting
        this.ins = new double[coordinatePairs.size()];
        final Double[] wrappedInputs = coordinatePairs.keySet().toArray(new Double[ins.length]);

        for(int i = 0; i < ins.length; i++) {
            ins[i] = wrappedInputs[i].doubleValue();
        }

        Arrays.sort(this.ins);

        // Getting the outs
        this.outs = new double[coordinatePairs.size()];

        for(int i = 0; i < outs.length; i++) {
            outs[i] = coordinatePairs.get(ins[i]).doubleValue(); 
        }
    }

    /**
     * Determines what angle the ramp would be at given the provided 
     * input. If the argument is out of bounds (`input < 0` or 
     * `input > max(coordinatePairs.keySet())`), then this throws an IllegalArgumentException
     * 
     * @param input The input to use for calculation. 
     * @return A value linearly interpolated between data points provided at construction
     */
    public double calculate(double input) {
        final int i = -1 - Arrays.binarySearch(ins, input);

        // If the input was in the list, return the corresponding angle
        if(i < 0) {
            return outs[-1 - i];
        }

        // Checking that the input is inside the ranges
        if(i <= 0 || i >= ins.length) {
            throw new IllegalArgumentException("Input " + input + " was out of bounds for " + this.toString());
        }

        // Interpolating the input
        final double interpFactor = Util.invLerp(ins[i - 1], input, ins[i]);
        return Util.lerp(outs[i - 1], interpFactor, outs[i]);
    }

    /**
     * Determines what angle the ramp would be at given the provided 
     * input. If the argument is out of bounds (`input < 0` or 
     * `input > max(coordinatePairs.keySet())`), then the output is equal to the 
     * closest input. In other words, if input is less than 0, `calculate(0)` is 
     * returned. If input is greater than the maximum input, then 
     * `calculate(maxInput)` is returned.
     * 
     * @param input The input to use for calculation, clamped to be within bounds
     * @return The calculated output. 
     */
    public double clampedCalculate(double input) {
        return calculate(Util.clamp(0, input, ins[ins.length - 1]));
    }

    @Override
    public double applyAsDouble(double input) {
        return calculate(input);
    }

    /**
     * Creates the inverse function of this interpolator. The returned interpolator 
     * is a new interpolator and the inputs and outputs are cloned by this method.
     * 
     * The calling interpolator is not modified as a result of this method
     * 
     * @return An interpolator with this interpolator's inputs and outputs swapped
     */
    public LinearInterpolator inverse() {
        final HashMap<Double, Double> pairs = new HashMap<>();

        for(int i = 0; i < ins.length; i++) {
            pairs.put(outs[i], ins[i]);
        }

        return new LinearInterpolator(pairs);
    }

}