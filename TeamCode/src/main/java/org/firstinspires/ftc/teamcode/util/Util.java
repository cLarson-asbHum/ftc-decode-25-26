package org.firstinspires.ftc.teamcode.util;

import java.util.Iterator;
import java.util.function.Function;

public final class Util {
    public static final double TAU = 2 * Math.PI;
    public static final double SQRT1_2 = Math.sqrt(2) / 2;

    public static boolean near(double x, double target, double tolerance) {
        return Math.abs(x - target) <= tolerance;
    }

    /**
     * Computes the floor modulo of the arguments. This differs from the 
     * remainder operator (%) in that the sign of the result is equal to the
     * sign of the divisor, not the dividend. Therefore, results are in range 
     * [0, divisor] for divisor > 0, or (divisor, 0] for divisor < 0 
     * 
     * @param dividend Numerator of the division
     * @param divisor Denominator of the division. 
     * @return The remainder of dividend / divisor, where the result's sign is 
     * the same as the divisor's sign.
     */
    public static double mod(double dividend, double divisor) {
        return dividend - divisor * Math.floor(dividend / divisor);
    }

    /**
     * Returns a coterminal angle in the range [-pi, +pi)
     * 
     * @param theta The angle to normalize, in radians
     * @return Coterminal angle guarnateed to be in range [+pi, pi).
     */
    public static double normalizeRadians(double theta) {
        return Util.mod(theta + Math.PI, TAU) - Math.PI;
    }

    /**
     * Determines whether the angles are within tolerance of each other. This 
     * normalizes the given angles and avoids pitfalls such as the +pi to -pi 
     * jump.
     * 
     * Tolerances greater than pi or more will always return true, no matter the 
     * theta or the target. Negative tolerances will always return false. 
     * 
     * @param theta The current angle in radians. Can be unnormalized
     * @param target The angle to be near in radians. Can be unnormalized
     * @param tolerance Maximum difference of radians to be "near" the target. 
     * Is inclusive
     * @return Whether the (normalized) theta is within tolerance of the 
     * (normalized) target.
     */
    public static boolean anglesNear(double theta, double target, double tolerance) {
        return Math.abs(normalizeRadians(theta - target)) <= tolerance;
    }

    public static double lerp(double a, double t, double b) {
        return t * (b - a) + a;
    }

    public static double invLerp(double a, double l, double b) {
        return (l - a) / (b - a);
    }

    public static double clamp(double min, double x, double max) {
        return Math.max(min, Math.min(x, max));
    }

    public static String header(String title, int length, String filler) {
        final int leftRepeat = (length - title.length()) / (2 * filler.length());
        final int rightRepeat = (length - title.length()) / filler.length() - leftRepeat;
        return filler.repeat(leftRepeat) + title + filler.repeat(rightRepeat); 
    }

    public static final int DEFAULT_HEADER_LENGTH = 60;

    public static String header(String title, int length) {
        return header(title, length, "-");
    }

    public static String header(String title) {
        return header(title, DEFAULT_HEADER_LENGTH);
    }

    public static String padHeader(String title, int length, String filler) {
        return header(" " + title + " ", length, filler);
    }

    public static String padHeader(String title, int length) {
        return header(" " + title + " ", length);
    } 
    
    public static String padHeader(String title) {
        return header(" " + title + " ");
    } 

    public static <T> boolean any(Iterable<T> iter, Function<T, Boolean> predicate) {
        return any(iter.iterator(), predicate);
    }

    public static <T> boolean any(Iterator<T> iter, Function<T, Boolean> predicate) {
        while(iter.hasNext()) {
            if(predicate.apply(iter.next())) {
                return true;
            }
        }

        return false;
    }

    public static <T> boolean all(Iterable<T> iter, Function<T, Boolean> predicate) {
        return all(iter.iterator(), predicate);
    }

    public static <T> boolean all(Iterator<T> iter, Function<T, Boolean> predicate) {
        while(iter.hasNext()) {
            if(!predicate.apply(iter.next())) {
                return false;
            }
        }

        return true;
    }

    public static double avg(double[] doubles) {
        double total = 0.0;

        for(final double dub : doubles) {
            total += dub;
        }

        return total / doubles.length;
    }

    public static int floorAvg(int[] ints) {
        int total = 0;

        for(final int i : ints) {
            total += i;
        }

        return total / ints.length;
    }

    public static String lines(String... lines) {
        String result = "";

        for(final String line : lines) {
            result += line + "\n";
        }

        return result;
    }
}