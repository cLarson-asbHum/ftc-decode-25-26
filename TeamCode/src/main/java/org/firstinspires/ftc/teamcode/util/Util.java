package org.firstinspires.ftc.teamcode.util;

import java.util.Iterator;
import java.util.function.Function;

public final class Util {
    public static boolean near(double x, double target, double tolerance) {
        return Math.abs(x - target) < tolerance;
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
}