package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public interface AngleGetter {
    /**
     * Gets the current angle. No guarantees are made that the value will be 
     * unique or up-to-date.
     */
    public double getAngle(AngleUnit units);

    /**
     * Gets the angle in radians. By default, this uses the one-parameter 
     * overload of `getAngle()` using `AngleUnit.RADIANS`.
     * 
     * @return The current angle, in radians
     */
    public default double getAngle() {
        return getAngle(AngleUnit.RADIANS);
    }
}