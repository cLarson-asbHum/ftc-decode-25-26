package org.firstinspires.ftc.teamcode.util;

/**
 * Uses a sensor or more to determine the most accurate artifact color. This can
 * also state a color as unknown, stating it is no single artifact color.
 */
public interface ArtifactColorGetter {
    /**
     * Gets the most accurate color for the current sensor readings
     */
    public ArtifactColor getColor();
}