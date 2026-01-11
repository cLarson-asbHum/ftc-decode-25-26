package org.firstinspires.ftc.teamcode.hardware.subsystem;

import com.arcrobotics.ftclib.command.Command;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.hardware.subsystem.StateSubsystem;
import org.firstinspires.ftc.teamcode.util.WrapConcurrentCommand;

/**
 * Represents a Subsystem capable of rotating to a position and holding that 
 * position. Whether a position is held after reaching it is up to the 
 * implementation, although it is recommended to always hold.
 */
public interface PivotSubsystem extends StateSubsystem<PivotSubsystem.Status> {
    public static enum Status {
        /**
         * The current state cannot be determined. This is the intended state before 
         * any angle has been set
         */
        UNKNOWN,

        /**
         * The target angle has not been reached. The subsystem is actively 
         * traveling to it.
         */
        POSITIONING,

        /**
         * The target angle has been reached and is currently being maintained.
         */
        HOLDING;
    }

    /**
     * Alias for `getStatus()`.
     */
    public default Status getState() {
        return this.getStatus();
    }

    public default Status getStatus() {
        // The subsystem is within the target angle
        if(isWithinTarget(getTargetTolerance()) && !isRunningToAngle()) {
            return Status.HOLDING;
        } 

        // The subsystem is not within the target angle
        if(!isWithinTarget(getTargetTolerance()) && isRunningToAngle()) {}

        // The state could not be determined
        return Status.UNKNOWN;
    }

    /**
     * Gets the current angle of the pivot in radians.
     * 
     * Note: The returned angle can be unnormalized.
     * 
     * @return The angle of the actuator(s). in radians.
     */
    public double getCurrentAngle();
    
    /**
     * Gets the current angle of the pivot in the given units.
     * 
     * Note: The returned angle can be unnormalized.
     * 
     * @return The angle of the actuator(s) in the given units.
     */
    public default double getCurrentAngle(AngleUnit units) {
        return units.fromRadians(getCurrentAngle());
    }

    /**
     * Gets the current normalized angle in radians.
     * 
     * Note: The current angle may already be normalized
     * 
     * @return The current angle in radians, on interval [-pi, pi)
     */
    public default double getCurrentAngleNorm() {
        return AngleUnit.normalizeRadians(getCurrentAngle());
    }
    
    /**
     * Gets the current normalized angle in the given unit.
     * 
     * Note: The current angle may already be normalized
     * 
     * @return The current angle in the given unit, normalized.
     */
    public default double getCurrentAngleNorm(AngleUnit units) {
        return units.normalize(getCurrentAngle(units));
    }

    /**
     * Gets the angle the pivot is trying to run to/hold. If the motor is not
     * running to a position nor holding a position, then the last set target 
     * angle is used. If no position has been set, the angle is set at 
     * initialization.
     * 
     * Note: The returned angle can be unnormalized.
     * 
     * @return The last target position set by `runToAngle()`, in radians 
     */
    public double getTargetAngle();
    
    /**
     * Gets the last tolerance set by runToAngle()
     * 
     * @return The last tolerance set by runToAngle(), in radians.
     */
    public double getTargetTolerance();

    /**
     * Determines whether the angle of the pivot is within `tolerance` 
     * radians of the given target angle. If the angle is exactly 
     * `tolerance` radians away from target (i.e. 
     * Math.abs(currentTheta - target) = tolerance) then this will return false.
     * 
     * @param target - The desired angle to be close to.
     * @param tolerance - Maxmimum (exclusive) difference between the 
     * target and the current angle 
     * @return Whether the absolute difference between the current angle and the 
     * target is less than `tolerance`.
     */
    public default boolean isWithin(double target, double tolerance) {
        return Math.abs(this.getCurrentAngle() - target) < tolerance;
    }

    /**
     * Determines whether the angle of the pivot is within `tolerance` 
     * radians of the target angle set by runToAngle. If the angle is exactly 
     * `tolerance` radians away from target (i.e. 
     * Math.abs(currentTheta - target) = tolerance) then this will return false.
     * 
     * @param tolerance Maxmimum (exclusive) difference between the 
     * target and the current angle 
     * @return Whether the absolute difference between the current angle and the 
     * target is less than `tolerance`.
     */
    public default boolean isWithinTarget(double tolerance) {
        return isWithin(getTargetAngle(), tolerance);
    }

    /**
     * Returns whether runToAngle() has finished. In other words,
     * this checks if the difference between the current angle and 
     * the last target angle is within the tolerance last set by 
     * `runToAngle()`.
     * 
     * @return True if the current angle is within tolerance of the last target.
     */
    public default boolean isRunningToAngle() {
        return isWithinTarget(getTargetTolerance());
    }

    /**
     * Runs the pivot to the given angle, in radians. This only needs to be 
     * called once to go to the angle; any additional control to reach the
     * target is handled by `periodic()`.
     * 
     * @return Whether the target was succesfully set. False if identical to
     * the given angle.
     */
    public boolean runToAngle(double theta, double tolerance);

    /**
     * Creates a new command wrapping the `runToAngle()` method. The command
     * will call `runToAngle()` at the start until the subsystem .
     * 
     * @param theta Target angle, in radians
     * @param tolerance Acceptable tolerance above or below the target, in 
     * radians 
     * @return A Command that does all that is described (see above).
     */
    public default Command runToAngleCommand(double theta, double tolerance) {
        return new WrapConcurrentCommand<Status>(this, () -> runToAngle(theta, tolerance), Status.POSITIONING);
    }
}