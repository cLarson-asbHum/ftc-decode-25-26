package org.firstinspires.ftc.teamcode.hardware.subsystem;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.Subsystem;

public interface MecanumSubsystem extends Subsystem {
    public double getLateralMultiplier();

    public boolean setLateralMultiplier(double newMultiplier);

    public double getForwardMultiplier();

    public boolean setForwardMultiplier(double newMultiplier);

    public double getYawMultiplier();

    public boolean setYawMultiplier(double newMultiplier);

    /**
     * Gives the given power to each motor.
     * 
     * @param powers The new powers the motors will be set to.
     * @return Whether *any* of the motor powers were changed. 
     */
    public boolean powerMotors(double fl, double bl, double fr, double br);

    default public boolean stop() {
        return powerMotors(0, 0, 0, 0);
    }

    /**
     * Drives the drivetrain using a right-angle mecanum setup. 
     * 
     * @param forward The factor moving forward, from 
     * @return Whether the motor powers were actually changed.
     */
    default public boolean mecanumDrive(double forward, double lateral, double yaw) {
        final double scaledForward = forward * getForwardMultiplier();
        final double scaledLateral = lateral * getLateralMultiplier();
        final double scaledYaw = yaw * getYawMultiplier();

        return powerMotors(
            /* frontLeft  */ scaledForward + scaledLateral + scaledYaw,
            /* backLeft */ scaledForward - scaledLateral + scaledYaw,
            /* frontRight   */ scaledForward - scaledLateral - scaledYaw,
            /* backRight  */ scaledForward + scaledLateral - scaledYaw
        );

    }
}