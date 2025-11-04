package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class CarwashIntake implements IntakeSubsystem {
    public static class Powers {
        public double intake = 1.0;
        public double eject = -1.0;
        public double hold = 0.0;
        public double tolerance = 0.05;
    }

    public static Powers POWERS = new Powers();

    private final Position position = Position.UNKNOWN;
    private DcMotorEx motor;
    private double targetPower = 0;
    private boolean hasSetPower = false;

    public CarwashIntake(DcMotorEx motor) {
        this.motor = motor;
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setPower(0);
        hasSetPower = true;
    }

    /**
     * Attempts to set the targetPower. Fails if the previous targetPower is 
     * within the given tolerance.
     * 
     * NOTE: This does **not** change the run speed of the *hardware*. For that to happen,
     * use the `periodic()` method.
     * 
     * @param newPower What to change the targetPower to. 
     * @param tolerance Maximum (exclusive) difference that doesn't change the power
     * @return Whether the power was actually changed
     */
    private boolean setPower(double newPower, double tolerance) {
        if(Math.abs(newPower - targetPower) < tolerance) {
            return false;
        }

        hasSetPower = false;
        targetPower = newPower;
        return true;
    }

    /**
     * Always returns `UNKNOWN`.
     */
    @Override
    public Position getPosition() {
        return this.position;
    }

    @Override
    public boolean intakeGamePieces() {
        return setPower(POWERS.intake, POWERS.tolerance);
    }

    @Override
    public boolean holdGamePieces() {
        return setPower(POWERS.hold, POWERS.tolerance);
    }

    @Override
    public boolean ejectGamePieces() {
        return setPower(POWERS.eject, POWERS.tolerance);
    }


    @Override
    public void periodic() {
        if(!hasSetPower) {
            motor.setPower(targetPower);
        }

        
        // TODO: Implement time-based power check.
    }

    @Override
    public boolean pivotToIntake() {
        return false;
    }

    @Override
    public boolean pivotToHold() {
        return false;
    }

    @Override
    public boolean pivotToEject() {
        return false;
    }
}