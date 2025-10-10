package org.firstinspires.ftc.teamcode.subsystem;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.PwmControl.PwmRange;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static org.firstinspires.ftc.teamcode.util.Util.padHeader;

@Configurable
public class FlywheelTubeShooter implements ShooterSubsystem {
    public static final class FlywheelConst {
        // TODO: Tune these const!
        public double ticksPerRev = 576.6;
        public double rpm = 312.0;

        public double ticksPerSec() {
            return rpm / 60  * ticksPerRev;
        }

        public double unchargedPower = 0;
        public double chargedPower = 1.0;
        public double reloadingPower = chargedPower;
        public double firingPower = chargedPower;
        public double abortingPower = 0.3;

        public double powerTolerance = 0.05;
    };

    public static FlywheelConst FLYWHEEL_CONST = new FlywheelConst();

    public static final class FeederConst {
        // TODO: Tune these const!
        public double ticksPerRev = 576.6;
        public double maxRpm = 312.0;

        public double ticksPerSec() {
            return maxRpm / 60  * ticksPerRev;
        }

        public double unchargedPower = 0;
        public double chargedPower = 0;
        public double reloadingPower = 1.0;
        public double firingPower = 0;
        public double abortingPower = -1.0;

        public double powerTolerance = 0.05;
    };

    public static FeederConst FEEDER_CONST = new FeederConst();

    public static final class Timeout {
        public double uncharging = 5.0; // seconds
        public double charging = 5.0; // seconds
        public double reloading = 2.0; // seconds
        public double firing = 1.0; // seconds
        public double multiFiring = Double.POSITIVE_INFINITY; // seconds, but still indefinite
        public double aborting = 1.0; // seconds
    }

    public Timeout TIMEOUT = new Timeout();

    private final DcMotorEx flywheels;
    private double targetFlyWheelPower = 0; 
    private boolean hasSetFlywheelPower = false;
    
    private final CRServo feeder;
    private double targetFeederPower = 0; 
    private boolean hasSetFeederPower = false;

    private Status status = Status.UNKNOWN;

    private Telemetry telemetry = null;
    public static boolean SUPPRESS_TELEMTERY = false;

    /**
     * @param flywheels What propels the projectile at high speeds
     * @param feeder What moves balls into the flywheels. Can be null
     */
    public FlywheelTubeShooter(DcMotorEx flywheels, CRServo feeder) {
        this.flywheels = flywheels;
        this.feeder = feeder;

        // The status is automatically set to UNKNOWN, which transitions to UNCHRAGED.
        // this does our initialization for us.
    }
    
    /**
     * @param flywheels What propells the projectile at high speeds
     */
    public FlywheelTubeShooter(DcMotorEx flywheels) {
        this(flywheels, null);
    }

    private boolean setFlywheelPower(double power, double powerTolerance) {
        if(flywheels == null) {
            return false;
        }

        if(Math.abs(power - targetFlyWheelPower) < FLYWHEEL_CONST.powerTolerance) {
            return false;
        }

        targetFlyWheelPower = power;
        hasSetFlywheelPower = false;
        return true;
    }

    private boolean setFeederPower(double power, double powerTolerance) {
        if(feeder == null) {
            return false;
        }

        if(Math.abs(power - targetFeederPower) < FLYWHEEL_CONST.powerTolerance) {
            return false;
        }

        targetFeederPower = power;
        hasSetFeederPower = false;
        return true;
    }

    private boolean transitionTo(Status newStatus) {
        status = newStatus; 
        return true;
    }

    @Override
    public Status getStatus() {
        return this.status;
    }

    @Override
    public boolean charge() {
        setFeederPower(FEEDER_CONST.chargedPower, FEEDER_CONST.powerTolerance);
        setFlywheelPower(FLYWHEEL_CONST.chargedPower, FLYWHEEL_CONST.powerTolerance);
        startTimeout(Status.UNCHARGING, TIMEOUT.uncharging);
        return transitionTo(Status.CHARGING);
    }

    @Override
    public boolean uncharge() {
        setFeederPower(FEEDER_CONST.unchargedPower, FEEDER_CONST.powerTolerance);
        setFlywheelPower(FLYWHEEL_CONST.unchargedPower, FLYWHEEL_CONST.powerTolerance);
        startTimeout(Status.CHARGING, TIMEOUT.charging);
        return transitionTo(Status.UNCHARGING);
    }

    @Override
    public boolean reload() {
        setFeederPower(FEEDER_CONST.reloadingPower, FEEDER_CONST.powerTolerance);
        setFlywheelPower(FLYWHEEL_CONST.reloadingPower, FLYWHEEL_CONST.powerTolerance);
        startTimeout(Status.RELOADING, TIMEOUT.reloading);
        return transitionTo(Status.RELOADING);
    }

    @Override
    public boolean fire() {
        setFeederPower(FEEDER_CONST.firingPower, FEEDER_CONST.powerTolerance);
        setFlywheelPower(FLYWHEEL_CONST.firingPower, FLYWHEEL_CONST.powerTolerance);
        startTimeout(Status.FIRING, TIMEOUT.firing);
        return transitionTo(Status.FIRING);
    }

    /**
     * Attempts to shoot as many projectiles as possible by using an indeifinite 
     * timeout length and continually reloading. This means that it only ends multi
     * fire whe another state is externally transitioned (e.g. the external opmode 
     * calls `charge()`)
     * 
     * Whether the state was successfully transitioned to.
     */
    public boolean multiFire() {
        setFeederPower(FEEDER_CONST.firingPower, FEEDER_CONST.powerTolerance);
        setFlywheelPower(FLYWHEEL_CONST.firingPower, FLYWHEEL_CONST.powerTolerance);
        startTimeout(Status.FIRING, TIMEOUT.multiFiring);
        return transitionTo(Status.FIRING);
    }

    @Override
    public boolean abort() {
        setFeederPower(FEEDER_CONST.abortingPower, FEEDER_CONST.powerTolerance);
        setFlywheelPower(FLYWHEEL_CONST.abortingPower, FLYWHEEL_CONST.powerTolerance);
        startTimeout(Status.ABORTING, TIMEOUT.aborting);
        return transitionTo(Status.ABORTING);
    }

    public boolean checkConsideredUncharged() {
        final double currentPower = flywheels.getVelocity() / FLYWHEEL_CONST.ticksPerSec();
        return Math.abs(currentPower - FLYWHEEL_CONST.unchargedPower) < FLYWHEEL_CONST.powerTolerance;
    }

    public boolean checkConsideredCharged() {
        final double currentPower = flywheels.getVelocity() / FLYWHEEL_CONST.ticksPerSec();
        return Math.abs(currentPower - FLYWHEEL_CONST.chargedPower) < FLYWHEEL_CONST.powerTolerance;
    }

    public boolean checkIsReloaded() {
        // FIXME: We always assume the shooter is charged (even though it isn't!)
        return true;
    }

    private void startTimeout(Object label, double durationSec) {
        // FIXME: Does nothing! We want this to create a new timeout!
    }

    public boolean isTimedOut(Object label) {
        // FIXME: This always is false. We need something that checks for timeout!
        return false;
    }

    protected void periodicUnkown(Telemetry telemetry) {
        // Just go to uncharged and call it a day.
        // if(feeder != null) {
        //     feeder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // }
        flywheels.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        uncharge();
    }

    protected void periodicUncharging(Telemetry telemetry) {
        // TODO: Transition to previous state if timed out?
        // Transitioning to UNCHARGED if we are within power
        if(checkConsideredUncharged()) {
            transitionTo(Status.UNCHARGED);
        } else if(isTimedOut(Status.UNCHARGING)) {
            // FIXME: This can still maintain charging power.
            // FIXME: This can get stuck in an infinite loop!
            charge();
        }
    }

    protected void periodicCharging(Telemetry telemetry) {
        // TODO: Transition to previous state if timed out?
        // Transitioning to EMPTY_CHARGED or RELOADED_CHARGED if we are within power
        if(checkConsideredCharged()) {
            if(checkIsReloaded()) {
                transitionTo(Status.RELOADED_CHARGED);
            } else {
                transitionTo(Status.EMPTY_CHARGED);
            }
        } else if(isTimedOut(Status.CHARGING)) {
            // FIXME: This can get stuck in an infinite loop!
            uncharge();
        }
    }

    protected void periodicReloading(Telemetry telemetry) {
        if(checkIsReloaded() || isTimedOut(Status.RELOADING)) {
            // Whether we go to EMTPTY_CHARGED or RELOADED_CHARGED is determined by charge().
            charge();
        } 
    }

    protected void periodicFiring(Telemetry telemetry) {
        final boolean timedOut = isTimedOut(Status.FIRING);
        final boolean isCharged = checkConsideredCharged();
        final boolean isReloaded = checkIsReloaded();
        
        // TODO: Add check for when a projectile leaves to end before timeout
        if(timedOut && isReloaded && isCharged) {
            charge(); // Just make sure that the correct powers are set
            transitionTo(Status.RELOADED_CHARGED);
        } else if(timedOut && !isReloaded && isCharged) {
            charge(); // Just make sure that the correct powers are set
            transitionTo(Status.EMPTY_CHARGED);
        } else if(timedOut && checkConsideredUncharged()) {
            // Using uncharge rather than transition to make sure the powers are set.
            uncharge();
        } else if(timedOut && !isCharged) {
            // The robot is neither charged nor uncharged. It is safer to charge to fire again.
            charge();
        }
    }
    
    protected void periodicAborting(Telemetry telemetry) {
        final boolean timedOut = isTimedOut(Status.ABORTING);
        final boolean isCharged = checkConsideredCharged();
        final boolean isReloaded = checkIsReloaded();
        
        // TODO: Add check for when a projectile leaves to end before timeout
        if(timedOut && isReloaded && isCharged) {
            charge(); // Just making sure the powers are correct.
            transitionTo(Status.RELOADED_CHARGED);
        } else if(timedOut && !isReloaded && isCharged) {
            charge(); // Just making sure the powers are correct.
            transitionTo(Status.EMPTY_CHARGED);
        } else if(timedOut && checkConsideredUncharged()) {
            // Using uncharge rather than transition to make sure the powers are set.
            uncharge();
        } else if(timedOut && !isCharged) {
            // What a Terrible Failure: the robot is neither charged nor uncharged. It is safer to charge
            charge();
        }
    }

    public void setTelemetry(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public Telemetry getTelemetry() {
        return telemetry;
    }

    protected void logExtraTelemetry(Telemetry telemetry) {
        telemetry.addData("checkConsideredUncharged()", checkConsideredUncharged());
        telemetry.addData("checkConsideredCharged()", checkConsideredCharged());
        telemetry.addData("checkIsReloaded()", checkIsReloaded());
        telemetry.addLine();
    }

    @Override
    public void periodic() {
        if(telemetry != null && !SUPPRESS_TELEMTERY) {
            telemetry.addLine(padHeader("FlywheelTubeShooter"));
            telemetry.addLine();
            telemetry.addData("Status", getStatus());
            telemetry.addLine();
        }

        // Handling transition statuses
        switch(getStatus()) {
            case UNKNOWN: 
                periodicUnkown(telemetry);
                break;

            case UNCHARGING: 
                periodicUncharging(telemetry);
                break;
                
            case CHARGING: 
                periodicCharging(telemetry);
                break;
            
            case RELOADING: 
                periodicReloading(telemetry);
                break;
            
            case FIRING: 
                periodicFiring(telemetry);
                break;
            
            case ABORTING: 
                periodicAborting(telemetry);
                break;
            
            default: 
                // Unkown transition state.
        }

        // Updating the motor powers
        if(hasSetFlywheelPower) {
            flywheels.setPower(targetFlyWheelPower);
        }

        if(hasSetFeederPower && feeder != null) {
            feeder.setPower(targetFeederPower);
        }

        // Adding any other telemetry.
        if(telemetry != null && !SUPPRESS_TELEMTERY) {
            logExtraTelemetry(telemetry);
        }
    }
}