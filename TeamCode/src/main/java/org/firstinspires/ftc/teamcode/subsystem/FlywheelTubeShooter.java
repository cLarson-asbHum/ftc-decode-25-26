package org.firstinspires.ftc.teamcode.subsystem;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.PwmControl.PwmRange;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.HashMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static org.firstinspires.ftc.teamcode.util.Util.padHeader;

/**
 * Represents a shooter subsystem with a single set of flywheels as well as a 
 * few independent feeder servos to load projectiles into the flywheels.
 * 
 * NOTE: Any of the "do-a-shooter-thing" methods (i.e `charge()`, `uncharge()`, 
 * `reload()`, `fire()`, `abort()`, and their variants) force the shooter into
 * their new and respective state, without checking the previous state, 
 * overriding what state the shooter may previously have been in. This means 
 * that you could theoretically fire from any state for example, even if undesirable.
 */
@Configurable
public class FlywheelTubeShooter implements ShooterSubsystem {
    /**
     * Constants for the flywheel shooters; namely powers and their tolerance
     */
    public static final class FlywheelConst {
        // TODO: Tune these const!
        public double ticksPerRev = 576.6;
        public double rpm = 312.0;

        public double ticksPerSec() {
            return rpm / 60  * ticksPerRev;
        }

        public double unchargedPower = 0;
        public double chargedPower = 0.6;
        public double reloadingPower = chargedPower;
        public double firingPower = chargedPower;
        public double abortingPower = -0.3;

        public double powerTolerance = 0.05;
    };

    /**
     * Constants for the feeders; namely powers and their tolerance. Applies to
     * both the left and right feeders.
     */
    public static final class FeederConst {
        // TODO: Tune these const!
        public double ticksPerRev = 1; // Because its a servo
        public double maxRpm = 120; // FIXME: I think this constant is wrong...

        public double ticksPerSec() {
            return maxRpm / 60  * ticksPerRev;
        }

        public double unchargedPower = 0;
        public double chargedPower = 0;
        public double reloadingPower = 0.7;
        public double firingPower = 1.0;
        public double abortingPower = -1.0;

        public double powerTolerance = 0.05;
    };

    /**
     * Constants for how long timers ought to last.
     */
    public static final class Timeout {
        public double uncharging = 5.0; // seconds
        public double charging = 5.0; // seconds
        public double reloading = 2.0; // seconds
        public double firing = 1.0; // seconds
        public double multiFiring = Double.POSITIVE_INFINITY; // seconds, but still indefinite
        public double aborting = 1.0; // seconds
    }

    public static FlywheelConst FLYWHEEL_CONST = new FlywheelConst();
    public static FeederConst FEEDER_CONST = new FeederConst();

    /**
     * What is responsible for propelling the projectile very quickly
     */
    private final DcMotorEx flywheels;
    private double targetFlyWheelPower = 0; 
    private boolean hasSetFlywheelPower = false;
    
    /**
     * One of two servos that moves a projectile into the flywheels
     */
    private final CRServo leftFeeder;
    private double targetLeftFeederPower = 0; 
    private boolean hasSetLeftFeederPower = false;
    
    /**
     * One of two servos that moves a projectile into the flywheels
     */
    private final CRServo rightFeeder;
    private double targetRightFeederPower = 0; 
    private boolean hasSetRightFeederPower = false;

    private Status status = Status.UNKNOWN;

    /**
     * Whether any telemetry data ought to be shown, even if a telemetry has 
     * been set. 
     */
    public static boolean SUPPRESS_TELEMTERY = false;
    private Telemetry telemetry = null;

    public static Timeout TIMEOUT = new Timeout();
    private HashMap<Object, Double> timers = new HashMap<>();
    // TODO: Make this use an Injectible-Elapsed time instead of a straight ElapsedTime
    private final ElapsedTime lifetime; // NOTE: Use deltaTime for any implementations, not this!!!
    private double lastTime = 0;        // NOTE: Use deltaTime for any implementations, not this!!!
    private double deltaTime = 0;

    private FlywheelTubeShooter() {
        this.lifetime = new ElapsedTime();
        this.lifetime.startTime();

        // The status is automatically set to UNKNOWN, which transitions to UNCHRAGED.
        // this does our initialization for us.
    }

    public static final class Builder {
        private final DcMotorEx flywheels;
        private CRServo rightFeeder = null;
        private CRServo leftFeeder = null;
        
        public Builder(DcMotorEx flywheels) {
            this.flywheels = flywheels;
        }

        public Builder setRightFeeder(CRServo newRightFeeder) {
            this.rightFeeder = newRightFeeder;
            return this;
        }
        
        public Builder setLeftFeeder(CRServo newLeftFeeder) {
            this.leftFeeder = newLeftFeeder;
            return this;
        }

        public FlywheelTubeShooter build() {
            final FlywheelTubeShooter result = new FlywheelTubeShooter();
            result.flywheels = flywheels;
            result.rightFeeder = rightFeeder;
            result.leftFeeder = leftFeeder;
            return result;
        }
    }

    private boolean setFlywheelPower(double power, double powerTolerance) {
        if(flywheels == null) {
            return false;
        }

        if(Math.abs(power - targetFlyWheelPower) < powerTolerance) {
            return false;
        }

        targetFlyWheelPower = power;
        hasSetFlywheelPower = true;
        return true;
    }

    /**
     * Sets the power of both feeders, equally (assumming neither is already 
     * within tolerance of the target). This is identical to calling 
     * `setLeftFeederPower()` and `setRightFeederPower()` with identical arguments.
     * 
     * No reversal is done to either feeder; both get the same value. To make the 
     * feeders spin in opposite directions, reverse the feeder before construction
     * 
     * @param power The target power for the feeders. Is the same for both
     * @param powerTolerance How far the current power must be in order for the 
     * new target power to override the current power.
     * @return Whether either *or* both feeder's power was changed. 
     */
    private boolean setFeederPower(double power, double powerTolerance) {
        final boolean didRightChange = setRightFeederPower(power, powerTolerance);
        final boolean didLeftChange = setLeftFeederPower(power, powerTolerance);
        return didRightChange || didLeftChange;
    }

    private boolean setLeftFeederPower(double power, double powerTolerance) {
        if(leftFeeder == null) {
            return false;
        }

        if(Math.abs(power - targetLeftFeederPower) < powerTolerance) {
            return false;
        }

        targetLeftFeederPower = power;
        hasSetLeftFeederPower = true;
        return true;
    }
    
    private boolean setRightFeederPower(double power, double powerTolerance) {
        if(rightFeeder == null) {
            return false;
        }

        if(Math.abs(power - targetRightFeederPower) < powerTolerance) {
            return false;
        }

        targetRightFeederPower = power;
        hasSetRightFeederPower = true;
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

    /**
     * Attempts to get the shooter up to speed. The feeders are not moved.
     * 
     * The shooter becomes charged asyncrhonously; to wait for the shooter to 
     * become charged, wait for the state (from `getStatus()` or `getState()`)
     * to become `EMPTY_CHARGED` or `RELOADED_CHARGED`. Note that this may fail
     * and become `UNCHARGING` or `UNCHARGED` if the shooter cannot get up to speed.
     */
    @Override
    public boolean charge() {
        setFeederPower(FEEDER_CONST.chargedPower, FEEDER_CONST.powerTolerance);
        setFlywheelPower(FLYWHEEL_CONST.chargedPower, FLYWHEEL_CONST.powerTolerance);
        startTimeout(Status.CHARGING, TIMEOUT.charging);
        return transitionTo(Status.CHARGING);
    }

    /**
     * Goes to the `CHARGED` state, whether you like it or not. This is in 
     * contrast to `charge()`, which goes to `CHARGING`, which can fail if 
     * the velocity is not sensing correctly.
     * 
     * The state change happens upon invocation, and the shooter is 
     * guaranteed to be either in `EMPTY_CHARGED` or `RELOADED_CHARGED`.
     * 
     * NOTE: This does not guarantee that the velocity will be correct!
     * 
     * @return Whether the transition to `CHARGED` was successful.
     */
    public boolean forceCharged() {
        charge();
        removeTimeout(Status.CHARGING);
        if(checkIsReloaded()) {
            return transitionTo(Status.RELOADED_CHARGED);
        } else {
            return transitionTo(Status.EMPTY_CHARGED);
        }
    }

    /**
     * Attempts to brake the shooter. The feeders are not moved.
     * 
     * The shooter becomes uncharged asyncrhonously; to wait for the shooter to 
     * become fully uncharged, wait for the state (from `getStatus()` or `getState()`)
     * to become `UNCHARGED`. Note that this may fail become `CHARGING` or a charged 
     * state if the shooter cannot get up to speed. 
     */
    @Override
    public boolean uncharge() {
        setFeederPower(FEEDER_CONST.unchargedPower, FEEDER_CONST.powerTolerance);
        setFlywheelPower(FLYWHEEL_CONST.unchargedPower, FLYWHEEL_CONST.powerTolerance);
        startTimeout(Status.UNCHARGING, TIMEOUT.uncharging);
        return transitionTo(Status.UNCHARGING);
    }

    /**
     * Attempts to load a projectile with the feeders. The shooter will attempt 
     * to remain charged. 
     * 
     * A projectile is reloaded asynchronously; to wait for the reloading to 
     * finish, wait for the state (from `getStatus()` or `getState()`) to become 
     * `RELOADED_CHARGED`. Note that this may fail and become `EMPTY_CHARGED`
     * if a projectile is not detected.
     */
    @Override
    public boolean reload() {
        setFeederPower(FEEDER_CONST.reloadingPower, FEEDER_CONST.powerTolerance);
        setFlywheelPower(FLYWHEEL_CONST.reloadingPower, FLYWHEEL_CONST.powerTolerance);
        startTimeout(Status.RELOADING, TIMEOUT.reloading);
        return transitionTo(Status.RELOADING);
    }

    public boolean reloadRight() {
        setRightFeederPower(FEEDER_CONST.reloadingPower, FEEDER_CONST.powerTolerance);
        setLeftFeederPower(FEEDER_CONST.unchargedPower, FEEDER_CONST.powerTolerance);
        setFlywheelPower(FLYWHEEL_CONST.reloadingPower, FLYWHEEL_CONST.powerTolerance);
        startTimeout(Status.RELOADING, TIMEOUT.reloading);
        return transitionTo(Status.RELOADING);
    }
    
    public boolean reloadLeft() {
        setLeftFeederPower(FEEDER_CONST.reloadingPower, FEEDER_CONST.powerTolerance);
        setRightFeederPower(FEEDER_CONST.unchargedPower, FEEDER_CONST.powerTolerance);
        setFlywheelPower(FLYWHEEL_CONST.reloadingPower, FLYWHEEL_CONST.powerTolerance);
        startTimeout(Status.RELOADING, TIMEOUT.reloading);
        return transitionTo(Status.RELOADING);
    }

    /**
     * Moves any loaded projectiles into the shooter (if necessary) and puts the 
     * shooter at the correct power (if necessary). The shooter may already be 
     * at the correct power, if it is charged.
     * 
     * The shooter will automatically end firing after a specified time.
     * 
     * @see `{@link #multiFire()}`
     */
    @Override
    public boolean fire() {
        setFeederPower(FEEDER_CONST.firingPower, FEEDER_CONST.powerTolerance);
        setFlywheelPower(FLYWHEEL_CONST.firingPower, FLYWHEEL_CONST.powerTolerance);
        startTimeout(Status.FIRING, TIMEOUT.firing);
        return transitionTo(Status.FIRING);
    }

    public boolean fireRight() {
        setRightFeederPower(FEEDER_CONST.firingPower, FEEDER_CONST.powerTolerance);
        setLeftFeederPower(FEEDER_CONST.unchargedPower, FEEDER_CONST.powerTolerance);
        setFlywheelPower(FLYWHEEL_CONST.firingPower, FLYWHEEL_CONST.powerTolerance);
        startTimeout(Status.FIRING, TIMEOUT.firing);
        return transitionTo(Status.FIRING);
    }
    
    public boolean fireLeft() {
        setLeftFeederPower(FEEDER_CONST.firingPower, FEEDER_CONST.powerTolerance);
        setRightFeederPower(FEEDER_CONST.unchargedPower, FEEDER_CONST.powerTolerance);
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
     * @return Whether the state was successfully transitioned to.
     */
    public boolean multiFire() {
        setFeederPower(FEEDER_CONST.firingPower, FEEDER_CONST.powerTolerance);
        setFlywheelPower(FLYWHEEL_CONST.firingPower, FLYWHEEL_CONST.powerTolerance);
        startTimeout(Status.FIRING, TIMEOUT.multiFiring);
        return transitionTo(Status.FIRING);
    }

    public boolean multiFireRight() {
        setRightFeederPower(FEEDER_CONST.firingPower, FEEDER_CONST.powerTolerance);
        setLeftFeederPower(FEEDER_CONST.unchargedPower, FEEDER_CONST.powerTolerance);
        setFlywheelPower(FLYWHEEL_CONST.firingPower, FLYWHEEL_CONST.powerTolerance);
        startTimeout(Status.FIRING, TIMEOUT.multiFiring);
        return transitionTo(Status.FIRING);
    }

    public boolean multiFireLeft() {
        setLeftFeederPower(FEEDER_CONST.firingPower, FEEDER_CONST.powerTolerance);
        setRightFeederPower(FEEDER_CONST.unchargedPower, FEEDER_CONST.powerTolerance);
        setFlywheelPower(FLYWHEEL_CONST.firingPower, FLYWHEEL_CONST.powerTolerance);
        startTimeout(Status.FIRING, TIMEOUT.multiFiring);
        return transitionTo(Status.FIRING);
    }

    /**
     * Attempts to remove any mistaken projectiles. The feeders are moved in order
     * to remove the projectile. This is not the same as firing, which attempts to 
     * score; this is instead done in any way that gets a projectile out from firing.
     * 
     * Aborting is doen asynchronously; to wait for the aborting to complete, wait 
     * for when the state is no longer `ABORTING`.
     */
    @Override
    public boolean abort() {
        setFeederPower(FEEDER_CONST.abortingPower, FEEDER_CONST.powerTolerance);
        setFlywheelPower(FLYWHEEL_CONST.abortingPower, FLYWHEEL_CONST.powerTolerance);
        startTimeout(Status.ABORTING, TIMEOUT.aborting);
        return transitionTo(Status.ABORTING);
    }

    public boolean abortLeft() {
        setLeftFeederPower(FEEDER_CONST.abortingPower, FEEDER_CONST.powerTolerance);
        setRightFeederPower(FEEDER_CONST.unchargedPower, FEEDER_CONST.powerTolerance);
        setFlywheelPower(FLYWHEEL_CONST.abortingPower, FLYWHEEL_CONST.powerTolerance);
        startTimeout(Status.ABORTING, TIMEOUT.aborting);
        return transitionTo(Status.ABORTING);
    }

    public boolean abortRight() {
        setRightFeederPower(FEEDER_CONST.abortingPower, FEEDER_CONST.powerTolerance);
        setLeftFeederPower(FEEDER_CONST.unchargedPower, FEEDER_CONST.powerTolerance);
        setFlywheelPower(FLYWHEEL_CONST.abortingPower, FLYWHEEL_CONST.powerTolerance);
        startTimeout(Status.ABORTING, TIMEOUT.aborting);
        return transitionTo(Status.ABORTING);
    }

    /**
     * Determines whether the shooter is fully uncharged. This is determined solely
     * based on whether the current shooter velocity is within tolerance of the 
     * `unchargedPower` target (from `FlywheelConst`).
     * 
     * NOTE: This is **not** the opposite of `checkConsideredCharged()`, as a 
     * velocity in-between the charged and uncharged velocities may be possible.
     * 
     * @return Whether the current velocity is close enough to fully uncharged.
     */
    public boolean checkConsideredUncharged() {
        final double currentPower = flywheels.getVelocity() / FLYWHEEL_CONST.ticksPerSec();
        return Math.abs(currentPower - FLYWHEEL_CONST.unchargedPower) < FLYWHEEL_CONST.powerTolerance;
    }

    
    /**
     * Determines whether the shooter is fully charged. This is determined solely
     * based on whether the current shooter velocity is within tolerance of the 
     * `chargedPower` target (from `FlywheelConst`).
     * 
     * NOTE: This is **not** the opposite of `checkConsideredUncharged()`, as a 
     * velocity in-between the charged and uncharged velocities may be possible.
     * 
     * @return Whether the current velocity is close enough to fully uncharged.
     */
    public boolean checkConsideredCharged() {
        final double currentPower = flywheels.getVelocity() / FLYWHEEL_CONST.ticksPerSec();
        return Math.abs(currentPower - FLYWHEEL_CONST.chargedPower) < FLYWHEEL_CONST.powerTolerance;
    }

    /**
     * Determines whether a projectile is immediately ready for shooting. This 
     * currently checks based off of color and distance whether a projectile is 
     * underneath a feeder. 
     * 
     * For a double-barreled shooter, this checks if *either* barrel is reloaded.
     * 
     * @return Whether at least one projectile is immediately ready.
     */
    public boolean checkIsReloaded() {
        // FIXME: We always assume the shooter is reloaded (even though it isn't!)
        return true;
    }

    /**
     * Creates a timer that counts down to 0. This is useful for creating states
     * that last a specifed amoutn of time. 
     * 
     * In order for the timer to be updated, this method 
     * 
     * Calling this when a timer with the same label is still active will reset 
     * the timer. The timer will be reset to the new `durationSec`.
     * 
     * @param label The key with which to find the specific timer.
     * @param durationSec Number of seconds the timer lasts.
     */
    private void startTimeout(Object label, double durationSec) {
        timers.put(label, durationSec);
    }

    /**
     * Removes a timer created by startTimeout. Nothing happens if the timer
     * does not exist.
     * 
     * @param label The key of the timer
     * @return Whether a timer was actually removed. False if one didn't exist.
     */
    private boolean removeTimeout(Object label) {
        return timers.remove(label) != null;
    }

    /**
     * Checks whether a timer with the given label has expired. If no timer with 
     * the label exists, then this returns true. 
     * 
     * This mthod 
     * 
     * @param label The key of the timer
     * @return
     */
    public boolean isTimedOut(Object label) {
        // Updating all timers
        for(final Object foundLabel : timers.keySet()) {
            timers.put(foundLabel, timers.get(foundLabel) - deltaTime);
        }

        // If the timer exists and is above 0, then we are not timed out
        if(timers.containsKey(label) && timers.get(label) > 0) {
            return false;
        }

        // We are timed out; remove the timer
        timers.remove(label);
        return true;
    }

    /**
     * Transitions from `UNKNOWN` to `UNCHARGING`, guaranteeing a state.
     * 
     * @param telemetry Unused
     */
    protected void periodicUnknown(Telemetry telemetry) {
        // Just go to uncharged and call it a day.
        // if(feeder != null) {
        //     feeder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // }
        // flywheels.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // FIXME: No encoder connected!
        uncharge();
    }

    /**
     * Sets the target powers of the shooter to 0(-ish), makes the feeders still, 
     * and waits until the shooter is nearly still. Will transition to `CHARGING`
     * if it takes too long. 
     * 
     * @param telemetry Unused
     */
    protected void periodicUncharging(Telemetry telemetry) {
        // Transitioning to UNCHARGED if we are within power
        if(checkConsideredUncharged()) {
            transitionTo(Status.UNCHARGED);
        } else if(isTimedOut(Status.UNCHARGING)) {
            // FIXME: This can still maintain charging power.
            // FIXME: This can get stuck in an infinite loop!
            charge();
        }
    }

    /**
     * Sets the target powers of the shooter to fast(-ish), makes the feeders 
     * still, and waits until the shooter is nearly still. Will transition to 
     * `UNCHARGING` if it takes too long. 
     * 
     * @param telemetry Unused
     */
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

    /**
     * Keeps the shooter at a charged power, moves the feeders to intake a 
     * projectile, and waits until a projectile or projectiles are ready to be 
     * shot. Goes to `CHARGING` thereafter.
     * 
     * This does not shoot a projectile.
     * 
     * @param telemetry Unused
     */
    protected void periodicReloading(Telemetry telemetry) {
        if(checkIsReloaded() || isTimedOut(Status.RELOADING)) {
            // Whether we go to EMTPTY_CHARGED or RELOADED_CHARGED is determined by charge().
            charge();
        } 
    }

    /**
     * Keeps the shooter at firing speed and feeders reloading until the time
     * has elapsed. Goes to `RELOADED_CHARGED`, `EMPTY_CHARGED`, or `CHARGING` 
     * afterwards.
     * 
     * @param telemetry Unused
     */
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
        }
        //  else if(timedOut && checkConsideredUncharged()) {
        //     // Using uncharge rather than transition to make sure the powers are set.
        //     uncharge();
        // } 
        else if(timedOut && !isCharged) {
            // We want to ensure that the shooter is charged because we are likely to fire again.
            charge();
        }
    }
    
    /** 
     * Keeps the shooter and the feeders removing any projectiles that were 
     * misplaced. This ends after a specified amount of time. The shooter
     * can go to the following afterwards:
     * 
     *   - `RELOADED_CHARGED`, if the shooter is charged and a projectile is ready;
     *   - `EMPTY_CHARGED`, if the shooter is charged but there's no projectile;
     *   - `UNCHARGING`, if the shooter is not already charged
     * 
     * @param telemetry Unused. 
     */
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
        }
        // else if(timedOut && checkConsideredUncharged()) {
        //     // Using uncharge rather than transition to make sure the powers are set.
        //     uncharge();
        // } 
        else if(timedOut && !isCharged) {
            // The robot is neither charged nor uncharged. It is safer to uncharge
            uncharge();
        }
    }

    /**
     * Sets the location any data will be logged. Update() or clear() is never
     * called by this subsystem. 
     * 
     * @param telemetry Where any data shall be logged. Can be null. 
     */
    public void setTelemetry(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    /**
     * Gets the most recent telemetry given by `setTelemetry()`
     * 
     * @return Where data is being logged. Defaults to null.
     */
    public Telemetry getTelemetry() {
        return telemetry;
    }

    protected void logExtraTelemetry(Telemetry telemetry) {
        // FIXME: This may cause uncessary slowdown
        telemetry.addData("lifetime", lifetime.seconds());
        telemetry.addData("Current Timer", timers.get(getStatus()));
        telemetry.addData("isTimedOut()", isTimedOut(getStatus()));
        telemetry.addLine();

        telemetry.addData("checkConsideredUncharged()", checkConsideredUncharged());
        telemetry.addData("checkConsideredCharged()", checkConsideredCharged());
        telemetry.addData("checkIsReloaded()", checkIsReloaded());
        telemetry.addLine();

        telemetry.addData("targetFlyWheelPower", targetFlyWheelPower);
        telemetry.addData("actualFlywheelPower", flywheels.getPower());
        if(leftFeeder != null) {
            telemetry.addData("targetLeftFeederPower", targetLeftFeederPower);
            telemetry.addData("actualLeftFeederPower", leftFeeder.getPower());
        }
        if(rightFeeder != null) {
            telemetry.addData("targetRightFeederPower", targetRightFeederPower);
            telemetry.addData("actualRightFeederPower", rightFeeder.getPower());
        }
        telemetry.addLine();
    }

    @Override
    public void periodic() {
        // Keeping record of time
        final double currentTime = lifetime.seconds();
        deltaTime = currentTime - lastTime; 
        lastTime = currentTime;

        if(telemetry != null && !SUPPRESS_TELEMTERY) {
            telemetry.addLine(padHeader("FlywheelTubeShooter"));
            telemetry.addLine();
            telemetry.addData("Status", getStatus());
            telemetry.addLine();
        }

        // Handling transition statuses
        switch(getStatus()) {
            case UNKNOWN: 
                periodicUnknown(telemetry);
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
            hasSetFlywheelPower = false;
        }

        if(hasSetLeftFeederPower && leftFeeder != null) {
            leftFeeder.setPower(targetLeftFeederPower);
            hasSetLeftFeederPower = false;
        }
        
        if(hasSetRightFeederPower && rightFeeder != null) {
            rightFeeder.setPower(targetRightFeederPower);
            hasSetRightFeederPower = false;
        }
        
        // Adding any other telemetry.
        if(telemetry != null && !SUPPRESS_TELEMTERY) {
            logExtraTelemetry(telemetry);
        }
    }
}