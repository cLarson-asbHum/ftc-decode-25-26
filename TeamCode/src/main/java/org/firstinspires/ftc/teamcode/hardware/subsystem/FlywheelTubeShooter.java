package org.firstinspires.ftc.teamcode.hardware.subsystem;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.PwmControl.PwmRange;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.function.DoubleUnaryOperator;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.temp.TimeInjectionUtil;
import org.firstinspires.ftc.teamcode.util.ArtifactColor;
import org.firstinspires.ftc.teamcode.util.ArtifactColorGetter;
import org.firstinspires.ftc.teamcode.hardware.DcMotorGroup;
import org.firstinspires.ftc.teamcode.util.Util;

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
        public double ticksPerRev = 28;
        public double rpm = 5400;

        public double ticksPerSec() {
            return rpm / 60  * ticksPerRev;
        }

        public double unchargedPower = 0; // Ticks / sec
        // public double chargedPower = 2000; // Ticks / sec, tuned for 24 ins away
        public double chargedPower = 1366; // Ticks / sec, tuned for 24 ins away
        public double reloadingPower = chargedPower; // Ticks / sec
        public double firingPower = chargedPower; // Ticks / sec
        public double abortingPower = 0.3 * ticksPerSec(); // Ticks / sec

        public double powerTolerance = 25; // Ticks / sec
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
        public double chargedPower = -0.02;
        public double reloadingPower = 0.3;
        public double autoReloadingPower = 0.2;
        public double firingPower = 1.0;
        public double preventMisfire = -0.8;
        public double abortingPower = -1.0;

        public double powerTolerance = 0.01;
    };

    /**
     * Constants for how long timers ought to last.
     */
    public static final class Timeout {
        public double uncharging = Double.POSITIVE_INFINITY; // seconds
        public double charging = Double.POSITIVE_INFINITY; // seconds
        public double reloading = 1.0; // seconds
        public double autoReloading = 1.0; // seconds
        public double finishingReloading = 0.0; // seconds; happends after we have a projectile
        public double firing = 1.5; // seconds
        public double multiFiring = Double.POSITIVE_INFINITY; // seconds, but still indefinite
        public double aborting = 3.0; // seconds
    }

    /**
     * Describes specifcally how the shooter is reloading at the current point in time.
     * Except for `UNKNOWN`, these states will only be seen during `Status.RELOADING` 
     */
    public static enum ReloadingState {
        /**
         * The shooter is not reloading or the state cannot be determined.
         */
        UNKNOWN, 

        /**
         * The left barrel of the shooter is being reloaded. No projectile has been seen yet.
         */
        RELOADING_LEFT,

        /**
         * The right barrel of the shooter is being reloaded. No projectile has been seen yet.
         */
        RELOADING_RIGHT,

        /**
         * Both barrels are being reloaded at the same time. No projectile in either barrel has 
         * been seen yet.
         */
        RELOADING_BOTH,

        /**
         * A projectile has been seen somewhere, and we are making sure that it is comepletely 
         * secure under the feeder (because it might have been seen too early otherwise).
         */
        FINISHING_RELOAD
    }

    /** 
     * Describes more specifically how the shooter is shooting. Besides `UNKNOWN`,
     * these states will only be seen during firing
     */
    public static enum FiringState {
        /**
         * We are not firing.
         */
        UNKNOWN,

        /**
         * Only the left chamber is firing. The right is not
         */
        FIRING_LEFT,

        /**
         * Only the right chamber is firing. The left is not
         */
        FIRING_RIGHT,

        /**
         * Both chambers are being fired
         */
        FIRING_BOTH
    }

    public static FlywheelConst FLYWHEEL_CONST = new FlywheelConst();
    public static FeederConst FEEDER_CONST = new FeederConst();

    private final DoubleUnaryOperator ticksToInches;
    private final DoubleUnaryOperator inchesToTicks;

    /**
     * What is responsible for propelling the projectile very quickly
     */
    private final DcMotorGroup flywheels;
    private double targetFlyWheelPower = 0; 
    private double chargedSpeed = FLYWHEEL_CONST.chargedPower;
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

    private final ArtifactColorGetter rightReloadClassifier;
    private final ArtifactColorGetter leftReloadClassifier;

    private Status status = Status.UNKNOWN;
    private ReloadingState reloadingState = ReloadingState.UNKNOWN;
    private FiringState firingState = FiringState.UNKNOWN;

    /**
     * Whether any telemetry data ought to be shown, even if a telemetry has 
     * been set. 
     */
    public static boolean SUPPRESS_TELEMTERY = false;
    private Telemetry telemetry = null;

    public static Timeout TIMEOUT = new Timeout();
    private HashMap<Object, Double> timers = new HashMap<>();
    private final ElapsedTime lifetime; // NOTE: Use deltaTime for any implementations, not this!!!
    private double lastTime = 0;        // NOTE: Use deltaTime for any implementations, not this!!!
    private double deltaTime = 0;
    private boolean isReadyToStartLifetime = true;

    private FlywheelTubeShooter(Builder builder) {
        this.lifetime = TimeInjectionUtil.getElapsedTime();
        this.flywheels = new DcMotorGroup(builder.flywheels.toArray(new DcMotorEx[0]));
        this.rightFeeder = builder.rightFeeder;
        this.leftFeeder = builder.leftFeeder;
        this.rightReloadClassifier = builder.rightReloadClassifier;
        this.leftReloadClassifier = builder.leftReloadClassifier;
        this.inchesToTicks = builder.inchesToTicks;
        this.ticksToInches = builder.ticksToInches;
    }

    public static final class Builder {
        public final ArrayList<DcMotorEx> flywheels = new ArrayList<DcMotorEx>();
        public CRServo rightFeeder = null;
        public CRServo leftFeeder = null;
        public ArtifactColorGetter rightReloadClassifier = null;
        public ArtifactColorGetter leftReloadClassifier = null;
        public DoubleUnaryOperator ticksToInches = DoubleUnaryOperator.identity();
        public DoubleUnaryOperator inchesToTicks = DoubleUnaryOperator.identity();
        
        public Builder() {
            // Do nothing; everything is initialized with the fields
        }

        public Builder(DcMotorEx... flywheels) {
            addMotors(flywheels);
        }

        public Builder addMotors(DcMotorEx... flywheels) {
            for(final DcMotorEx flywheel : flywheels) {
                this.flywheels.add(flywheel);
            }
            return this;
        }

        public Builder setRightFeeder(CRServo newRightFeeder) {
            this.rightFeeder = newRightFeeder;
            return this;
        }
        
        public Builder setLeftFeeder(CRServo newLeftFeeder) {
            this.leftFeeder = newLeftFeeder;
            return this;
        }

        public Builder setRightReloadClassifier(ArtifactColorGetter rightReloadClassifier) {
            this.rightReloadClassifier = rightReloadClassifier;
            return this;
        }

        public Builder setLeftReloadClassifier(ArtifactColorGetter leftReloadClassifier) {
            this.leftReloadClassifier = leftReloadClassifier;
            return this;
        }

        public Builder setTicksToInches(DoubleUnaryOperator ticksToInches) {
            this.ticksToInches = ticksToInches;
            return this;
        }
        
        public Builder setInchesToTicks(DoubleUnaryOperator inchesToTicks) {
            this.inchesToTicks = inchesToTicks ;
            return this;
        }

        /**
         * Creates the shooter. This throws if no flywheels were added. 
         * 
         * @return New shooter, given the builder's parameters
         */
        public FlywheelTubeShooter build() {
            if(flywheels.size() == 0) { 
                throw new IllegalArgumentException("Number of flywheels added to builder is 0");
            }

            return new FlywheelTubeShooter(this);
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
     * Guarantees that the target flywheel power is set to the given target.
     * 
     * @param power The target speed, in ticks per second.
     * @param powerTolerance Used only in the return value.
     * @return Whether the power would've been changed were tolerance being used.
     */
    private boolean forceFlywheelPower(double power, double powerTolerance) {
        if(flywheels == null) {
            return false;
        }

        final boolean wouldveChanged = Math.abs(power - targetFlyWheelPower) < powerTolerance;
        targetFlyWheelPower = power;
        hasSetFlywheelPower = true;
        return !wouldveChanged;
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
        return transitionTo(newStatus, ReloadingState.UNKNOWN);
    }
    
    private boolean transitionTo(Status newStatus, ReloadingState newReloadingState) {
        status = newStatus; 
        reloadingState = newReloadingState;
        firingState = FiringState.UNKNOWN;
        return true;
    }

    private boolean transitionTo(Status newStatus, FiringState newFiringState) {
        status = newStatus; 
        reloadingState = ReloadingState.UNKNOWN;
        firingState = newFiringState;
        return true;
    }

    @Override
    public Status getStatus() {
        return this.status;
    }

    public ReloadingState getReloadingState() {
        return this.reloadingState;
    }

    public FiringState getFiringState() {
        return this.firingState;
    }

    /**
     * Attempts to get the shooter up to speed. The feeders are not moved.
     * 
     * The shooter becomes charged asyncrhonously; to wait for the shooter to 
     * become charged, wait for the state (from `getStatus()` or `getState()`)
     * to become `CHARGED`. Note that this may fail and become `UNCHARGING` 
     * or `UNCHARGED` if the shooter cannot get up to speed.
     */
    @Override
    public boolean charge() {
        setFeederPower(FEEDER_CONST.chargedPower, FEEDER_CONST.powerTolerance);
        setFlywheelPower(FLYWHEEL_CONST.chargedPower, FLYWHEEL_CONST.powerTolerance);
        chargedSpeed = FLYWHEEL_CONST.chargedPower;
        startTimeout(Status.CHARGING, TIMEOUT.charging);
        return transitionTo(Status.CHARGING);
    }

    /**
     * Attempst ot get the shooter to shoot at the given speed. The feeders
     * are not moved. If the current shooter speed is already within 
     * tolerance, the state remains unchanged. 
     * 
     * When true, the `forceTargetUpdate` forces the change of the speed target
     * even when it is already within tolerance. Keeping it as false can improve
     * code performance but may decrease accuracy.
     * 
     * The shooter becomes charged asynchronously; to wait for the shooter to 
     * become charged, wait for the state (from `getStatus()` or `getState()`)
     * to become `CHARGED`. Note that this may fail and become `UNCHARGING` 
     * or `UNCHARGED` if the shooter cannot get up to speed.
     * 
     * @param speed How fast a projectile should exit, in inches per second.
     * @param forceTargetUpdate True if the target speed should be updated 
     * even if the current speed is already within tolerance of the new target.
     * @return Whether the state was changed. False if already near new target
     */
    public boolean charge(double inchesPerSec, boolean forceTargetUpdate) {
        // Setting the correct powers
        final double nativeTargetSpeed = inchesToTicks.applyAsDouble(inchesPerSec);
        boolean didChangePower = false; // Placeholder value only

        if(forceTargetUpdate) {
            didChangePower = forceFlywheelPower(nativeTargetSpeed, FLYWHEEL_CONST.powerTolerance);
        } else {
            didChangePower = setFlywheelPower(nativeTargetSpeed,FLYWHEEL_CONST.powerTolerance);
        }

        chargedSpeed = nativeTargetSpeed;
        setFeederPower(FEEDER_CONST.chargedPower, FEEDER_CONST.powerTolerance);

        // Changing the state
        if(didChangePower) {
            startTimeout(Status.CHARGING, TIMEOUT.charging);
            transitionTo(Status.CHARGING);
        }

        return didChangePower;
    }

    private boolean ticksCharge(double ticksPerSec) {
        setFeederPower(FEEDER_CONST.chargedPower, FEEDER_CONST.powerTolerance);
        setFlywheelPower(ticksPerSec, FLYWHEEL_CONST.powerTolerance);
        chargedSpeed = ticksPerSec;
        startTimeout(Status.CHARGING, TIMEOUT.charging);
        return transitionTo(Status.CHARGING);
    }

    /** 
     * Gets the speed of the shooter. If there are multiple flywheels, their 
     * speeds are averaged
     * 
     * @return Average speed of the flywheels, in inches per second.
     */
    public double getSpeed() {
        return ticksToInches.applyAsDouble(flywheels.getVelocity());
    }

    /**
     * Goes to the `CHARGED` state, whether you like it or not. This is in 
     * contrast to `charge()`, which goes to `CHARGING`, which can fail if 
     * the velocity is not sensing correctly.
     * 
     * The state change happens upon invocation, and the shooter is 
     * guaranteed to be either in `CHARGED`.
     * 
     * NOTE: This does not guarantee that the velocity will be correct!
     * 
     * @return Whether the transition to `CHARGED` was successful.
     */
    public boolean forceCharged() {
        charge();
        removeTimeout(Status.CHARGING);
        return transitionTo(Status.CHARGED);
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
     * `CHARGED`. This does not guarantee that a projectile will be successfully 
     * loaded, as the reloading may time out before anything could be processed.
     */
    @Override
    public boolean reload() {
        setFeederPower(FEEDER_CONST.reloadingPower, FEEDER_CONST.powerTolerance);
        setFlywheelPower(chargedSpeed, FLYWHEEL_CONST.powerTolerance);
        startTimeout(Status.RELOADING, TIMEOUT.reloading);
        return transitionTo(Status.RELOADING, ReloadingState.RELOADING_BOTH);
    }

    public boolean reloadRight() {
        setRightFeederPower(FEEDER_CONST.reloadingPower, FEEDER_CONST.powerTolerance);
        setLeftFeederPower(FEEDER_CONST.unchargedPower, FEEDER_CONST.powerTolerance);
        setFlywheelPower(chargedSpeed, FLYWHEEL_CONST.powerTolerance);
        startTimeout(Status.RELOADING, TIMEOUT.reloading);
        return transitionTo(Status.RELOADING, ReloadingState.RELOADING_RIGHT);
    }
    
    public boolean reloadLeft() {
        setLeftFeederPower(FEEDER_CONST.reloadingPower, FEEDER_CONST.powerTolerance);
        setRightFeederPower(FEEDER_CONST.unchargedPower, FEEDER_CONST.powerTolerance);
        setFlywheelPower(chargedSpeed, FLYWHEEL_CONST.powerTolerance);
        startTimeout(Status.RELOADING, TIMEOUT.reloading);
        return transitionTo(Status.RELOADING, ReloadingState.RELOADING_LEFT);
    }

    public boolean reloadEmpty() {
        final boolean leftReloaded = this.checkIsLeftReloaded();
        final boolean rightReloaded = this.checkIsRightReloaded();

        if(!leftReloaded && !rightReloaded) {
            return this.reload();
        } 

        if(!leftReloaded && rightReloaded) {
            return this.reloadLeft();
        }

        if(leftReloaded && !rightReloaded) {
            return this.reloadRight();
        }
    
        return false;
    }

    public boolean autoReload() {
        final boolean leftReloaded = this.checkIsLeftReloaded();
        final boolean rightReloaded = this.checkIsRightReloaded();

        // If both are reloaded, exit early and return false
        if(leftReloaded && rightReloaded) {
            return false;
        }

        // Getting what powers we need
        ReloadingState newReloadState = ReloadingState.RELOADING_BOTH; // Placeholder
        double leftFeederPower = FEEDER_CONST.unchargedPower;
        double rightFeederPower = FEEDER_CONST.unchargedPower;

        if(!leftReloaded && !rightReloaded) {
            leftFeederPower = FEEDER_CONST.autoReloadingPower;
            rightFeederPower = FEEDER_CONST.autoReloadingPower;
            newReloadState = ReloadingState.RELOADING_BOTH;
        } 

        if(!leftReloaded && rightReloaded) {
            leftFeederPower = FEEDER_CONST.autoReloadingPower;
            newReloadState = ReloadingState.RELOADING_LEFT;
        }

        if(leftReloaded && !rightReloaded) {
            rightFeederPower = FEEDER_CONST.autoReloadingPower;
            newReloadState = ReloadingState.RELOADING_RIGHT;
        }

        // getAsDoubleing the found powers
        setRightFeederPower(rightFeederPower, FEEDER_CONST.powerTolerance);
        setLeftFeederPower(leftFeederPower, FEEDER_CONST.powerTolerance);
        setFlywheelPower(chargedSpeed, FLYWHEEL_CONST.powerTolerance);
        startTimeout(Status.RELOADING, TIMEOUT.autoReloading);
        return transitionTo(Status.RELOADING, newReloadState);
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
        setFlywheelPower(chargedSpeed, FLYWHEEL_CONST.powerTolerance);
        startTimeout(Status.FIRING, TIMEOUT.firing);
        return transitionTo(Status.FIRING, FiringState.FIRING_BOTH);
    }

    public boolean fireRight() {
        setRightFeederPower(FEEDER_CONST.firingPower, FEEDER_CONST.powerTolerance);
        setLeftFeederPower(FEEDER_CONST.unchargedPower, FEEDER_CONST.powerTolerance);
        setFlywheelPower(chargedSpeed, FLYWHEEL_CONST.powerTolerance);
        startTimeout(Status.FIRING, TIMEOUT.firing);
        return transitionTo(Status.FIRING, FiringState.FIRING_RIGHT);
    }
    
    public boolean fireLeft() {
        setLeftFeederPower(FEEDER_CONST.firingPower, FEEDER_CONST.powerTolerance);
        setRightFeederPower(FEEDER_CONST.unchargedPower, FEEDER_CONST.powerTolerance);
        setFlywheelPower(chargedSpeed, FLYWHEEL_CONST.powerTolerance);
        startTimeout(Status.FIRING, TIMEOUT.firing);
        return transitionTo(Status.FIRING, FiringState.FIRING_LEFT);
    }

    /**
     * Attempts to shoot the sepecific side so that a green artifact is fired. 
     * If no side has a green artifact, then this does nothing and returns 
     * false. If there are two greens, only the right side is fired.
     * 
     * If both artifact color getters are null, then this method acts identical 
     * to `fire()` except that it always returns true.
     * 
     * @return Whether any firing was done. Also returns true even if already in
     * the `FIRING` state.
     */
    public boolean fireGreen() {
        // Firing if both sides cannot sense artifacts
        // This is to prevent not being able to fire because we sense nothing
        if(rightReloadClassifier == null && leftReloadClassifier == null) {
            fire();
            return true;
        }

        // Doing the regular color checks
        if(rightReloadClassifier.getColor() == ArtifactColor.GREEN) {
            fireRight();
            return true;
        }

        if(leftReloadClassifier.getColor() == ArtifactColor.GREEN) {
            fireLeft();
            return true;
        }

        // No side is loaded with the correct color.; do nothing
        return false;
    }

    /**
     * Attempts to shoot the sepecific side so that a purple artifact is fired.
     * If no side has a purple artifact, then this does nothing and returns 
     * false. If there are two purple, only the right side is fired.
     * 
     * If both artifact color getters are null, then this method acts identical 
     * to `fire()` except that it always returns true.
     * 
     * @return Whether any firing was done. Also returns true even if already in
     * the `FIRING` state.
     */
    public boolean firePurple() {
        // Firing if both sides cannot sense artifacts
        // This is to prevent not being able to fire because we sense nothing
        if(rightReloadClassifier == null && leftReloadClassifier == null) {
            fire();
            return true;
        }

        // Checking that either barrel has purple
        if(rightReloadClassifier.getColor() == ArtifactColor.PURPLE) {
            fireRight();
            return true;
        }

        if(leftReloadClassifier.getColor() == ArtifactColor.PURPLE) {
            fireLeft();
            return true;
        }

        // No side is loaded with the correct color.; do nothing
        return false;
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
        setFlywheelPower(chargedSpeed, FLYWHEEL_CONST.powerTolerance);
        startTimeout(Status.FIRING, TIMEOUT.multiFiring);
        return transitionTo(Status.FIRING, FiringState.FIRING_BOTH);
    }

    public boolean multiFireRight() {
        setRightFeederPower(FEEDER_CONST.firingPower, FEEDER_CONST.powerTolerance);
        setLeftFeederPower(FEEDER_CONST.unchargedPower, FEEDER_CONST.powerTolerance);
        setFlywheelPower(chargedSpeed, FLYWHEEL_CONST.powerTolerance);
        startTimeout(Status.FIRING, TIMEOUT.multiFiring);
        return transitionTo(Status.FIRING, FiringState.FIRING_RIGHT);
    }

    public boolean multiFireLeft() {
        setLeftFeederPower(FEEDER_CONST.firingPower, FEEDER_CONST.powerTolerance);
        setRightFeederPower(FEEDER_CONST.unchargedPower, FEEDER_CONST.powerTolerance);
        setFlywheelPower(chargedSpeed, FLYWHEEL_CONST.powerTolerance);
        startTimeout(Status.FIRING, TIMEOUT.multiFiring);
        return transitionTo(Status.FIRING, FiringState.FIRING_LEFT);
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
        // final double currentPower = flywheels.getVelocity() / FLYWHEEL_CONST.ticksPerSec();
        return Math.abs(flywheels.getVelocity() - FLYWHEEL_CONST.unchargedPower) < FLYWHEEL_CONST.powerTolerance;
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
        // final double currentPower = flywheels.getVelocity() / FLYWHEEL_CONST.ticksPerSec();
        // return Math.abs(flywheels.getVelocity() - FLYWHEEL_CONST.chargedPower) < FLYWHEEL_CONST.powerTolerance;
        return Util.any(
            Arrays.asList(flywheels.getMotors()), 
            (motor) -> Math.abs(motor.getVelocity() - chargedSpeed) < FLYWHEEL_CONST.powerTolerance
        );
    }

    /**
     * Checks motor 1
     * @return
     */
    public boolean checkLeftConsideredCharged() {
        final double velocity = flywheels.getMotors()[1].getVelocity();
        return Math.abs(velocity - chargedSpeed) < FLYWHEEL_CONST.powerTolerance;
    }

    /**
     * Checks motor 0
     * @return
     */
    public boolean checkRightConsideredCharged() {
        final double velocity = flywheels.getMotors()[0].getVelocity();
        return Math.abs(velocity - chargedSpeed) < FLYWHEEL_CONST.powerTolerance;
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
        return checkIsRightReloaded() || checkIsLeftReloaded();
    }

    public boolean checkIsRightReloaded() {
        if(rightReloadClassifier == null) {
            return true; // Assume loaded to avoid falling into an unfirable state
        }

        return rightReloadClassifier.getColor() != ArtifactColor.UNKNOWN;
    }
    
    public boolean checkIsLeftReloaded() {
        if(leftReloadClassifier == null) {
            return true; // Assume loaded to avoid falling into an unfirable state
        }

        return leftReloadClassifier.getColor() != ArtifactColor.UNKNOWN;
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
            ticksCharge(chargedSpeed);
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
        // Transitioning to CHARGED if we are within power
        if(checkConsideredCharged()) {
            transitionTo(Status.CHARGED);
        } else if(isTimedOut(Status.CHARGING)) {
            // FIXME: This can get stuck in an infinite loop!
            uncharge();
        }
    }

    protected void periodicCharged(Telemetry telemetry) {
        // Going to the charging state if the velocity changes too much
        if(!Util.near(flywheels.getVelocity(), chargedSpeed, FLYWHEEL_CONST.powerTolerance)) {
            ticksCharge(chargedSpeed);
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
        // Handling how specifically we are going to reload
        switch(reloadingState) {
            case RELOADING_BOTH:
                if(checkIsReloaded()) {
                    startTimeout(Status.RELOADING, TIMEOUT.finishingReloading);
                    transitionTo(Status.RELOADING, ReloadingState.FINISHING_RELOAD);
                }
                break;
                
            case RELOADING_LEFT:
                if(checkIsLeftReloaded()) {
                    startTimeout(Status.RELOADING, TIMEOUT.finishingReloading);
                    transitionTo(Status.RELOADING, ReloadingState.FINISHING_RELOAD);
                }
                break;
                
            case RELOADING_RIGHT:
                if(checkIsRightReloaded()) {
                    startTimeout(Status.RELOADING, TIMEOUT.finishingReloading);
                    transitionTo(Status.RELOADING, ReloadingState.FINISHING_RELOAD);
                }
                break;
                
            case FINISHING_RELOAD:
                // Moving the projectiles just a little bit more
                // We do reset the timeout before transitioning to this.
                break;

            case UNKNOWN:
            default:
                // Just waiting for timeout
                break;
        }

        // Exiting from reloading automatically.
        if(isTimedOut(Status.RELOADING)) {
            // Rather than just jumping to CHARGED, we charge again just to be sure.
            ticksCharge(chargedSpeed);
        } 
    }

    /**
     * Keeps the shooter at firing speed and feeders reloading until the time
     * has elapsed. Goes to `CHARGED` or `CHARGING` afterwards.
     * 
     * @param telemetry Unused
     */
    protected void periodicFiring(Telemetry telemetry) {
        final boolean timedOut = isTimedOut(Status.FIRING);
        final boolean isCharged = checkConsideredCharged();
        
        // TODO: Add check for when a projectile leaves to end before timeout
        if(timedOut && isCharged) {
            ticksCharge(chargedSpeed); // Just make sure that the correct powers are set
            transitionTo(Status.CHARGED);
        }
        //  else if(timedOut && checkConsideredUncharged()) {
        //     // Using uncharge rather than transition to make sure the powers are set.
        //     uncharge();
        // } 
        else if(timedOut && !isCharged) {
            // We want to ensure that the shooter is charged because we are likely to fire again.
            ticksCharge(chargedSpeed); // Just make sure that the correct powers are set
        }
    }
    
    /** 
     * Keeps the shooter and the feeders removing any projectiles that were 
     * misplaced. This ends after a specified amount of time. The shooter
     * can go to the following afterwards:
     * 
     *   - `CHARGED`, if the shooter is charged;
     *   - `UNCHARGING`, if the shooter is not already charged
     * 
     * @param telemetry Unused. 
     */
    protected void periodicAborting(Telemetry telemetry) {
        final boolean timedOut = isTimedOut(Status.ABORTING);
        final boolean isCharged = checkConsideredCharged();
        // final boolean isReloaded = checkIsReloaded();
        
        // TODO: Add check for when a projectile leaves to end before timeout
        if(timedOut && isCharged) {
            ticksCharge(chargedSpeed); // Just make sure that the correct powers are set
            transitionTo(Status.CHARGED);
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

        telemetry.addData("flywheel velocity", flywheels.getVelocity());
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
        // Starting lifetime if necessary
        if(isReadyToStartLifetime) {
            lifetime.startTime();
            isReadyToStartLifetime = false;
        }

        // Keeping record of time
        final double currentTime = lifetime.seconds();
        deltaTime = currentTime - lastTime; 
        lastTime = currentTime;
        
        // Updating all timers
        for(final Object foundLabel : timers.keySet()) {
            timers.put(foundLabel, timers.get(foundLabel) - deltaTime);
        }

        if(telemetry != null && !SUPPRESS_TELEMTERY) {
            telemetry.addLine(padHeader("FlywheelTubeShooter"));
            telemetry.addLine();
            telemetry.addData("Status", getStatus());
            telemetry.addData("    Reload", reloadingState);
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

            case CHARGED:
                periodicCharged(telemetry);
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
            flywheels.setVelocity(targetFlyWheelPower);
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