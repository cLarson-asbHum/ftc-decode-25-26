package clarson.ftc.faker;

// import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.Range;

import clarson.ftc.faker.updater.Rotateable;
import clarson.ftc.faker.updater.Updateable;

import java.util.HashMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class MotorData implements Rotateable, Updateable {
    /**
     * Copies all attributes of the given `MotorData` into a new `MotorData` 
     * object whose `actuator` field references the given motor. This 
     * effectively sets the actuator property of the data to the motor. 
     * 
     * @param motor The motor to refer to in the new data
     * @param data - 
     * @return
     */
    public static MotorData copyForMotor(DcMotorImplEx motor, MotorData data) {
        final MotorData result = new MotorData(
            motor, 
            data.maxTicksPerSec / data.ticksPerRev * 60, 
            data.ticksPerRev
        );
        result.copyAvailableProperties(data);
        return result;
    }

    private boolean isUpdatingEnabled = true;

    // Descriptor fields - These describe unchaning properties of the motor itself. 
    public final DcMotorImplEx actuator;
    public final double ticksPerRev;
    public final double maxTicksPerSec; // Can be negative

    // Basic Fields - These are the most essential parts to making the update logic work.
    public double position = 0; // Ticks
    public double unaffectedVelocity = 0; // Ticks / sec
    public double power = 0;
    public double accumulatedVelOffset = 0; // Ticks / sec. How much is added by addAngularVel.
    public DcMotor.RunMode mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER; 
    public DcMotor.ZeroPowerBehavior behavior = DcMotor.ZeroPowerBehavior.BRAKE;

    // RUN_TO_POSITION Fields. 
    public int tolerance = 5; // Ticks above or below the target, inclusive
    public int targetPosition = 0; // Ticks
    public boolean isEnabled = true;
    public boolean isBusy = false;
    public boolean isTargetSet = false;
    private double runToFactor = 1;
    
    // Compatability Fields -  These are (currently) unused by the update logic
    public double currentAlertAmps = 0; // Amperes
    public final HashMap<DcMotor.RunMode, PIDCoefficients> pidCoefficients = new HashMap<>();
    public final HashMap<DcMotor.RunMode, PIDFCoefficients> pidfCoefficients = new HashMap<>();
    public MotorConfigurationType motorType;

    public MotorData(double rpm, double ticksPerRev) {
        this(null, rpm, ticksPerRev);
    }
    
    public MotorData(double rpm, double ticksPerRev, double initialPosition) {
        this(null, rpm, ticksPerRev, initialPosition);
    }

    public MotorData(DcMotorImplEx actuator, double rpm, double ticksPerRev) {
        this(actuator, rpm, ticksPerRev, 0);
    } 
    
    public MotorData(DcMotorImplEx actuator, double rpm, double ticksPerRev, double initialPosition) {
        // TODO: This should register the motor so that it can be updated and affected
        this.actuator = actuator;
        this.ticksPerRev = ticksPerRev;
        this.maxTicksPerSec = ticksPerRev * rpm / 60;
        this.position = initialPosition;

        pidCoefficients.put(DcMotor.RunMode.RUN_USING_ENCODER, new PIDCoefficients());
        pidCoefficients.put(DcMotor.RunMode.RUN_TO_POSITION, new PIDCoefficients());

        pidfCoefficients.put(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients());
        pidfCoefficients.put(DcMotor.RunMode.RUN_TO_POSITION, new PIDFCoefficients());
    } 

    public double getAngle(AngleUnit unit) {
        return unit.fromDegrees(position * 360.0 / ticksPerRev);
    }

    public double getVelocity(AngleUnit unit) {
        return unit.fromDegrees(this.getActualVelocity() * 360.0 / ticksPerRev);
    }

    /**
     * Sets all settable properties on this `MotorData` to the corresponding 
     * value in the given data. This, therefore, copies the given data to this
     * object except for all primitive final properties.
     * 
     * @param source Where to get all values to copy.
     */
    public void copyAvailableProperties(MotorData source) {
        this.position = source.position;
        this.unaffectedVelocity = source.unaffectedVelocity;
        this.power = source.power;
        this.accumulatedVelOffset = source.accumulatedVelOffset;
        this.mode = source.mode;
        this.behavior = source.behavior;

        this.tolerance = source.tolerance;
        this.targetPosition = source.targetPosition;
        this.isEnabled = source.isEnabled;
        this.isBusy = source.isBusy;
        this.isTargetSet = source.isTargetSet;

        this.currentAlertAmps = source.currentAlertAmps;
        this.pidCoefficients.putAll(source.pidCoefficients);
        this.pidfCoefficients.putAll(source.pidfCoefficients);
        this.motorType = source.motorType;
    }

    private double updateRunToVelocity(double vel) {
        // TODO: Turn RUN_TO_POSITION into a PID method instead of this
        // Getting the speed factor
        double reverseFactor  = 1; // Reverse at a lower speed if the target is missed.
        if((this.targetPosition - this.position) / vel < 0 && this.runToFactor == 1) {
            // Reversing the power so wer alsways start in the correct direction
            reverseFactor *= -1;
        } else if((this.targetPosition - this.position) / vel < 0) {
            reverseFactor *= -0.5; // Put it in reverse, Ter! ...and put half the previous speed
            // reverseFactor *= -1; // Put it in reverse, Ter! ...and put half the previous speed
        }

        this.runToFactor *= reverseFactor;

        // Setting the velocity;
        this.unaffectedVelocity = this.power * this.runToFactor * this.maxTicksPerSec;
        return reverseFactor;
    }

    /**
     * Gets whether this motor is attempting to brake. This is meant to be used 
     * by a test method, not by an opmode, to apply less rotation than normal,
     * dependent on this motor's torque (something not stored in this class).
     * 
     * @return Whether the motor is attempting to brake.
     */
    public boolean isBraking() {
        return this.behavior == DcMotor.ZeroPowerBehavior.BRAKE 
            && this.power == 0;
    }

    /**
     * Adds the given rotation to the velocity in the forward direction. The 
     * given velocity is in radians/second. This is designed to allow for unit
     * tests to simulate physics, such as the moment on an arm from gravity.
     * 
     * If the motor is in DcMotor.RunMode.RUN_USING_ENCODER or 
     * DcMotor.RunMode.RUN_TO_POSITION, this only clamps the velocity. The new 
     * range is [tickPrime - |maxTicksPerSec|,  tickPrime + |maxTicksPerSec|], where
     * tickPrime is thetaPrime converted to ticks per second. This simulates the 
     * effect of the motor being overpowered and, therefore, not holding 
     * velocity. 
     * 
     * To illustrate the previous point, take for example setting a motor to 
     * power 1.0 sets the velocity to 100 ticks per sec. A load on the motor, 
     * however, is calculated to apply -20 ticks per sec to the motor, so the 
     * velocity of the motor goes to 80 ticks per sec. However, if the motor was 
     * originally set to 0.3 power andthe velocity to 30 ticks per second, the 
     * motor would stay at 30 ticks per second, because the motor can apply more
     * torque to overcome the counter-torque of the load.
     * 
     * The position is not updated until the `update()` is called (which can 
     * happen automatically because of `HardwareFakeUpdater`), but the added
     * angular velocity persists until the next call to setAngularVelOffset().
     * 
     * @param thetaPrime Added angular velocity offset, in radians per second.
     * @return The new velocity, in ticks per second.
     */
    public double addAngularVelOffset(double thetaPrime) {
        return setAngularVelOffset(thetaPrime + this.accumulatedVelOffset * (2 * Math.PI) / ticksPerRev);
    }

    /**
     * Adds only the given rotation to the velocity in the forward direction. 
     * The given velocity is in radians/second. This is designed to allow for 
     * unit tests to simulate physics, such as the moment on an arm from gravity.
     * 
     * If the motor is in DcMotor.RunMode.RUN_USING_ENCODER or 
     * DcMotor.RunMode.RUN_TO_POSITION, this only clamps the velocity. The new 
     * range is [tickPrime - |maxTicksPerSec|,  tickPrime + |maxTicksPerSec|], where
     * tickPrime is thetaPrime converted to ticks per second. This simulates the 
     * effect of the motor being overpowered and, therefore, not holding 
     * velocity. 
     * 
     * To illustrate the previous point, take for example setting a motor to 
     * power 1.0 sets the velocity to 100 ticks per sec. A load on the motor, 
     * however, is calculated to apply -20 ticks per sec to the motor, so the 
     * velocity of the motor goes to 80 ticks per sec. However, if the motor was 
     * originally set to 0.3 power andthe velocity to 30 ticks per second, the 
     * motor would stay at 30 ticks per second, because the motor can apply more
     * torque to overcome the counter-torque of the load.
     * 
     * The position is not updated until the `update()` is called (which can 
     * happen automatically because of `HardwareFakeUpdater`), but the added
     * angular velocity persists until the next call to setAngularVelOffset(),
     * which will override this angular velocity offset
     * 
     * @param thetaPrime The new angular velocity offset, in radians per second.
     * @return The new velocity, in ticks per second.
     */
    
    public double setAngularVelOffset(double thetaPrime) {
        this.accumulatedVelOffset = thetaPrime / (2 * Math.PI) * this.ticksPerRev;
        return this.getActualVelocity();
    }

    public double getActualVelocity() {
        switch(this.mode) {
            case RUN_USING_ENCODER: 
            case RUN_TO_POSITION:
                // Only if we are enabled do we actually keep velocity; otherwise, we go into the 
                // default block, which causes the motor vel to change.
                if(this.isEnabled) {
                    return Range.clip(
                        this.unaffectedVelocity,
                        -this.maxTicksPerSec + accumulatedVelOffset,
                        this.maxTicksPerSec + accumulatedVelOffset
                    );
                }

                // No break statement. We only break in the if statement above

            default:
                return unaffectedVelocity + accumulatedVelOffset;
        } 
    }

    @Override
    public double update(double deltaSec) {
        if(!this.isUpdatingEnabled()) {
            return 0;
        }

        final double actualVel = getActualVelocity();
        final double delta = actualVel * deltaSec;
        this.position += delta;

        // Ending run to position when within the target
        if(
            this.mode == DcMotor.RunMode.RUN_TO_POSITION
            && Math.abs(this.position - this.targetPosition) <= this.tolerance
        ) {
            this.isBusy = false;
            this.isTargetSet = false;
            this.power = 0;
            this.unaffectedVelocity = 0;
            this.runToFactor = 1;
            this.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER;
        }

        // Updating the velocity if the postion is off
        if(this.mode == DcMotor.RunMode.RUN_TO_POSITION) {
            this.isBusy = true;
            this.updateRunToVelocity(actualVel);
        } 
        return delta;
    }

    @Override
    public boolean setUpdatingEnabled(boolean newUpdatingEnabled) {
        return this.isUpdatingEnabled != (this.isUpdatingEnabled = newUpdatingEnabled);
    }

    @Override
    public boolean isUpdatingEnabled() {
        return this.isUpdatingEnabled;
    }
}