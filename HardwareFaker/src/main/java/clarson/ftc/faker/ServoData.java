package clarson.ftc.faker;

import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.CRServoImpl;
import com.qualcomm.robotcore.hardware.ServoImpl;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.ServoConfigurationType;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import clarson.ftc.faker.updater.Rotateable;
import clarson.ftc.faker.updater.Updateable;

import static com.qualcomm.robotcore.hardware.PwmControl.PwmRange;

abstract public class ServoData<T extends HardwareDevice> implements Rotateable, Updateable {
    private boolean isUpdatingEnabled = true;

    // Descriptor fields - These describe unchaning properties of the servo itself. 
    public final T actuator;
    public final double maxRevsPerSec; // Can be negative
    public final PwmRange maxPwm;

    // Basic Fields - These are the most essential parts to making the update logic work.
    public double position = 0; // Revolutions
    public double unaffectedVelocity = 0; // Revolutions / sec
    public double accumulatedVelOffset = 0; // Revolutions / sec. How much is added by addAngularVel.
    public boolean isTargetSet = false; // Servos can be moved until they receive a PWM signal.
    public boolean isEnabled = true;
    public PwmRange range = PwmRange.defaultRange;

    // Compatability Fields -  These are (currently) unused by the update logic
    public ServoConfigurationType servoType;

    public ServoData(double rpm) {
        this(null, rpm);
    }
    
    public ServoData(double rpm, double initialPosition) {
        this(null, rpm, initialPosition);
    }
    
    public ServoData(double rpm, double initialPosition, PwmRange maxPwm) {
        this(null, rpm, initialPosition, maxPwm);
    }

    public ServoData(T actuator, double rpm) {
        this(actuator, rpm, 0);
    } 
    

    public ServoData(T actuator, double rpm, double initialPosition) {
        this(actuator, rpm, initialPosition, PwmRange.defaultRange);
    } 
    
    public ServoData(T actuator, double rpm, double initialPosition, PwmRange maxPwm) {
        // TODO: This should register the servo so that it can be updated and affected
        this.actuator = actuator;
        this.maxRevsPerSec = rpm / 60;
        this.position = initialPosition;
        this.maxPwm = maxPwm;
    }

    public boolean isPositional() {
        return this.actuator instanceof ServoImpl;
    }
    
    public boolean isContinuous() {
        return this.actuator instanceof CRServoImpl;
    }

    public double getAngle(AngleUnit unit) {
        return unit.fromDegrees(position * 360.0);
    }

    public double getVelocity(AngleUnit unit) {
        return unit.fromDegrees(this.getActualVelocity() * 360.0);
    }

    /**
     * Adds the given rotation to the velocity in the forward direction. The 
     * given velocity is in radians/second. This is designed to allow for unit
     * tests to simulate physics, such as the moment on an arm from gravity.
     * 
     * If the servo is in DcMotor.RunMode.RUN_USING_ENCODER or 
     * DcMotor.RunMode.RUN_TO_POSITION, this only clamps the velocity. The new 
     * range is [tickPrime - |maxRevsPerSec|,  tickPrime + |maxRevsPerSec|], where
     * tickPrime is thetaPrime converted to ticks per second. This simulates the 
     * effect of the servo being overpowered and, therefore, not holding 
     * velocity. 
     * 
     * To illustrate the previous point, take for example setting a servo to 
     * power 1.0 sets the velocity to 100 ticks per sec. A load on the servo, 
     * however, is calculated to apply -20 ticks per sec to the servo, so the 
     * velocity of the servo goes to 80 ticks per sec. However, if the servo was 
     * originally set to 0.3 power andthe velocity to 30 ticks per second, the 
     * servo would stay at 30 ticks per second, because the servo can apply more
     * torque to overcome the counter-torque of the load.
     * 
     * The position is not updated until the `update()` is called (which can 
     * happen automatically because of `HardwareFakeUpdater`), but the added
     * angular velocity persists until the next call to setAngularVelOffset().
     * 
     * @param thetaPrime Added angular velocity offset, in radians per second.
     * @return The new velocity, in reovlutions per second.
     */
    public double addAngularVelOffset(double thetaPrime) {
        return setAngularVelOffset(thetaPrime + this.accumulatedVelOffset * (2 * Math.PI));
    }

    /**
     * Adds only the given rotation to the velocity in the forward direction. 
     * The given velocity is in radians/second. This is designed to allow for 
     * unit tests to simulate physics, such as the moment on an arm from gravity.
     * 
     * If the servo is in DcMotor.RunMode.RUN_USING_ENCODER or 
     * DcMotor.RunMode.RUN_TO_POSITION, this only clamps the velocity. The new 
     * range is [tickPrime - |maxRevsPerSec|,  tickPrime + |maxRevsPerSec|], where
     * tickPrime is thetaPrime converted to ticks per second. This simulates the 
     * effect of the servo being overpowered and, therefore, not holding 
     * velocity. 
     * 
     * To illustrate the previous point, take for example setting a servo to 
     * power 1.0 sets the velocity to 100 ticks per sec. A load on the servo, 
     * however, is calculated to apply -20 ticks per sec to the servo, so the 
     * velocity of the servo goes to 80 ticks per sec. However, if the servo was 
     * originally set to 0.3 power andthe velocity to 30 ticks per second, the 
     * servo would stay at 30 ticks per second, because the servo can apply more
     * torque to overcome the counter-torque of the load.
     * 
     * The position is not updated until the `update()` is called (which can 
     * happen automatically because of `HardwareFakeUpdater`), but the added
     * angular velocity persists until the next call to setAngularVelOffset(),
     * which will override this angular velocity offset
     * 
     * @param thetaPrime The new angular velocity offset, in radians per second.
     * @return The new velocity, in revolutions per second.
     */
    public double setAngularVelOffset(double thetaPrime) {
        this.accumulatedVelOffset = thetaPrime / (2 * Math.PI);
        return this.getActualVelocity();
    }

    abstract public double getActualVelocity();
    
    @Override
    public boolean setUpdatingEnabled(boolean newUpdatingEnabled) {
        return this.isUpdatingEnabled != (this.isUpdatingEnabled = newUpdatingEnabled);
    }

    @Override
    public boolean isUpdatingEnabled() {
        return this.isUpdatingEnabled;
    }
}