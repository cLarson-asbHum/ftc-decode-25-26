package clarson.ftc.faker;

import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.ServoConfigurationType;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import static com.qualcomm.robotcore.hardware.PwmControl.PwmRange;

public class ContinuousServoData extends ServoData<CRServoImplEx> {
    /**
     * Copies all attributes of the given `ServoData` into a new `ServoData` 
     * object whose `actuator` field references the given servo. This 
     * effectively sets the actuator property of the data to the servo. 
     * 
     * Neither the ServoData source nor the servo is modified as a result of 
     * this method.
     * 
     * @param servo The servo to refer to in the new data
     * @param source Where the values for the properties are to come from. 
     * @return A new ServoData reference, with the same fields except for 
     * `actuator` which is now the given servo argument. 
     */
    public static ContinuousServoData copyForServo(CRServoImplEx servo, ContinuousServoData source) {
        final ContinuousServoData result = new ContinuousServoData(
            servo, 
            source.maxRevsPerSec * 60, 
            source.position, 
            source.maxPwm
        );
        result.copyAvailableProperties(source);
        return result;
    }
    
    public double power = 0; 

    public ContinuousServoData(double rpm) {
        this(null, rpm);
    }
    
    public ContinuousServoData(double rpm, double initialPosition) {
        this(null, rpm, initialPosition);
    }
    
    public ContinuousServoData(double rpm, double initialPosition, PwmRange maxPwm) {
        this(null, rpm, initialPosition, maxPwm);
    }

    public ContinuousServoData(CRServoImplEx actuator, double rpm) {
        this(actuator, rpm, 0);
    } 
    

    public ContinuousServoData(CRServoImplEx actuator, double rpm, double initialPosition) {
        this(actuator, rpm, initialPosition, PwmRange.defaultRange);
    } 
    
    public ContinuousServoData(CRServoImplEx actuator, double rpm, double initialPosition, PwmRange maxPwm) {
        super(actuator, rpm, initialPosition, maxPwm);
    }

    /**
     * Gets whether this servo is attempting to brake. This is meant to be used 
     * by a test method, not by an opmode, to apply less rotation than normal,
     * dependent on this servo's torque (something not stored in this class).
     * 
     * @return Whether the servo is attempting to brake.
     */
    public boolean isBraking() {
        return this.power == 0;
    }

    /**
     * Sets all settable properties on this `ServoData` to the corresponding 
     * value in the given data. This, therefore, copies the given data to this
     * object except for all primitive final properties.
     * 
     * @param source Where to get all values to copy.
     */
    public void copyAvailableProperties(ContinuousServoData source)  {
        this.position = source.position;
        this.unaffectedVelocity = source.unaffectedVelocity;
        this.accumulatedVelOffset = source.accumulatedVelOffset;
        this.isTargetSet = source.isTargetSet;
        this.isEnabled = source.isEnabled;
        this.servoType = source.servoType;
        this.power = source.power;
    }

    @Override
    public boolean isPositional() {
        return false;
    }

    @Override
    public boolean isContinuous() {
        return true;
    }

    @Override
    public double getActualVelocity() {
        if(this.isTargetSet && this.isEnabled) {
            return Range.clip(
                this.unaffectedVelocity,
                -this.maxRevsPerSec + this.accumulatedVelOffset,
                this.maxRevsPerSec + this.accumulatedVelOffset
            );
        }

        return this.unaffectedVelocity + this.accumulatedVelOffset;
    }
    
    
    // TODO: Update the `update()` method JavaDoc
    /** 
     * Performs all calculations necessary for correct operation of this fake. 
     * This should be called by the test itself to simulate real life for an 
     * opmode, not by opmodes themselves. 
     * 
     * @param deltaSec Elapsed number of seconds since the last call to update()
     * @return The changed number of reovlutions.
     */
    @Override
    public double update(double deltaSec) {
        if(!this.isUpdatingEnabled()) {
            return 0;
        }

        final double actualVel = getActualVelocity();
        final double delta = actualVel * deltaSec;
        this.position += delta;
        return delta;
    }
}
