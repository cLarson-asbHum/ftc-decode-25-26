package clarson.ftc.faker;

import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.ServoConfigurationType;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import static com.qualcomm.robotcore.hardware.PwmControl.PwmRange;

public class PositionalServoData extends ServoData<ServoImplEx> {
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
    public static PositionalServoData copyForServo(ServoImplEx servo, PositionalServoData source) {
        final PositionalServoData result = new PositionalServoData(
            servo, 
            source.maxRevsPerSec * 60, 
            source.maxPosition, 
            0, 
            source.maxPwm
        );
        result.copyAvailableProperties(source);
        return result;
    }

    public final double maxPosition; // Revolutions
    public double tolerance = 5.0 / 360.0; // Revolutions above or below the target, inclusive
    public double targetPosition = 0; // Revolutions. On interval [0, maxPosition]
    private double runToFactor = 1;
    private boolean wasPreviouslyInRange = false;

    public PositionalServoData(double rpm, double maxRevolutions) {
        this(null, rpm, maxRevolutions);
    }
    
    public PositionalServoData(double rpm, double maxRevolutions, double initialPosition) {
        this(null, rpm, maxRevolutions, initialPosition);
    }
    
    public PositionalServoData(
        double rpm, 
        double maxRevolutions, 
        double initialPosition, 
        PwmRange maxPwm
    ) {
        this(null, rpm, maxRevolutions, initialPosition, maxPwm);
    }

    public PositionalServoData(ServoImplEx actuator, double rpm, double maxRevolutions) {
        this(actuator, rpm, maxRevolutions, 0);
    } 
    

    public PositionalServoData(
        ServoImplEx actuator, 
        double rpm, 
        double maxRevolutions, 
        double initialPosition
    ) {
        this(actuator, rpm, maxRevolutions, initialPosition, PwmRange.defaultRange);
    } 
    
    public PositionalServoData(
        ServoImplEx actuator, 
        double rpm, 
        double maxRevolutions, 
        double initialPosition, 
        PwmRange maxPwm
    ) {
        super(actuator, rpm, initialPosition, maxPwm);
        this.maxPosition = maxRevolutions;
    }

    /**
     * Sets all settable properties on this `ServoData` to the corresponding 
     * value in the given data. This, therefore, copies the given data to this
     * object except for all primitive final properties.
     * 
     * @param source Where to get all values to copy.
     */
    public void copyAvailableProperties(PositionalServoData source) {
        this.position = source.position;
        this.unaffectedVelocity = source.unaffectedVelocity;
        this.accumulatedVelOffset = source.accumulatedVelOffset;
        this.isTargetSet = source.isTargetSet;
        this.isEnabled = source.isEnabled;
        this.servoType = source.servoType;

        this.tolerance = source.tolerance;
        this.targetPosition = source.targetPosition;
        this.runToFactor = source.runToFactor;
    }

    @Override
    public double getActualVelocity() {
        if(this.isEnabled && this.isTargetSet) {
            return Range.clip(
                this.unaffectedVelocity,
                -this.maxRevsPerSec + accumulatedVelOffset,
                this.maxRevsPerSec + accumulatedVelOffset
            );
        }

        return unaffectedVelocity + accumulatedVelOffset;
    }

    private double updateRunToVelocity(double vel) {
        // TODO: Turn servo positioning into a PID method instead of this
        // Getting the speed factor
        double reverseFactor = 1; // Reverse at a lower speed if the target is missed.
        if((this.targetPosition - this.position) / vel < 0 && this.runToFactor == 1) {
            // Reversing the power so we always start in the correct direction
            reverseFactor *= -1;
        } else if((this.targetPosition - this.position) / vel < 0) {
            reverseFactor *= -0.5; // Put it in reverse, Ter! ...and put half the previous speed
            // reverseFactor *= -1; // Put it in reverse, Ter! ...and put half the previous speed
        }

        this.runToFactor *= reverseFactor;

        // Setting the velocity;
        this.unaffectedVelocity = this.runToFactor * this.maxRevsPerSec;
        return reverseFactor;
    }

    @Override
    public double update(double deltaSec) {
        if(!this.isUpdatingEnabled()) {
            return 0;
        }

        final double startPosition = position;
        final double actualVel = getActualVelocity();
        final double delta = actualVel * deltaSec;
        this.position = Range.clip(startPosition + delta, 0, maxPosition);

        // Reseting the run to factor to move quickly to position. 
        if(
            this.isEnabled 
            && this.isTargetSet
            && wasPreviouslyInRange 
            && Math.abs(this.position - this.targetPosition) > this.tolerance
        ) {
            // this.isTargetSet = false;
            this.runToFactor = 1;
        }

        // Stopping the servo if we get in range
        if(
            this.isEnabled
            && this.isTargetSet
            && Math.abs(this.position - this.targetPosition) <= this.tolerance
        ) {
            this.unaffectedVelocity = 0;
            this.runToFactor = 1; // Reset for next time.
        }

        // Updating the velocity if the postion is off
        if(
            this.isEnabled 
            && this.isTargetSet
            && Math.abs(this.position - this.targetPosition) >= this.tolerance
        ) {
            wasPreviouslyInRange = Math.abs(this.position - this.targetPosition) <= this.tolerance;
            this.updateRunToVelocity(actualVel);
        }
        return this.position - startPosition;
    }
}