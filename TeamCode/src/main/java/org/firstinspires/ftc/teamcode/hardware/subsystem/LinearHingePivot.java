package org.firstinspires.ftc.teamcode.hardware.subsystem;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.HashMap;
import java.util.function.DoubleUnaryOperator;

import org.firstinspires.ftc.teamcode.temp.TimeInjectionUtil;
import org.firstinspires.ftc.teamcode.util.Util;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@Configurable
public class LinearHingePivot implements PivotSubsystem {
    public static class PivotConstants {
        public double defaultPosition = 0.67; // This position is... <imitates a scale> eh.
        public double defaultTolerance = 0.02;
    } 

    public static class Timeout {
        public double retryPosition = 1.0;
    }

    public static PivotConstants PIVOT_CONSTANTS = new PivotConstants();

    private Status status = PivotSubsystem.Status.UNKNOWN;

    private final DoubleUnaryOperator radiansToPosition;
    private final DoubleUnaryOperator positionToRadians;

    private final Servo pivot;

    private double lastTolerance = 0;
    private double targetPosition = PIVOT_CONSTANTS.defaultPosition;
    private boolean hasSetTarget = false;

    private Telemetry telemetry = null;
    
    public static Timeout TIMEOUT = new Timeout();
    private HashMap<Object, Double> timers = new HashMap<>();
    private final ElapsedTime lifetime; // NOTE: Use deltaTime for any implementations, not this!!!
    private double lastTime = 0;        // NOTE: Use deltaTime for any implementations, not this!!!
    private double deltaTime = 0;
    private boolean isReadyToStartLifetime = true;

    private LinearHingePivot(Builder builder) {
        this.lifetime = TimeInjectionUtil.getElapsedTime();
        this.pivot = builder.pivot;
        this.radiansToPosition = builder.radiansToPosition;
        this.positionToRadians = builder.positionToRadians;
    }

    public static final class Builder {
        public Servo pivot = null;
        public DoubleUnaryOperator radiansToPosition = DoubleUnaryOperator.identity();
        public DoubleUnaryOperator positionToRadians = DoubleUnaryOperator.identity();

        public Builder(Servo pivot) {
            this.pivot = pivot;
        }
        
        public Builder() {
            // Do nothing.
        }

        public Builder setPivot(Servo pivot) {
            this.pivot = pivot;
            return this;
        }

        public Builder setRadiansToPosition(DoubleUnaryOperator radiansToPosition) {
            this.radiansToPosition = radiansToPosition;
            return this;
        }
        
        public Builder setPositionToRadians(DoubleUnaryOperator positionToRadians) {
            this.positionToRadians = positionToRadians;
            return this;
        }

        public LinearHingePivot build() {
            if(pivot == null) {
                throw new IllegalArgumentException("No servo was set on builder");
            }

            return new LinearHingePivot(this);
        }
    }
    
    public double convertFromPosition(double positon) {
        return this.positionToRadians.applyAsDouble(positon);
    }

    private boolean setServoPosition(double position, double tolerance) {
        if(pivot == null) {
            return false;
        }

        if(Util.near(position, targetPosition, tolerance)) {
            return false;
        }

        lastTolerance = tolerance;
        targetPosition = position;
        hasSetTarget = true;
        return true;
    }

    @Override
    public double getCurrentAngle() {
        return positionToRadians.applyAsDouble(this.pivot.getPosition());
    }
    
    @Override
    public double getTargetAngle() {
        return positionToRadians.applyAsDouble(this.targetPosition);
    }
    
    @Override
    public double getTargetTolerance() {
        return positionToRadians.applyAsDouble(this.lastTolerance);
    }

    @Override
    public Status getStatus() {
        return this.status;
    }

    private boolean transitionTo(Status newStatus) {
        this.status = newStatus;
        return true;
    }
    
    @Override
    public boolean runToAngle(double theta, double tolerance) {
        startTimeout(PivotSubsystem.Status.POSITIONING, TIMEOUT.retryPosition);
        setServoPosition(
            radiansToPosition.applyAsDouble(theta), 
            radiansToPosition.applyAsDouble(tolerance)
        );
        return transitionTo(PivotSubsystem.Status.POSITIONING);
    }

    public boolean runToAngle(double theta) {
        return runToAngle(theta, PIVOT_CONSTANTS.defaultTolerance);
    }

    private boolean retryPosition() {
        final double oldTolerance = lastTolerance;
        setServoPosition(targetPosition, 0.0);
        lastTolerance = oldTolerance;
        startTimeout(PivotSubsystem.Status.POSITIONING, TIMEOUT.retryPosition);
        return transitionTo(PivotSubsystem.Status.POSITIONING);
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

    private void periodicPositioning() {
        // If we are at our target, then exit.
        if(isWithinTarget(lastTolerance)) {
            transitionTo(PivotSubsystem.Status.HOLDING); // Defense, number 74. 10 yard penalty; first down.
            return;
        }

        // Every second that we aren't at position, try again
        if(isTimedOut(PivotSubsystem.Status.POSITIONING)) {
            retryPosition();
            return;
        }

        // Otherwise, we just do nothing.
    }

    private void periodicUnknown() {
        retryPosition();
    }

    public void setTelemetry(Telemetry telemetry) {
        this.telemetry = telemetry;
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

        // Handling the current state
        switch(status) {
            case POSITIONING:
                periodicPositioning();
                break;

            case UNKNOWN:
                periodicUnknown();
                break;
                
            default:
                // Do nothing
                break;
        }

        // Updating the position
        if(hasSetTarget) {
            this.pivot.setPosition(targetPosition);
        }
    }
}
