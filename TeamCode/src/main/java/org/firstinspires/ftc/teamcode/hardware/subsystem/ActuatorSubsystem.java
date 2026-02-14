package org.firstinspires.ftc.teamcode.hardware.subsystem;

import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.util.Util;

public class ActuatorSubsystem implements Subsystem {
    public static class Speeds {
        public double extendLeft = 1.0;
        public double extendRight = 1.0;

        public double holdLeft = 0.05;
        public double holdRight = 0.05;
        
        public double lowerLeft = -0.5 * extendLeft;
        public double lowerRight = -0.5 * extendRight;

        public double tolerance = 0.001;
    }

    private final Speeds SPEED = new Speeds();
    
    private final CRServo left;
    private double targetSpeedLeft = 0;
    private boolean hasSetSpeedLeft = false;
    
    private final CRServo right;
    private double targetSpeedRight = 0;
    private boolean hasSetSpeedRight = false;

    public ActuatorSubsystem(CRServo left, CRServo right) {
        this.left = left;
        this.right = right;
    }

    private boolean setLeftSpeed(double speed, double tolerance) {
        // If the new target is already close to the last one, we don't set it 
        if(Util.near(speed, targetSpeedLeft, tolerance)) {
            return false;
        }

        // Updating the speed
        hasSetSpeedLeft = true;
        targetSpeedLeft = speed;
        return true;
    }
    
    private boolean setRightSpeed(double speed, double tolerance) {
        // If the new target is already close to the last one, we don't set it 
        if(Util.near(speed, targetSpeedRight, tolerance)) {
            return false;
        }

        // Updating the speed
        hasSetSpeedRight = true;
        targetSpeedRight = speed;
        return true;
    }
    
    public boolean extend() {
        final boolean didSetLeft = setLeftSpeed(SPEED.extendLeft, SPEED.tolerance);
        final boolean didSetRight = setRightSpeed(SPEED.extendRight, SPEED.tolerance);
        return didSetLeft || didSetRight;
    }
    
    public boolean hold() {
        final boolean didSetLeft = setLeftSpeed(SPEED.holdLeft, SPEED.tolerance);
        final boolean didSetRight = setRightSpeed(SPEED.holdRight, SPEED.tolerance);
        return didSetLeft || didSetRight;
    }
    
    public boolean lower() {
        final boolean didSetLeft = setLeftSpeed(SPEED.lowerLeft, SPEED.tolerance);
        final boolean didSetRight = setRightSpeed(SPEED.lowerRight, SPEED.tolerance);
        return didSetLeft || didSetRight;
    }

    @Override
    public void periodic() {
        // Updating the speed
        if(hasSetSpeedLeft) {
            this.left.setPower(targetSpeedLeft);
            hasSetSpeedLeft = false;
        }
        
        if(hasSetSpeedRight) {
            this.right.setPower(targetSpeedRight);
            hasSetSpeedRight = false;
        }

    }
}