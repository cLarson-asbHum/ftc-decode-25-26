package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.Util;

public class BlockerSubsystem implements BinaryStateSubsystem {
    public static interface Positions {
        double getOpen();

        double getClosed();
    }

    public static enum PositionPresets implements Positions {
        LEFT(0.595, 0.575),

        RIGHT(0.525, 0.55);

        private final double open;
        private final double closed;
        private PositionPresets(double closed, double open) {
            this.open = open;
            this.closed = closed;
        }

        @Override
        public double getOpen() {
            return this.open;
        }
        
        @Override
        public double getClosed() {
            return this.closed;
        }
    }

    public static double TOLERANCE = 0.001;

    private final Servo servo;
    private final double closedPos;
    private final double openPos;

    private State state = State.UNKNOWN;

    private double targetPosition = 0;
    private boolean hasSetPosition = false;

    public BlockerSubsystem(Servo servo, double closedPosition, double openPosition) {
        this.closedPos = closedPosition;
        this.openPos = openPosition;
        this.servo = servo;
    }

    public BlockerSubsystem(Servo servo, Positions positions) {
        this(servo, positions.getClosed(), positions.getOpen());
    }

    private boolean setPosition(double position, double tolerance) {
        // If the new target is already close to the last one, we don't set it 
        if(Util.near(position, targetPosition, tolerance)) {
            return false;
        }

        // Updating the position
        hasSetPosition = true;
        targetPosition = position;
        return true;
    }

    @Override
    public State getState() {
        return this.state;
    }
    
    @Override
    public boolean open() {
        if(this.state == State.OPEN) {
            return false;
        }

        // servo.setPosition(openPos);
        setPosition(openPos, TOLERANCE);
        this.state = State.TO_OPEN;
        return true;
    }
    
    @Override
    public boolean close() {
        if(this.state == State.CLOSED) {
            return false;
        }

        // servo.setPosition(closedPos);
        setPosition(closedPos, TOLERANCE);
        this.state = State.TO_CLOSED;
        return true;
    }

    @Override
    public void periodic() {
        switch(this.state) {
            case TO_OPEN:
                // NOTE: The servo doesn't have actual position feedback available
                //       We just assume that we are at position
                state = State.OPEN;
                break;
                
            case TO_CLOSED:
                // NOTE: The servo doesn't have actual position feedback available
                //       We just assume that we are at position
                state = State.CLOSED;
                break;

            case UNKNOWN:
                // Just open the thing
                open();
                break;
        }

        // Updating the position
        if(hasSetPosition) {
            this.servo.setPosition(targetPosition);
            hasSetPosition = false;
        }
    }
}