package org.firstinspires.ftc.teamcode.util;

import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;

/**
 * Represents a Subsystem whose primary goal is regulating the uptake and 
 * control of game pieces. The primary motions often change, but the main 
 * operations often include the following: 
 * 
 *  1. Intaking game pieces, and positioning to such;
 *  2. Holding game pieces, and position to do such;
 *  3. Ejecting mistaken game pieces, and positioning to do such. 
 * 
 * All active instances of IntakeSubsystem should have their periodic() method called 
 * continually to enusure position is read and held. This is because all methods that
 * set hardware states (e.g. `pivotToIntake()`, but not `getPositioon()`) are async;
 * that is, they themselves do not block the thread until their action is finished. 
 * They set only the initial condition to perform the action, but the `periodic()` 
 * method is expected to actually do the updating. 
 * 
 * The recommended way to implement correct handling of the async (or, more 
 * accurately, concurrent) setter methods is to write a Finite State Machine for the
 * `periodic()` method. Command versions of all these methods also exist to facilitate
 * when an action is completed, which is most useful in autonomous where actions need 
 * to be synchronized.
 */
public interface IntakeSubsystem extends Subsystem {
    /**
     * Represents a type of position the intake is in. Transition states
     * are provided for better Finite State Machine implementation.
     */
    public static enum Position {
        /**
         * Position is unset or does not fit into the following categories.
         */
        UNKNOWN, 

        /**
         * The intake is transitioning to being able to take game pieces
         */
        TO_INTAKE,

        /**
         * The intake is able to take game pieces
         */
        INTAKE,

        /**
         * The intake is transitioning to being able to safely clasp game pieces
         */
        TO_HOLD,

        /**
         * The intake is in a position where it is safe to clasp game pieces
         */
        HOLD,

        /**
         * The intake is moving to be able to remove any mistaken game pieces
         */
        TO_EJECT,

        /**
         * The intake is able to remove any mistaken game pieces.
         */
        EJECT
    }

    /**
     * Returns the abstract position this intake is in or moving to.
     * To determine whether the position has been reached, check that 
     * `isInTransit()` returns false.
     * 
     * @return Which position the intake is in.
     */
    public Position getPosition();

    /**
     * Wraps a method that otherwise acts concurrently. For example, 
     * `pivotToIntake()` normally needs to only be called once, and the 
     * `periodic()` method would have to be called to actually get the intake
     * to the intaking position. This class simplifies the boilerplate for 
     * that by writing only the initial action and the `Positon` the intake
     * must be in and waiting for the state to be reached.
     * 
     * `periodic()` must be called on the subsystem externally, either directly 
     * by user code or throught the `CommandScheduler.run()` method.
     */
    public static class WrapConcurrentCommand extends CommandBase {
        private final Runnable initialAction;
        private final Position endState;
        private final IntakeSubsystem subsystem;
        
        public WrapConcurrentCommand(IntakeSubsystem requirement, Runnable initialAction, Position endState) {
            this.initialAction = initialAction;
            this.endState = endState;
            this.subsystem = requirement;
            addRequirements(requirement);
        }

        @Override
        public void initialize() {
            initialAction.run();
        }

        @Override
        public boolean isFinished() {
            return subsystem.getPosition() == endState;
        }
    }

    /**
     * Positions this subsystem for intaking. This sets the current 
     * `IntakeSubsystem.Position` to `TO_INTAKE`.
     * 
     * @return Whether the position of the intake changed since last 
     * `Subsystem.run()` call
     */
    public boolean pivotToIntake();

    public default Command pivotToIntakeCommand() {
        return new WrapConcurrentCommand(this, () -> pivotToIntake(), Position.INTAKE);
    }

    /**
     * Engages the intake to acquire a game piece or two. 
     * 
     * @return Whether the power of the intake changed since last 
     * `Subsystem.run()` call
     */
    public boolean intakeGamePieces();
    
    /**
     * Positions this subsystem for holding any acquired game pieces. This sets 
     * the current `IntakeSubsystem.Position` to `TO_HOLD`.
     * 
     * This does not set the intake's power to hold, and so the pieces may be 
     * liable to leaving. To retain the game pieces, call `holdGamePiecse()`   
     * 
     * @return Whether the position of the intake changed since last 
     * `Subsystem.run()` call
     */
    public boolean pivotToHold();

    public default Command pivotToHoldCommand() {
        return new WrapConcurrentCommand(this, () -> pivotToHold(), Position.HOLD);    
    }

    /**
     * Changes the power of the intake mechanism to keep game pieces safe in 
     * possession.
     * 
     * @return Whether the power of the intake changed since last 
     * `Subsystem.run()` call
     */
    public boolean holdGamePieces();

    /**
     * Positions this subsystem for disharging any acquired game pieces. This 
     * sets the current `IntakeSubsystem.Position` to `TO_EJECT`
     * 
     * @return Whether the position of the intake changed since last 
     * `Subsystem.run()` call
     */
    public boolean pivotToEject();

    public default Command pivotToEjectCommand() {
        return new WrapConcurrentCommand(this, () -> pivotToEject(), Position.EJECT);    
    }

    /**
     * Changes the intake power to move unwanted pieces out of the system. 
     * 
     * @return Whether the speed of the intake changed since last 
     * `Subsystem.run()` call
     */
    public boolean ejectGamePieces();
}