package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.command.Command;

import org.firstinspires.ftc.teamcode.util.WrapConcurrentCommand;

/**
 * Represents a Subsystem whose two primary states are represented by on/off.
 * This most closely resembles subsystems, but can be used for other systems.
 * 
 * NOTE: Implementations must manage the `TO_OPEN` and `TO_CLOSE` states in their 
 * run() method so that `TO_OPEN` and `TO_CLOSE` eventually lead to `OPEN` and 
 * `CLOSED` respectively. Because of this, implementations are ***strongly** 
 * recommended to use a Finite State Machine.
 */
public interface BinaryStateSubsystem extends StateSubsystem<BinaryStateSubsystem.State> {
    public static enum State {
        /**
         * The position does not match the following states. Most common before 
         * a subsystem has been opened or closed (i.e. nothing has happened yet) 
         */
        UNKNOWN,

        /**
         * The subsystem is performing it's dedicated command. Think of this as "on"
         */
        CLOSED,

        /**
         * The subsystem is actively attempting to transition to its closed state.
         */
        TO_CLOSED,

        /** 
         * The subsystem is not actively performing work. Think of this as "off"
         */
        OPEN,

        /**
         * The subsystem is actively attempting to transition to its open state
         */
        TO_OPEN;
    }
    
    /**
     * Returns the current state of the subsystem.
     * 
     * @return Current state of the Subsystem. See `State` for more.
     */ 
    public State getState();

    /**
     * Opens the subsystem. This sets the current `SubsystemSubsystem.State` 
     * to `TO_OPEN`.
     * 
     * @return Whether the state of the subsystem was actually changed
     */
    public boolean open();

    public default Command openSubsystemCommand() {
        return WrapConcurrentCommand.wrapUntilNotState(this, this::open, State.TO_OPEN);
    }
    
    /**
     * Closes the subsystem, clasping anything it in's maw. This sets the current
     * `SubsystemSubsystem.State` to `CLOSED`
     * 
     * @return Whether the state of the subsystem was actually changed
     */
    public boolean close();

    public default Command closeSubsystemCommand() {
        return WrapConcurrentCommand.wrapUntilNotState(this, this::close, State.TO_CLOSED);
    }
}