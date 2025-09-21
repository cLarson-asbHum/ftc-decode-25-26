package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.command.Subsystem;

/**
 * Represents a Subsystem that implements a finite number of states.
 */
public interface StateSubsystem<T extends Enum> extends Subsystem {
    public T getState();
}