package org.firstinspires.ftc.teamcode.util;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.Subsystem;

/**
 * Wraps a method that otherwise acts concurrently. For example, 
 * `IntakeSubsytem.pivotToIntake()` normally needs to only be called once, 
 * and the `periodic()` method would have to be called to actually get the intake
 * to the intaking position. This class simplifies the boilerplate for that by 
 * executing the initial action and waiting for the state to be reached.
 * 
 * `periodic()` must be called on the subsystem externally, either directly 
 * by user code or throught the `CommandScheduler.run()` method.
 */
public class WrapConcurrentCommand<E extends Enum> extends CommandBase {
    private final Runnable initialAction;
    private final StateSubsystem<E> subsystem;
    private final E endState;
    
    public WrapConcurrentCommand(StateSubsystem<E> requirement, Runnable initialAction, E endState) {
        this.initialAction = initialAction;
        this.endState = endState;
        this.subsystem = requirement;
        addRequirements((Subsystem) requirement);
    }

    @Override
    public void initialize() {
        initialAction.run();
    }

    @Override
    public boolean isFinished() {
        return subsystem.getState() == endState;
    }
}