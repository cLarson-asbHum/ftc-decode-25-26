package org.firstinspires.ftc.teamcode.util;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.Subsystem;

import java.util.function.BooleanSupplier;

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
    /**
     * Creates a new WrapConcurrentCommand that is finished once the state is *not* 
     * the given state.
     * 
     * @param requirement The subsystem used by the command
     * @param initialAction What is done at the start of the action only.
     * @param continutationState The state the subsytem will not be finished in. 
     * If the state is ever different, the command will be finished.
     */
    public static <T extends Enum> WrapConcurrentCommand<T> wrapUntilNotState(
        StateSubsystem<T> requirement,
        Runnable initialAction,
        T continuationState
    ) {
        return new WrapConcurrentCommand<T>(
            requirement, 
            initialAction, 
            () -> requirement.getState() != continuationState
        );
    }
    
    private final Runnable initialAction;
    private final StateSubsystem<E> subsystem;
    private final BooleanSupplier endCondition;
    
    public WrapConcurrentCommand(StateSubsystem<E> requirement, Runnable initialAction, E endState) {
        this(requirement, initialAction, () -> requirement.getState() == endState);
    }

    public WrapConcurrentCommand(
        StateSubsystem<E> requirement, 
        Runnable initialAction, 
        BooleanSupplier endCondition
    ) {
        this.initialAction = initialAction;
        this.endCondition = endCondition;
        this.subsystem = requirement;
        addRequirements((Subsystem) requirement);
    }

    @Override
    public void initialize() {
        initialAction.run();
    }

    @Override
    public boolean isFinished() {
        return endCondition.getAsBoolean();
    }
}