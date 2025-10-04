package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.util.WrapConcurrentCommand;

/**
 * Represents a subsystem capable of firing a projectile. In order to facilitate 
 * shooting, the subsystem's full capabilties are as follows: 
 * 
 *    1. Charging to completly enable shooting, 
 *    2. Uncharging to completly disable shooting,
 *    3. Reloading to prepare for shooting,
 *    4. Firing to score a loaded projectile,
 *    5. Aborting to remove a loaded projectile.
 * 
 * Certain features, such as variable power and angle and multi-fire, are not 
 * directly implementd in this interface, but are supported. When changing power 
 * or angle, the `CHARGING` state should be used to indicate the change. If 
 * multiple projectiles can be fired, `FIRING` should be able to transition to the 
 * `RELOADED_CHARGED` state wihtout another transition state.
 * 
 * All hardware-setting methods are concurrent and do not block the thread until 
 * finished. For example, the `fire()` method does not block the calling thread until
 * the projectile has fully left. Instead, users are expected to handle this 
 * functionality with Finite State Machine (or equivalent) in the `periodic()` method.
 * Command methods (in this case `fireCommand()`) are finished only when the correct 
 * state has been reached, but still require `periodic()` to be called either by the user
 * or by `CommandScheduler.run()` until the command has completed.
 */
public interface ShooterSubsystem extends StateSubsystem<ShooterSubsystem.Status> {
    public static enum Status {
        /**
         * No indication of current state has been made. Most common after initializing 
         * the shooter after having it denergized. Shooters in this state should 
         * initialize or await input to do some default action.
         */
        UNKNOWN, // Transition state

        /**
         * The shooter is currently getting hardware ready to fire. Examples may be
         * pulling a spring or rubber band, spinning flywheels, or resetting a trebuchet's 
         * counterweight. The shooter cannot fire from this state.
         * 
         * NOTE: This state is also used to represent when a shooter is changing 
         * its angle or changing the expected shot distance. As a result, this 
         * status does not guarantee a charged status will come afterwards; for 
         * example, changing the angle of a shooter while, say, flywheels are 
         * motionless does not mean the shooter will be able to fire. 
         */
        CHARGING, // Transition state

        /**
         * The shooter is in the process of disabling firing. The shooter cannot
         * fire from this state.
         */
        UNCHARGING, // Transition state

        /**
         * The shooter is not charged and so is unable to fire. This does not 
         * inidicate whether the shooter is loaded or not.
         */
        UNCHARGED,

        /**
         * The shooter would be able to shoot if it had a projectile ready. 
         * A shooter can dry fire from this state, although this is **not**
         * recommended.
         */
        EMPTY_CHARGED(true), // Allowed to dry fire (for some unfathomable reason)

        /**
         * A projectile is being moved into the shooter. The shooter may not fire at 
         * this time.
         */
        RELOADING, // Transition state

        /**
         * The shooter is completely able to fire. The shot may also be aborted to 
         * remove the projectile by transitioning to the `ABORTING` state.
         */
        RELOADED_CHARGED(true), // Allowed to fire the current projectile(s)

        /**
         * A projectile is in the process of being shot. 
         */
        FIRING, // Transition state

        /**
         * A projectile is being removed without being fired. This may be to use some
         * other projectile, or to remove a mistaken projectile. The shooter is unable
         * to fire at this time.
         */
        ABORTING; // Transition state

        /**
         * Whether the shooter can transition to `FIRING` in this status. 
         * Defaults to false. 
         */
        public final boolean shouldBeAbleToFire;
        private Status(boolean shouldBeAbleToFire) {
            this.shouldBeAbleToFire = shouldBeAbleToFire;
        }

        private Status() {
            this(false);
        }
    } 

    /**
     * Gets the current status of the shooter. The main purpose is to describe 
     * whether the shooter is able to fire or not. 
     * 
     * @return The current status of the shooter.
     */
    public Status getStatus();

    /**
     * Alias for `getStatus()`.
     */
    default public Status getState() {
        return getStatus();
    }

    /**
     * Whether this shooter can transition to the `FIRING` status. The default 
     * uses the current status' default "shouldBeAbleToFire" field.
     * 
     * @return Whether the `FIRING` status is able to be transitioned to.
     */
    default public boolean shouldBeAbleToFire() {
        return this.getStatus().shouldBeAbleToFire;
    }

    /**
     * Charges the shooter to its (default) charged state, going into the `CHARGING` 
     * state. If the shooter is already charged or otherwise unable to charge, this
     * method does nothing and returns false.
     * 
     * A projectile is not loaded by this method. For that, see `reload()`.
     * 
     * @return Whether the shooter transitioned to the `CHARGING` state.
     */
    public boolean charge();

    default public Command chargeCommand() {
        return WrapConcurrentCommand.wrapUntilNotState(this, () -> charge(), Status.CHARGING);
    }
    
    /**
     * Uncharges the shooter, disabling firing altogether by going into the `UNCHARGING`
     * state. If the shooter is already uncharged or otherwise unable to uncharge, this
     * method does nothing and returns false.
     * 
     * @return Whether the shooter transitioned to the `UNCHARGING` state.
     */
    public boolean uncharge();

    default public Command unchargeCommand() {
        return WrapConcurrentCommand.wrapUntilNotState(this, () -> uncharge(), Status.UNCHARGING);
    }

    /**
     * Loads a projectile into the shooter, transitioning to the `RELOADING` state. If the
     * shooter is unable to reload- likely because it is fully loaded already, this method 
     * does nothing and returns false.
     * 
     * This does not guarantee the shooter can fire, as it may be uncharged and/or the 
     * reloading can fail. To charge the shooter, please use `charge()` or `chargeCommand()`
     * 
     * @return Whether the shooter transitioned to the `RELOADING` state.
     */
    public boolean reload();

    default public Command reloadCommand() {
        return WrapConcurrentCommand.wrapUntilNotState(this, () -> reload(), Status.RELOADING);
    }

    /**
     * Fires the projectile, transitioning to the `FIRING` status. If the 
     * shooter is unable to fire (it's not charged, for example), this does 
     * nothing and returns false. 
     * 
     * @return Whether status could be updated to `FIRING`.
     */
    public boolean fire();

    default public Command fireCommand() {
        return WrapConcurrentCommand.wrapUntilNotState(this, () -> fire(), Status.FIRING);
    }

    public boolean abort();

    default public Command abortCommand() {
        return WrapConcurrentCommand.wrapUntilNotState(this, () -> abort(), Status.ABORTING);
    }
}