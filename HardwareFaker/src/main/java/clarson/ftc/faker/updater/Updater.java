package clarson.ftc.faker.updater;

import static com.qualcomm.hardware.lynx.LynxModule.BulkData;

import java.util.Collection;
import java.util.Iterator;

public interface Updater {
    /**
     * Stores the length of delay for automatic updates from the source of the 
     * update. All times are in seconds.
     */
    public enum UpdateDelaySource {
        /** DcMotor subclass or controller called `updateAll()` */   
        MOTOR(0.0025),

        /** Servo/CRServo subclass or controller called `updateAll()` */   
        SERVO(0.0025),

        /** Digital Device or controller called `updateAll()` */ 
        DIGITAL(0.0025),

        /** Analog Device or controller called `updateAll()` */  
        ANALOG(0.0025),

        /** 
         * I2C Device or controller called `updateAll()`. I2C devices ordinarily 
         * take multiple LynxMessages to read, causing a longer delay. 
         */     
        I2C(0.007);

        public double length; // Seconds

        private UpdateDelaySource(double lengthSec) {
            this.length = lengthSec;
        }
    }

    /**
     * Determines whether the given updateable has been registered. If the given
     * updateable has been unregistered after its last registration, this
     * method will return false.
     * 
     * @param updateable What to check for.
     * @return Whether the Updateable was registered and hasn't been 
     * unregistered since
     */
    public boolean hasRegistered(Updateable updateable);

    /**
     * Registers the given Updateable so that it can be updated by this `Updater`.
     * 
     * Circular references are handled on a case-by-case basis, depending on the
     * specific implementation. 
     * 
     * @param newUpdateable What to register. A weak reference will be created.
     * @return Whether the Updateable was now added. False if it was added 
     * previously without being unregistered.
     */
    public boolean register(Updateable newUpdateable);

    /**
     * Registers the given Updateable so that it can be updated by this `Updater`.
     * The Updateable then remembers the given Updater using its `remember` 
     * method.
     * 
     * Circular references are handled on a case-by-case basis, depending on the
     * specific implementation. 
     * 
     * @param newUpdateable What to register. A weak reference will be created.
     * This Updateable will also remember the Updater it was added to.
     * @return Whether the Updateable was now added. False if it was added 
     * previously without being unregistered.
     */
    public boolean register(TwoWayUpdateable newUpdateable);

    /**
     * Removes an updateable. After this is method, `hasRegistered()` for the 
     * given Updateable is guaranteed to return false, until it is registered
     * again, of course.
     * 
     * @param oldUpdateable What to find and unregister.
     * @return Whether the Updateable was now added. False if it was 
     * not registered (i.e. `hasRegistered()` returned false).
     */
    public boolean unregister(Updateable oldUpdateable);

    /**
     * Removes an updateable. After this is method, `hasRegistered()` for the 
     * given Updateable is guaranteed to return false, until it is registered
     * again, of course. The Updateable will forget that it was registered to
     * this Updater.
     * 
     * @param oldUpdateable What to find and unregister. Forgets this Updater
     * @return Whether the Updateable was now added. False if it was 
     * not registered (i.e. `hasRegistered()` returned false).
     */
    public boolean unregister(TwoWayUpdateable oldUpdateable);

    /**
     * Marks all Updateables as unable or able to be updated. True allows the 
     * Updateables to be updated, and false disallows it.
     */
    public void setUpdatingEnabledAll(boolean newUpdatingEnabled);
    
    /**
     * Saves for each Updateable whether it has updating enabled or not. 
     * This method can be reversed by calling `applyAndForgetEnablingStatus()`.
     * 
     * NOTE: Calling this method twice may override the previous remembered 
     * enabling statuses, although this is implementation dependent.
     * 
     * @see `applyAndForgetEnablingStatus()`
     */
    public void rememberEnablingStatus();
    
    /**
     * Reverts any changes to the registered Updateables' enabling statuses. If 
     * no enabling status is remembered, this returns false.
     * 
     * @return Whether a previous enabling status was remembered.
     * @see `rememberEnablingStatus()`
     */
    public boolean applyAndForgetEnablingStatus();

    /**
     * Updates all Updateables registered with this updater. Management of bulk 
     * caching is not done with this method.
     * 
     * Any registered Updaters will not have their `updateAll()` method called,
     * unless it is called in its own `update()` method. This is, however, 
     * **not recommended** as registered updateables may call updateAll() on
     * the Updater, causing an unintended double update.
     * 
     * @param deltaSec How much time has elapsed since last call, in seconds.
     * @see #updateAll(UpdateDelaySource delay)
     */
    public void updateAll(double deltaSec);

    /**
     * Updates all Updateables registered with this updater using the specified
     * source of a delay length. Management of bulk caching is not done with 
     * this method.
     * 
     * Any registered Updaters will not have their `updateAll()` method called,
     * unless it is called in its own `update()` method. This is, however, 
     * **not recommended** as registered updateables may call updateAll() on
     * the Updater, causing an unintended double update.
     * 
     * @param delay Source of the delay length from who called this method.
     * @see #updateAll(double deltaSec)
     */
    default public void updateAll(UpdateDelaySource delay) {
        updateAll(delay.length);
        // return updateAll(delay.length);
    }

    /**
     * Updates each Updateables registered in the given updaters no more than 
     * once. This prevents accidentally updating any Updateable twice or more by
     * having the same Updateable registered in two Updaters, and calling 
     * `updateAll()` on both. 
     * 
     * Updateable that have disabled updating will not be updated. Furthermore,
     * disabling/enabling is perserved by this method; a disabled Updateable will 
     * still be disabled, and an enabled one will stay enabled.
     * 
     * @param updaters Source of all the Updateables to update. 
     * @param deltaSec How long each update is to be simulated.
     */
    public static <E extends Updater> void updateAllOnce(Collection<E> updaters, double deltaSec) {
        final Updater[] reversedUpdaters = new Updater[updaters.size()];
        final Iterator<E> iterator = updaters.iterator();
        for(int i = 0; i < updaters.size(); i++) {
            final Updater updater = iterator.next();
            updater.rememberEnablingStatus();
            updater.updateAll(deltaSec);
            updater.setUpdatingEnabledAll(false);
            reversedUpdaters[reversedUpdaters.length - i - 1] = updater;
        }

        // Renabling to allow for Updateables to be updated later
        // We go through backwards because later updaters will remember the status
        // affected by the previous "updater.disableAll()". Therefore, we apply the early 
        // Updaters last as those have the most accurate information. 
        for(int i = 0; i < reversedUpdaters.length; i++) {
            reversedUpdaters[i].applyAndForgetEnablingStatus();
        }
    } 

    /**
     * Updates each Updateables registered in the given updaters no more than 
     * once. This prevents accidentally updating any Updateable twice or more by
     * having the same Updateable registered in two Updaters, and calling 
     * `updateAll()` on both. 
     * 
     * Updateable that have disabled updating will not be updated. Furthermore,
     * disabling/enabling is perserved by this method; a disabled Updateable will 
     * still be disabled, and an enabled one will stay enabled.
     * 
     * @param updaters Source of all the Updateables to update. 
     * @param delay Source of the delay length from who called this method.
     */
    public static <E extends Updater> void updateAllOnce(Collection<E> updaters, UpdateDelaySource delay) {
        updateAllOnce(updaters, delay.length);
    }
} 