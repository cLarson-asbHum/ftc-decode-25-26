package clarson.ftc.faker.updater;

public interface Updateable {
    /**
     * Performs all operations that are time-dependent. For many applications, 
     * this incluis the simulation of movement. If disabled by 
     * `disableUpdating()` (or more accurately, `isUpdatingEnabled()` is false), 
     * this should do nothing and return 0, no matter the input.
     * 
     * @param deltaSec - Elapsed time of this singular call, in seconds. This is
     * most commonly the time since the last call to update.
     * @return Change in an important statistic. For actuators, this returns
     * the change in rotation. Always 0 when disabled.
     */
    public double update(double deltaSec);

    /**
     * Marks this Updateable as unable or able to be updated. True allows the 
     * Updateable to be updated, and false disallows it.
     * 
     * Updateables should **always** be enabled by default.
     * 
     * @return Whether the enabling status changed as a result of this method.
     */
    public boolean setUpdatingEnabled(boolean newUpdatingEnabled);

    /**
     * Returns whether `update()` is able to be called. This can return false if 
     * `disableUpdating()` has been called and `enableUpdating()` has not been 
     * called since. 
     * 
     * Updateables should **always** be enabled by default.
     * 
     * @return Whether `update()` is enabled.
     */
    public boolean isUpdatingEnabled();
}