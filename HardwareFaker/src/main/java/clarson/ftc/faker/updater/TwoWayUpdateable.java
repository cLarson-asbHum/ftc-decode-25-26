package clarson.ftc.faker.updater;


/**
 * Represents an Updateable that remembers the Updaters it has been registered 
 * with. When unregistered, it forgets the given Updater. Both of these operations-
 * remembering and forgetting- are also accessible and able to be used by classes 
 * that are not Updaters.
 */
public interface TwoWayUpdateable extends Updateable {
    /**
     * Keeps a reference of the given Updater with the calling Updateable. Such
     * Updater can then be used by the Updateable until it is forgotten, most
     * commonly by having the `forget()` method called through 
     * `Updater.unregister()`.
     * 
     * NOTE: An Updater is not guaranteed to be remembered until `forget()` is
     * called. For example, if a the Updater is remembered with a weak reference
     * and the Updater is garbage collected, the Updater has been forgotten 
     * without having the `forget()` method called.
     * 
     * If an Updater is attempted to be remembered while still in the Updateable's
     * memory, the request for remembrance is ignored. Hence, duplicates are 
     * ignored.
     * 
     * @param updater An Updater that is to be referenced by the calling Updateable. 
     * Can be strong or weak reference. 
     */
    public void remember(Updater updater);

    /**
     * Removes the given Updater from the calling Updateable's memory. The given 
     * Updater is guaranteed to not be remembered by the Updateableafterwards, 
     * unless `remember()` is called with the same Updater again.
     * 
     * If the given Updater is not remembered by the Updateable for any reason,  
     * nothing will happen. 
     * 
     * Calls to `Updater.unregister()` ***must*** call this method on all 
     * registered Updateables in order to prevent memory leaks.
     * 
     * @param updater What to forget. Is guaranteed to not be remembered any more.
     */
    public void forget(Updater updater);
}