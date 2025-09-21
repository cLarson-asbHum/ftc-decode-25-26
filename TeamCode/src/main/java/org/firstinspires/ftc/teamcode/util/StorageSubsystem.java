package org.firstinspires.ftc.teamcode.util;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.Subsystem;

/**
 * Represents a subsystem capable of storing and retrieving several game pieces.
 * A storage subsystem's full capabilities are as follows:
 * 
 *    1. Detect the number of game pieces,
 *    2. Enable or Disable the ability to intake or remove pieces,
 *    3. Release/pop the most available game piece,
 *    4. Release a specific game piece (optional).
 *    5. Eject a given game piece from the robot (optional)
 */
public interface StorageSubsystem extends StateSubsystem<StorageSubsystem.Status> {
    public static enum Status {
        /**
         * The storage cannot be known at this time. This is most common before 
         * initialization. Storages in this state should use this to initialize
         * or wait for a user input to initialize, after which the user input is
         * processed. 
         */
        UNKNOWN,

        /**
         * The storage is able to take incoming game pieces and store them
         */
        ENABLED,

        /**
         * The storage is unable to take incoming game pieces. A storage may
         * or may not be able to release; that is up to the implementor.
         */
        DISABLED,

        /**
         * A stored game piece is currently being transfered from storage. The 
         * game piece is able to be used by the rest of the robot afterwards.
         */
        RELEASING,
        
        /**
         * A stored game piece is currently being removed from the robot entirely.
         * This is to allow for storages to remove any mistaken pieces. The game 
         * piece will not be useable afterwards.
         */
        EJECTING;
    }

    /**
     * Gets the current `Status` that this storage is in. The main purpose is to 
     * control releasing and ejecting using a Finite State Machine.
     * 
     * @return The status that this storage is in.
     */
    public Status getStatus();

    /**
     * Alias for `getStatus()`
     */
    default public Status getState() {
        return this.getStatus();
    }

    /**
     * Gets the maximum number of game elements that this storage can hold. If the maximum
     * number of elements is unkwown, this should be negative. If the maximum varies 
     * depending on external states (currently held pieces, other mechanisms' states), then
     * this should return the number most accurate at the time of calling, or a negative 
     * number if unknown.  
     * 
     * This method is in contrast to `getUnoccupiedCapacity()`, which returns the number of 
     * pieces the storage has yet to fill. This method should always equal 
     * `getAmmo() + getUnoccupiedCapacity()`
     * 
     * @return The max number of game pieces this could hold. 
     */
    public int getMaxCapacity();
    
    /**
     * Gets the number of stored game pieces. If the number of pieces is unknown, the 
     * return value is a negative number.
     * 
     * @return The number of currently stored game pieces. Is negative if unknown
     */
    public int getAmmo();

    /**
     * Returns the number of game pieces this storage can intake *on top of* its 
     * current ammo. In other words, this is equivalent to the max capacity minus 
     * the current ammo. 
     * 
     * NOTE: Implementations should be cautious if using the capacity and the ammo
     * to deduce the current remaining capacity. If both are negative, 
     * `getMaxCapacity() - getAmmo()` may be positive despite the fact that the 
     * capacity should also be unknown. The default behvior handles this correctly,
     * being negative if either or both are negative. This does not apply if 
     * the capacity can be determined externally (i.e. using sensors). 
     * 
     * @return The remaining number of game pieces the storage can hold. Negative 
     * if unknown.
     */
    default public int getUnoccupiedCapacity() {
        if(getMaxCapacity() < 0 || getAmmo() < 0) {
            return -1;
        }

        return getMaxCapacity() - getAmmo();
    }

    /**
     * Allows game pieces to enter the storage, going into the `ENABLED` state 
     * If the storage must actively capture game pieces and move them into 
     * storage (in contrast to a  storage that passively waits for, e.g., balls 
     * to roll in), then this method actively attempts to bring in the game piece. 
     * 
     * If the storage is already enabled or otherwise unable to be enabled, this
     * method returns false and does nothing.
     * 
     * @return Whether the state was successfully changed to `ENABLED`.
     */
    public boolean enableStorage();

    default public Command enableStorageCommand() {
        return WrapConcurrentCommand.wrapUntilNotState(this, () -> enableStorage(), Status.DISABLED);
    }

    /**
     * Prevents game pieces from entering the storage, going into the `DISABLED`
     * state. If the storage is already disabled or otherwise unable to be disabled,
     * this method does nothing and returns false.
     * 
     * @return Whether the state was successfully changed to `DISABLED`.
     */
    public boolean disableStorage();

    default public Command disableStorageCommand() {
        return WrapConcurrentCommand.wrapUntilNotState(this, () -> disableStorage(), Status.ENABLED);
    }

    /**
     * Releases the most available game piece, going into the `RELEASING` state.
     * What constitutes the "most available piece" changes from intake to intake.
     * By default, this method releases the most recently stored game piece.
     * 
     * If the storage is empty, already releasing, or ejecting, this method does 
     * nothing and returns false.
     * 
     * @return Whether the state was changed to `RELEASING`.
     */
    default public boolean pop() {
        return release(0);
    }

    default public Command popCommand() {
        return WrapConcurrentCommand.wrapUntilNotState(this, () -> pop(), Status.RELEASING);
    }

    /**
     * Releases a specific game piece from the storage, going to the `RELEASING` 
     * state. If the storage is already releasing, ejecting, or otherewise unable
     * to release a game piece, this method does nothing and returns false. Likewise,
     * if the given index is out of bounds (less than 0 or greater than/equal to 
     * `getAmmo()`), this does nothing and returns false.
     * 
     * The index is intended to be such that 0 is the ***most recent*** element 
     * stored, and `getAmmo() - 1` is the ***least*** recently stored game piece. 
     * If a "last" and "first" cannot be determined (such as in a moving and 
     * cyclic storage), then the indices should refer to specific positions. If 
     * indices do not apply to the storage (a heap of pieces, for example), this
     * does the same as `pop()`.
     *  
     * @param index What game piece to grab. 0 is the most recent/easiest to release,
     * and the largest index (`getAmmo() - 1`) is the least recent/hardest to release
     * @return Whether the storage was changed to the `RELEASING` state.
     */
    public boolean release(int index);

    default public Command releaseCommand(int index) {
        return WrapConcurrentCommand.wrapUntilNotState(this, () -> release(index), Status.RELEASING);
    }
    
    /**
     * Ejects the most available game piece, going into the `EJECTING` state.
     * What constitutes the "most available piece" changes from intake to intake.
     * By default, this method ejects the most recently stored game piece.
     * 
     * If the storage is empty, already ejecting, or releasing, this method does 
     * nothing and returns false.
     * 
     * @return Whether the state was changed to `EJECTING`.
     */
    default public boolean popEject() {
        return eject(0);
    }

    default public Command popEjectCommand() {
        return WrapConcurrentCommand.wrapUntilNotState(this, () -> popEject(), Status.EJECTING);
    }

    /**
     * Ejects a specific game piece from the storage, going to the `EJECTING` 
     * state. If the storage is already ejecting, releasing, or otherewise unable
     * to eject a game piece, this method does nothing and returns false. Likewise,
     * if the given index is out of bounds (less than 0 or greater than/equal to 
     * `getAmmo()`), this does nothing and returns false.
     * 
     * The index is intended to be such that 0 is the ***most recent*** element 
     * stored, and `getAmmo() - 1` is the ***least*** recently stored game piece. 
     * If a "last" and "first" cannot be determined (such as in a moving and 
     * cyclic storage), then the indices should refer to specific positions. If 
     * indices do not apply to the storage (a heap of pieces, for example), this
     * does the same as `popEject()`. 
     *  
     * @param index What game piece to grab. 0 is the most recent/easiest to eject,
     * and the largest index (`getAmmo() - 1`) is the least recent/hardest to eject
     * @return Whether the storage was changed to the `EJECTING` state.
     */
    public boolean eject(int index);

    default public Command ejectCommand(int index) {
        return WrapConcurrentCommand.wrapUntilNotState(this, () -> eject(index), Status.EJECTING);
    }

}