package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.ballistics.BallisticArcSelection;

public final class OpModeData {
    /**
     * Tells the current opmode whether the last opmode was in competition mode.
     * The expected use case is to hide debug information and functionality, 
     * notably telemetry that takes extra time.
     * 
     * Teleops are expected to respect this value, changing it only upon user 
     * input. Autos are allowed (and recommended) to force change this value, 
     * as the user has time to change this.
     */
    public static boolean inCompetitonMode = false;

    /**
     * Whether the current opmode should be played as blue or red. The expected use
     * case of this is to mirror key poses. 
     * 
     * Teleops are expected to respect this value, changing it only upon user 
     * input. Autos are allowed (and recommended) to force change this value, 
     * as the user has time to change this.
     */
    public static boolean isRed = false;

    /**
     * The last set of ballistic arcs that were read from the resources folder. The
     * expected use case of this is to save time in initialization (particularlly in 
     * competiton mode).
     * 
     * While no stipulations are made as to what opmodes can change this value, it
     * is recommended all opmodes use the last selection unless the user rereads 
     * them (perhaps by using a button press in initialization?).
     */
    public static BallisticArcSelection selection = null;

    /**
     * The last position the robot was in. The exepected use case is to provide
     * teleops a way to know where they are at the end of auto.
     * 
     * All autos and CompetitonTeleop are expected to update this value, including
     * ones that do not use PedroPathing. Test teleops should not update this value
     */
    public static Pose startPosition = new Pose(72, 72, 0);

    /**
     * The last follower constructed by an opmode. The expected use case is for 
     * CompetitonTeleop to both save time during initialization and to track the 
     * start position. Opmodes that consume this should not 
     * 
     * The only opmode expected to update this value are autos. Unless a teleop
     * constructs a new follower, it should **never** update this value.
     */
    public static Follower follower = null;
}