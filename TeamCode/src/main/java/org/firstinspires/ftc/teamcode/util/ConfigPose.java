package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.geometry.CoordinateSystem;
import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.PedroCoordinates;

public class ConfigPose {
    /**
     * The x coordinate. The coordinate system and units are assumed to be 
     * PedroPathing coords unless the `pedroPose()` is told otherwise.
     * 
     * Defaults to 0 (inches).
     */
    public double x;
    
    /**
     * The y coordinate. The coordinate system and units are assumed to be 
     * PedroPathing coords unless the `pedroPose()` is told otherwise.
     * 
     * Defaults to 0 (inches).
     */
    public double y;
    
    /**
     * The angle of the bot. The coordinate system and units are assumed to be 
     * PedroPathing coords unless the `pedroPose()` is told otherwise.
     * 
     * Defaults to 0 (radians)
     */
    public double yaw;

    /**
     * The CoordinateSystem used when calling `pedroPose()` with no arguments. 
     * This is completely ignored when calling `pedroPose()` with arguments.
     */
    public CoordinateSystem defaultSystem;

    /**
     * Creates a pose of only zeros, in the Pedro coordinate system.
     */
    public ConfigPose() {
        this(0, 0, 0);
    }

    /**
     * Creates a pose with a yaw of zero, in the Pedro coorindate system.
     */
    public ConfigPose(double x, double y) {
        this(x, y, 0);
    }

    /**
     * Creates a pose in the Pedro coordinate system
     */
    public ConfigPose(double x, double y, double yaw) {
        this(x, y, yaw, PedroCoordinates.INSTANCE);
    }

    /**
     * Creates a ConfigPose with the given CoordinateSystem as the default. 
     */
    public ConfigPose(double x, double y, double yaw, CoordinateSystem defaultSystem) {
        this.x = x;
        this.y = y;
        this.yaw = yaw;
        this.defaultSystem = defaultSystem;
    } 

    public ConfigPose(Pose pose) {
        this(pose.getX(), pose.getY(), pose.getHeading());
    }

    /**
     * Gets the equivalent PedroPathing `Pose`, assuming the coordinates
     * to already be in the specified coordinate system. 
     * 
     * @param coorindateSystem The system the components are currently specified
     * in
     * @return PedroPathing pose using this's coordinates as-is and 
     * the given coordinate system
     */
    public Pose pedroPose(CoordinateSystem coordinateSystem) {
        return new Pose(x, y, yaw, coordinateSystem);
    }

    /**
     * Gets the equivalent PedroPathing `Pose`, assuming the coordinates to 
     * already be in the default system specified at construction (or overriden
     * since then). This is eequivalent to calling 
     * `configPose.pedroPose(configPose.defaultSystem)`.
     * 
     * @return The PedroPathing pose, using the default coordinate system.
     */
    public Pose pedroPose() {
        return pedroPose(defaultSystem);
    }
}