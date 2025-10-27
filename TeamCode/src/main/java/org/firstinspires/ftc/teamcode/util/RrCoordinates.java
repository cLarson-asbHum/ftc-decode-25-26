package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.geometry.CoordinateSystem;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;

/**
 * Represents a coordinate system where the origin is at the center 
 * of the field, the +x axis faces towards the blue alliance, and -y
 * faces towards the audience. In other words: 
 * 
 *     - The blue goal is (-72, +72)
 *     - The red goal is (+72, +72)
 *     - The red loading zone is (-72, -72)
 *     - The blue loading zone is (+72, -72)
 * 
 * In effect, this is the PedroPathing DECODE coordinates, only translated so
 * the field center is at (0,0) rather than (72, 72)
 * 
 * Distances are in inches, and angles are in radians.
 */
public enum RrCoordinates implements CoordinateSystem {
    // HACK: This is an enum so that we have singleton behavior without calling
    //       "getInstance()" a bunch, which can get annoying.
    INSTANCE;

    @Override
    public Pose convertToPedro(Pose originalPose) {
        return new Pose(
            originalPose.getX() + 72, 
            originalPose.getY() + 72, 
            originalPose.getHeading(), // Identical 
            PedroCoordinates.INSTANCE
        );
    }

    @Override
    public Pose convertFromPedro(Pose pedroPose) {
        return new Pose(
            pedroPose.getX() - 72,
            pedroPose.getY() - 72,
            pedroPose.getHeading(), // Identical
            RrCoordinates.INSTANCE
        ); 
    }
}