package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public final class KeyPoses {
    public static final class Blue {
        public static final Pose BASE = new Pose(105, 33);
        public static final Pose GOAL = new Pose(12, 144 - 7.25);
        public static final Pose LOADING = new Pose(132, 12);

        public static final Pose SHOOTING = new Pose(
            56,
            106, // A little higher than the jigsaw so we are facing the goal
            Math.toRadians(325)
        );

        // Artifacts are on the side closest to the blue goal
        // Colors are ordered coming in from the inner edge
        public static final Pose LAST_PURPLE = new Pose(24, 84);
        public static final Pose MIDDLE_PURPLE = new Pose(24, 60);
        public static final Pose FIRST_PURPLE = new Pose(24, 46);
        // public static final Pose POSE = new Pose();

        
    }

    public static final class Red {
        public static final Pose BASE = Blue.BASE.mirror();
        public static final Pose GOAL = Blue.GOAL.mirror();
        public static final Pose LOADING = Blue.LOADING.mirror();
        
        public static final Pose SHOOTING = new Pose(
            144 - KeyPoses.Blue.SHOOTING.getX(),
            144 - KeyPoses.Blue.SHOOTING.getY(), // A little higher than the jigsaw so we are facing the goal
            AngleUnit.normalizeRadians(Math.PI - KeyPoses.Blue.SHOOTING.getHeading())
        );

        // Artifacts are on the side closest to the blue goal
        // Colors are ordered coming in from the inner edge
        public static final Pose LAST_PURPLE = Blue.LAST_PURPLE.mirror();
        public static final Pose MIDDLE_PURPLE = Blue.MIDDLE_PURPLE.mirror();
        public static final Pose FIRST_PURPLE = Blue.FIRST_PURPLE.mirror();
    }

    public static Pose base(boolean isRed) {
        return isRed ? Red.BASE : Blue.BASE;
    }

    public static Pose goal(boolean isRed) {
        return isRed ? Red.GOAL : Blue.GOAL;
    }

    public static Pose loading(boolean isRed) {
        return isRed ? Red.LOADING : Blue.LOADING;
    }
    
    public static Pose shooting(boolean isRed) {
        return isRed ? Red.SHOOTING : Blue.SHOOTING;
    }

    public static Pose lastPurple(boolean isRed)   {
        return isRed ? Red.LAST_PURPLE : Blue.LAST_PURPLE;
    }

    public static Pose middlePurple(boolean isRed) {
        return isRed ? Red.MIDDLE_PURPLE : Blue.MIDDLE_PURPLE;
    }

    public static Pose firstPurple(boolean isRed)  {
        return isRed ? Red.FIRST_PURPLE : Blue.FIRST_PURPLE;
    }

}