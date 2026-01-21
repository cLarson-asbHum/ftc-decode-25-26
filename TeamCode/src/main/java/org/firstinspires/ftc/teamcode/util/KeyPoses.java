package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public final class KeyPoses {
    public static final class Blue {
        public static final Pose BASE = new Pose(105, 33);
        public static final Pose GOAL_WALL = new Pose(24 - 6.57, 120 + 13.0); // Center of wall, determined from an image
        public static final Pose GOAL_CENTER = new Pose(13.11, 136.32); // Center of opening, determined from an image
        public static final Pose GOAL_BACKBOARD = new Pose(6, 144);
        public static final Pose LOADING = new Pose(132, 12);

        public static final Pose SHOOTING = new Pose(
            56,
            106, // A little higher than the jigsaw so we are facing the goal
            AngleUnit.normalizeRadians(Math.toRadians(325))
        );

        private static final double farX = 48 + 11.5;
        private static final double farY = 0 + 18.1;
        public static final Pose FAR_SHOOTING = new Pose(
            // Determines from an image
            farX,
            farY,
            AngleUnit.normalizeRadians(Math.PI + Math.atan2(
                GOAL_CENTER.getY() - farY,
                GOAL_CENTER.getX() - farX
            ) + Math.toRadians(5))
        );

        // Artifacts are on the side closest to the blue goal
        // Colors are ordered coming in from the inner edge
        public static final Pose LAST_GREEN = new Pose(24, 84);
        public static final Pose MIDDLE_GREEN = new Pose(24, 60);
        public static final Pose FIRST_GREEN = new Pose(24, 46);
        // public static final Pose POSE = new Pose();

        
    }

    public static final class Red {
        public static final Pose BASE = Blue.BASE.mirror();
        public static final Pose GOAL_WALL = Blue.GOAL_WALL.mirror();
        public static final Pose GOAL_CENTER = Blue.GOAL_CENTER.mirror();
        public static final Pose GOAL_BACKBOARD = Blue.GOAL_BACKBOARD.mirror();
        public static final Pose LOADING = Blue.LOADING.mirror();
        
        public static final Pose SHOOTING = new Pose(
            144 - KeyPoses.Blue.SHOOTING.getX(),
            144 - KeyPoses.Blue.SHOOTING.getY(), // A little higher than the jigsaw so we are facing the goal
            AngleUnit.normalizeRadians(Math.PI - KeyPoses.Blue.SHOOTING.getHeading())
        );

        
        public static final Pose FAR_SHOOTING = new Pose(
            144 - KeyPoses.Blue.FAR_SHOOTING.getX(),
            144 - KeyPoses.Blue.FAR_SHOOTING.getY(), 
            AngleUnit.normalizeRadians(Math.PI - KeyPoses.Blue.FAR_SHOOTING.getHeading())
        );

        // Artifacts are on the side closest to the blue goal
        // Colors are ordered coming in from the inner edge
        public static final Pose LAST_GREEN = Blue.LAST_GREEN.mirror();
        public static final Pose MIDDLE_GREEN = Blue.MIDDLE_GREEN.mirror();
        public static final Pose FIRST_GREEN = Blue.FIRST_GREEN.mirror();
    }

    public static Pose base(boolean isRed) {
        return isRed ? Red.BASE : Blue.BASE;
    }

    public static Pose goalWall(boolean isRed) {
        return isRed ? Red.GOAL_WALL : Blue.GOAL_WALL;
    }
    

    public static Pose goalCenter(boolean isRed) {
        return isRed ? Red.GOAL_CENTER : Blue.GOAL_CENTER;
    }
    
    public static Pose goalBackboard(boolean isRed) {
        return isRed ? Red.GOAL_BACKBOARD : Blue.GOAL_BACKBOARD;
    }

    public static Pose loading(boolean isRed) {
        return isRed ? Red.LOADING : Blue.LOADING;
    }
    
    public static Pose shooting(boolean isRed) {
        return isRed ? Red.SHOOTING : Blue.SHOOTING;
    }

    public static Pose farShooting(boolean isRed) {
        return isRed ? Red.FAR_SHOOTING : Blue.FAR_SHOOTING;
    }

    public static Pose lastGreen(boolean isRed)   {
        return isRed ? Red.LAST_GREEN : Blue.LAST_GREEN;
    }

    public static Pose middleGreen(boolean isRed) {
        return isRed ? Red.MIDDLE_GREEN : Blue.MIDDLE_GREEN;
    }

    public static Pose firstGreen(boolean isRed)  {
        return isRed ? Red.FIRST_GREEN : Blue.FIRST_GREEN;
    }

}