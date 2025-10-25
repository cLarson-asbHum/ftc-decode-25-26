package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.teamcode.util.ArtifactColor;

import static org.firstinspires.ftc.teamcode.util.ArtifactColor.GREEN;
import static org.firstinspires.ftc.teamcode.util.ArtifactColor.PURPLE;

/**
 * Uses a sensor or more to determine what the motif the obelisk indicates. 
 */
public interface MotifGetter {
    /**
     * A triplet of artifacts color, as inidcated by the obelisk. Motifs score
     * 
     */
    public static enum Motif {
        /**
         * The first color (closest to the gate) is green; all others are purple.
         * AprilTag id 21
         */
        FIRST_GREEN( 21, GREEN,  PURPLE, PURPLE ),

        /**
         * The middle color is green; all others are purple. AprilTag id 22
         */
        MIDDLE_GREEN( 22, PURPLE, GREEN,  PURPLE ),

        /**
         * The last color (closest to the goal) is green; all others are purple.
         * AprilTag id 23
         */
        LAST_GREEN( 23, PURPLE, PURPLE, GREEN  );

        public final int tagId;
        public final ArtifactColor firstColor;
        public final ArtifactColor middleColor;
        public final ArtifactColor lastColor;

        private Motif(
            int tagId,
            ArtifactColor firstColor, 
            ArtifactColor middleColor, 
            ArtifactColor lastColor
        ) {
            this.tagId = tagId;
            this.firstColor  = firstColor;
            this.middleColor = middleColor;
            this.lastColor   = lastColor;
        }

        /**
         * Gets the specific color at the given index in the motif. Index 0 is 
         * the closest to the gate, 1 is the middle, and 2 is the closest to the 
         * goal.
         * 
         * @param index Zero-based indicator of position the color to get. Must be 
         * between 0 and 2 (inclusive). 
         * @return The color in the specified position in the motif.
         */
        public ArtifactColor getColor(int index) {
            if(index < 0 || index > 2) {
                throw new IndexOutOfBoundsException(index);
            }

            final ArtifactColor[] colors = new ArtifactColor[]{firstColor, middleColor, lastColor};
            return colors[index];
        }
    }

    /**
     * Gets the most accurate motif based off of sensor readings. Should 
     * default to some safe motif if the motif cannot be determined. 
     * 
     * @return The observed motif, or some fallback if none can be determined.
     */
    public Motif getMotif();
}