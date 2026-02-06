package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;

import java.util.ArrayList;
import java.util.List;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit; 
import org.firstinspires.ftc.teamcode.util.MotifGetter;
import org.firstinspires.ftc.teamcode.util.MotifGetter.Motif;

public class MotifLimelight implements MotifGetter {
    // public static int PORTAL_WIDTH_PX = 640; // Only works at construction time
    // public static int PORTAL_HEIGHT_PX = 480; // Only works at construction time

    public static int MIN_TAG_ID = 21;
    public static int MAX_TAG_ID = 23;

    public static double DESIRED_YAW = -Math.PI / 2; // In radians 

    protected final Limelight3A limelight;

    private double globalRobotYaw = 0;
    private double cameraOffsetYaw = 0;

    /**
     * Creates a MotifGetteer that uses the given limelight to determine what april 
     * tag we're seeing. The angle parameter is used to determine what global
     * angle the observed April Tags are facing so that if we see multiple April 
     * Tags, we can choose the one that is closest to facing towards the audience
     * 
     * @param limelight The limelight used to detect april tags
     * @param pipelineSwitch The index of which pipeline to use. Should be the april 
     * tag pipeline
     * @param cameraOffsetYaw Angle offset from the robot's forward-pointing axis
     */
    public MotifLimelight(Limelight3A limelight, int pipelineSwitch, double cameraOffsetYaw) {
        setCameraOffsetYaw(cameraOffsetYaw);
        this.limelight = limelight;
        this.limelight.pipelineSwitch(pipelineSwitch);
        this.limelight.start();
    } 

    /**
     * Sets the orientation of the robot, using the offical field coordinate
     * system. In other words, having yaw, pitch, and roll all equal to 0 means
     * that the robot is upright and looking towards the audience. Increasing
     * yaw positively rotates the robot counter clockwise.
     * 
     * This is used to determine the direction of april tags on the obelisk. 
     * If multiple tags are on the obelisk, then the one closest to facing the 
     * audience is used.
     * 
     * The global orientation defaults to Yaw 0, Pitch 0, Roll 0 if this method 
     * is not called.
     * 
     * @param robotYaw Yaw relative to the the field's positive x axis, in radians
     */
    public void setGlobalRobotYaw(double robotYaw) {
        this.globalRobotYaw = robotYaw;
    }

    /**
     * Sets the angle offset of the camera relative to the robot's forward
     * axis. This is identical to setting the camera angle in the 
     * constructor. The previous value is overridden.
     * 
     * @param cameraOffsetYaw offset from the robot's forward-pointing axis, in radians
     */
    public void setCameraOffsetYaw(double cameraOffsetYaw) {
        this.cameraOffsetYaw = cameraOffsetYaw;
    }

    public void disable() {
        limelight.pause();
    }

    public void enable() {
        limelight.start(); // Also resumes, not just initializes
    }

    public void close() {
        limelight.stop();
    }

    protected Motif getMotifFromTag(FiducialResult tag) {
        // The tag is invalid; return null to signal to the opmode such event;
        if(tag.getFiducialId() < MIN_TAG_ID || tag.getFiducialId() > MAX_TAG_ID) {
            return null;
        }

        if(tag.getFiducialId() == Motif.FIRST_GREEN.tagId) {
            return Motif.FIRST_GREEN;
        }

        if(tag.getFiducialId() == Motif.MIDDLE_GREEN.tagId) {
            return Motif.MIDDLE_GREEN;
        }

        // tag.getFiducialId() == LAST_GREEN.tagId
        return Motif.LAST_GREEN;
    }

    protected double getTagGlobalYaw(FiducialResult tag) {
        return AngleUnit.normalizeRadians(
            tag.getTargetPoseCameraSpace().getOrientation().getYaw(AngleUnit.RADIANS)
            + globalRobotYaw 
            + cameraOffsetYaw
            - Math.PI
        );
    }

    public Limelight3A getLimelight() {
        return this.limelight;
    }

    /**
     * Gets the tag that is facin the audience the most, based off of the 
     * robot's current global position. This requires that the camera position 
     * and angle provided at construction time as well as 
     * `setGlobalRobotYaw()` having been called to be accurate
     * 
     * Returns null if the provided tags list is empty. Throws if the list or any 
     * tag in it is null.
     * 
     * @param tags All the tags that were detected
     * @return The tag closest to yaw = 0, in the field coordinate system.
     */
    public FiducialResult getMostAccurateTag(List<FiducialResult> tags) {
        double minAbsoluteYaw = Double.POSITIVE_INFINITY; // Absolute difference from the desired yaw value, in radians
        FiducialResult mostAccurateTag = null;

        for(final FiducialResult tag : tags) {
            final double currentAbsoluteYaw = Math.abs(getTagGlobalYaw(tag) - DESIRED_YAW);

            // If the angle is closer to the desired angle than the most 
            // accurate thus far, then update the most accurate tag 
            if(currentAbsoluteYaw < minAbsoluteYaw) {
                minAbsoluteYaw = currentAbsoluteYaw;
                mostAccurateTag = tag;
            }
        }

        return mostAccurateTag;
    }

    @Override
    public Motif getMotif() {
        // Finding all obelisk april tags
        final LLResult llResult = limelight.getLatestResult();

        if(llResult == null || !llResult.isValid()) {
            return null;
        }

        final List<FiducialResult> obeliskTags = llResult.getFiducialResults(); 

        // No tags were detected! Return null to signal to the opmode such event 
        if(obeliskTags == null || obeliskTags.size() == 0) {
            return null;
        }

        // Filtering the result if there are mutliple tags
        if(obeliskTags.size() > 1) {
            return getMotifFromTag(getMostAccurateTag(obeliskTags)); 
        }

        // There is only one tag, so return that tag's motif
        return getMotifFromTag(obeliskTags.get(0));
    }
}
