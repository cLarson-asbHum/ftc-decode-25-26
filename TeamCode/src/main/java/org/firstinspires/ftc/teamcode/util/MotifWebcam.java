package org.firstinspires.ftc.teamcode.util;

import android.util.Size;

import com.bylazar.configurables.annotations.Configurable;

import java.util.ArrayList;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
// import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Configurable
public class MotifWebcam implements MotifGetter {
    public static int PORTAL_WIDTH_PX = 640; // Only works at construction time
    public static int PORTAL_HEIGHT_PX = 480; // Only works at construction time

    public static int MIN_TAG_ID = 21;
    public static int MAX_TAG_ID = 23;

    public static double DESIRED_YAW = -Math.PI / 2; // In radians 

    // public static Motif DEFAULT_MOTIF = Motif.MIDDLE_GREEN;

    protected final VisionPortal stream;
    protected final AprilTagProcessor processor;

    // private YawPitchRollAngles globalRot = new YawPitchRollAngles(AngleUnit.RADIANS, 0, 0, 0, -1);
    // private YawPitchRollAngles cameraOffsetYaw = new YawPitchRollAngles(AngleUnit.RADIANS, 0, 0, 0, -1);
    private double globalRobotYaw = 0;
    private double cameraOffsetYaw = 0;

    /**
     * Creates a MotifGetteer that uses the given webcam to determine what april 
     * tag we're seeing. The angle parameter is used to determine what global
     * angle the observed April Tags are facing so that if we see multiple April 
     * Tags, we can choose the one that is closest to facing towards the audience
     * 
     * @param webcam The webcam (from the hardwareMap) used to detect april tags
     * @param cameraOffsetYaw Angle offset from the robot's forward-pointing axis
     */
    public MotifWebcam(CameraName stream, double cameraOffsetYaw) {
        setCameraOffsetYaw(cameraOffsetYaw);

        // Creating the April Tag Processor
        this.processor = new AprilTagProcessor.Builder()         
            // ---------- LOCALIZATION ---------- 
            // .setLensIntrinsics(, , , ,) // We use a C270, which is automatically supported by the SDK
            // .setCameraPose(cameraPosition, cameraOffsetYaw) // This is only used if you are deriving the global robot pose from the tag
            .setOutputUnits(DistanceUnit.INCH, AngleUnit.RADIANS) // PredoPathing uses Inches, and Radians are better for math
            
            // ---------- TAGS TO DETECT ---------- 
            .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11) // Square April Tags with white borders and black backgrounds
            .setTagLibrary(AprilTagGameDatabase.getDecodeTagLibrary()) // DECODE specific april tags

            // ---------- MISCELLANEOUS ---------- 
            // .setNumThreads(2) // Let the SDK handle this value
            .setSuppressCalibrationWarnings(false) // Show every warning we get

            // ---------- LIVE PREVIEW ---------- 
            .setDrawAxes(true) 
            .setDrawCubeProjection(true) 
            .setDrawTagOutline(true)
            .setDrawTagID(true)
            .build();
            
        // Creating the vision portal
        this.stream = new VisionPortal.Builder()
            // ---------- FUNDAMENTAL ---------- 
            .setCamera(stream)
            .addProcessor(processor)

            // ---------- STREAMING ---------- 
            .setAutoStartStreamOnBuild(true) // So that the users doesn't have to call enable()
            .setCameraResolution(new Size(PORTAL_WIDTH_PX, PORTAL_HEIGHT_PX)) // Keep the resolution small to save bandwith and CPU
            .setStreamFormat(VisionPortal.StreamFormat.MJPEG) // Use less bandwith

            // ---------- RC PREVIEW ----------  
            .setAutoStopLiveView(true) // Just in case we do use live preview
            .enableLiveView(false) // Using a control hub, not a phone.

            // ---------- DS PREVIEW ---------- 
            .setShowStatsOverlay(true) // Only for when we are doing the DS camera preview
            .build();
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
        this.stream.stopStreaming();
    }

    public void enable() {
        this.stream.resumeStreaming();
    }

    protected Motif getMotifFromTag(AprilTagDetection tag) {
        // The tag is invalid; return null to signal to the opmode such event;
        if(tag.id < MIN_TAG_ID || tag.id > MAX_TAG_ID) {
            return null;
        }

        if(tag.id == Motif.FIRST_GREEN.tagId) {
            return Motif.FIRST_GREEN;
        }

        if(tag.id == Motif.MIDDLE_GREEN.tagId) {
            return Motif.MIDDLE_GREEN;
        }

        // tag.id == LAST_GREEN.tagId
        return Motif.LAST_GREEN;
    }

    protected double getTagGlobalYaw(AprilTagDetection tag) {
        return AngleUnit.normalizeRadians(
            tag.ftcPose.yaw 
            + globalRobotYaw 
            + cameraOffsetYaw
            - Math.PI
        );
    }

    public VisionPortal getStream() {
        return this.stream;
    }
    
    public AprilTagProcessor getProcessor() {
        return this.processor;
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
    public AprilTagDetection getMostAccurateTag(ArrayList<AprilTagDetection> tags) {
        double minAbsoluteYaw = Double.POSITIVE_INFINITY; // Absolute difference from the desired yaw value, in radians
        AprilTagDetection mostAccurateTag = null;

        for(final AprilTagDetection tag : tags) {
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
        final ArrayList<AprilTagDetection> obeliskTags = processor.getFreshDetections(); 

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