package org.firstinspires.ftc.teamcode.auto;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PwmControl.PwmRange;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.io.IOException;
import java.util.ArrayList;
import java.util.concurrent.TimeUnit;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;

import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.ballistics.BallisticArc;
import org.firstinspires.ftc.teamcode.ballistics.BallisticArcSelection.Criterion;
import org.firstinspires.ftc.teamcode.hardware.ArtifactColorRangeSensor;
import org.firstinspires.ftc.teamcode.hardware.MotifWebcam;
import org.firstinspires.ftc.teamcode.hardware.subsystem.BasicMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.subsystem.BlockerSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystem.CarwashIntake;
import org.firstinspires.ftc.teamcode.hardware.subsystem.FlywheelTubeShooter;
import org.firstinspires.ftc.teamcode.hardware.subsystem.LinearHingePivot;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.subsystem.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystem.ShooterSubsystem.Status;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.res.R;
import org.firstinspires.ftc.teamcode.teleop.ClearCommandScheduler;
import org.firstinspires.ftc.teamcode.temp.TimeInjectionUtil;
import org.firstinspires.ftc.teamcode.util.ArtifactColor;
import org.firstinspires.ftc.teamcode.util.AimbotManager;
import org.firstinspires.ftc.teamcode.util.ConfigPose;
import org.firstinspires.ftc.teamcode.util.KeyPoses;
import org.firstinspires.ftc.teamcode.util.MotifGetter;
import org.firstinspires.ftc.teamcode.util.MotifGetter.Motif;
import org.firstinspires.ftc.teamcode.util.OpModeData;
import org.firstinspires.ftc.teamcode.util.RrCoordinates;
import org.firstinspires.ftc.teamcode.util.Util;
import org.firstinspires.ftc.teamcode.util.WrapConcurrentCommand;
import org.firstinspires.ftc.vision.VisionPortal;

import static org.firstinspires.ftc.teamcode.util.ArtifactColor.PURPLE;

// @Configurable
@Autonomous(name = "Penny Pincher Positioning", group = "A - Main")
public class PennyPincherPositioning extends LinearOpMode {
    private ElapsedTime timer = new ElapsedTime();
    private DcMotor backRight = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft = null;
    private DcMotor frontLeft = null;
    private IMU imu = null;
    private double startDeer = 0;

    public static int GAIN = 50;
    public static int EXPOSURE_MS = 1;

    public static double ROBOT_LENGTH = 16; // Inches parallel to the robot's forward-facing axis
    public static double ROBOT_WIDTH = 16; // Inches perpendicular to the robot's forward-facing axis 

    public static double CAMERA_YAW_OFFSET = 0; // In radians

    public static double SHOT_SPEED = 280; // Determined using the ballistic arc text user interface
    public static double SHOT_ANGLE = Math.toRadians(51.1); // Determined using the ballistic arc text user interface

    public static ConfigPose START_POS = new ConfigPose(
        // In Inches. Coveriing the jigsaw covering the center line
        48 + ROBOT_WIDTH / 2,

        // In Inches. Is along the top-most grid edge
        ROBOT_LENGTH / 2 + 2, // Adding 4 because of the intake

        // In Radians. Shooter facing the obelisk
        Math.toRadians(-90)
    );

    public static ConfigPose SHOOTING_POS = new ConfigPose(KeyPoses.Blue.FAR_SHOOTING);

    public static ConfigPose OBELISK = new ConfigPose(
        72,
        144, 
        -Math.PI / 2
    );

    
    private ArtifactColorRangeSensor rightReload = null;
    private ArtifactColorRangeSensor leftReload = null;

    private FlywheelTubeShooter shooter = null;
    private CarwashIntake intake = null;
    private BasicMecanumDrive drivetrain = null;
    private BlockerSubsystem leftBlocker = null;
    private BlockerSubsystem rightBlocker = null;

    private boolean isRed = false;
    private boolean inCompetitonMode = false;
    
    private ArrayList<String> nullDeviceNames = new ArrayList<>();
    private ArrayList<Class<?>> nullDeviceTypes = new ArrayList<>();


    /**
     * Attempts to get the given hardware from the hardwareMap. If it cannot be 
     * found, then it returns null without finding an error.
     * 
     * This method should be used instead of hardwareMap.get() because it allows
     * us to see **all** the hardware that we cannot find.
     * 
     * @return The hardware with that name, or null if it cannot be found.
     */
    private <T extends HardwareDevice> T findHardware(Class<T> hardwareType, String name) {
        final T result = hardwareMap.tryGet(hardwareType, name);

        // Adding it to the list if null
        if(result == null) {
            nullDeviceNames.add(name);
            nullDeviceTypes.add(hardwareType);
        }

        return result;
    }

    /**
     * Throws an exception if any devices are in the nullDeviceNames or 
     * nullDeviceTypes lists. The thrown exception contains the names and types 
     * of all null hardware devices. 
     */
    private void throwAFitIfAnyHardwareIsNotFound() {
        if(nullDeviceNames.size() != 0 || nullDeviceTypes.size() != 0) {
            String concat = "";

            for(int i = 0; i < nullDeviceNames.size() || i < nullDeviceNames.size(); i++) {
                final String name = nullDeviceNames.get(i);
                final Class type = nullDeviceTypes.get(i); 
                concat += "\n    ";

                if(name != null) {
                    concat += '"' + name + '"';
                } else {
                    concat += "[null]";
                }

                concat += " with type ";
                
                if(type != null) {
                    concat += type.getName() + ".class";
                } else {
                    concat += "[null]";
                }
            }

            throw new RuntimeException("Cannot find hardware:" + concat);
        }
    }

    private Pose mirror(Pose pose, boolean doMirror) {
        if(doMirror) {
            return new Pose(
                72 - (pose.getX() - 72), 
                pose.getY(), 
                AngleUnit.normalizeRadians(Math.PI - pose.getHeading())
            );
        }

        return pose;
    }

    private Map<String, PathChain> createPaths(Follower follower, boolean isRed) {
        final Map<String, PathChain> result = new HashMap<>();

        final Pose start = mirror(START_POS.pedroPose(), isRed);

        result.put("park", follower
            .pathBuilder()
            .addPath(new BezierLine(
                start, 
                mirror(new Pose(9, 9, start.getHeading()), isRed)
            ))
            .setTangentHeadingInterpolation()
            .build()
        );

        return result;
    }

    @Override
    public void runOpMode() {

        telemetry.setMsTransmissionInterval(30);

        // Creating subsystems
        // final Subsystem[] subsystems = createSubsystems(hardwareMap);
        
        // Creating the webcam
        final WebcamName obeliskViewerCam = null;
        final MotifWebcam motifGetter = null;

        // setManualExposure(motifGetter, GAIN, EXPOSURE_MS);

        // Bulk caching
        final List<LynxModule> modules = hardwareMap.getAll(LynxModule.class);

        for(final LynxModule module : modules) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // Creating paths
        final Follower follower = Constants.createFollower(hardwareMap);
        Map<String, PathChain> paths = createPaths(follower, isRed);
        OpModeData.follower = follower;

        // Creating the aimbot
        // This isn't used by the auto, but we want to save time for teleop

        // Init loop
        while(opModeInInit()) {
            OpModeData.isRed = isRed;
            OpModeData.inCompetitonMode = inCompetitonMode;

            telemetry.addData("Status", "Initialized");
            telemetry.addLine();
            // telemetry.addData("Total arcs", OpModeData.selection.size());
            telemetry.addLine();
            telemetry.addLine(Util.header("Settings"));
            telemetry.addLine();
            telemetry.addData("Toggle isRed", "A");
            telemetry.addData("isRed", isRed);
            telemetry.addLine();
            telemetry.addData("Toggle competiton mode", "Y");
            telemetry.addData("Competiton mode", OpModeData.inCompetitonMode);
            telemetry.update();

            if(gamepad1.aWasPressed()) {
                isRed = !isRed;
                paths = createPaths(follower, isRed);
            }
            
            if(gamepad1.yWasPressed()) {
                inCompetitonMode = !inCompetitonMode;
            }
        }
        
        waitForStart();
        follower.setPose(mirror(START_POS.pedroPose(), isRed));

        // Get the motif 
        final boolean cameraExists = obeliskViewerCam != null && motifGetter != null;
        Motif motif = null;

        // intake.intakeGamePieces();
        follower.followPath(paths.get("park"), false);

        while(follower.isBusy() && opModeIsActive()) {
            follower.update();
            // CommandScheduler.getInstance().run();
            OpModeData.startPosition = follower.getPose();
        }


        // END
        OpModeData.startPosition = follower.getPose();
        CommandScheduler.getInstance().reset();
    }

    private void runUntilCompleted(Command command) {
        CommandScheduler.getInstance().schedule(command);
        while(!command.isFinished() && opModeIsActive()) {
            telemetry.update();
            CommandScheduler.getInstance().run();
        }
    }

    private boolean setManualExposure(MotifWebcam motifGetter, int exposureMS, int gain) {
        // Ensure Vision Portal has been setup.
        if (motifGetter.getStream() == null) {
            return false;
        }

        // Wait for the camera to be open
        if (motifGetter.getStream().getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (motifGetter.getStream().getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!isStopRequested())
        {
            // Set exposure.  Make sure we are in Manual Mode for these values to take effect.
            ExposureControl exposureControl = motifGetter.getStream().getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);

            // Set Gain.
            GainControl gainControl = motifGetter.getStream().getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
            return (true);
        } else {
            return (false);
        }
    }
}
