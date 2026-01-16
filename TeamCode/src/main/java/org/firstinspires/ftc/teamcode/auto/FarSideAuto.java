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

@Configurable
@Autonomous(name = "Gary Larson's Colorblind Far Side auto", group = "A - Main")
public class FarSideAuto extends LinearOpMode {
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

    public static double SHOT_SPEED = 338; // Determined using the ballistic arc text user interface
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

        // Seting up the points
        final Pose start = mirror(START_POS.pedroPose(), isRed);
        final Pose shooting = mirror(SHOOTING_POS.pedroPose(), isRed);

        // Creating the paths
        result.put("goFromCameraToShooting", follower
            .pathBuilder()
            .addPath(
                new BezierLine(start, shooting)
            )
            .setLinearHeadingInterpolation(start.getHeading(), shooting.getHeading())
            .build()
        );

        final PathChain grabArtifacts = follower
            .pathBuilder()
            .addPath(new BezierCurve(
                shooting,
                mirror(new Pose(71.038, 31.500), isRed),
                mirror(new Pose(54.489, 31.500), isRed),
                mirror(new Pose(50.089, 31.500), isRed)
            ))
            .setLinearHeadingInterpolation(shooting.getHeading(), isRed ? 0 : Math.toRadians(-180))
            .addPath(new BezierLine(
                mirror(new Pose(50.089, 31.5), isRed),
                mirror(new Pose(38.089, 31.500), isRed)
            ))
            .setConstantHeadingInterpolation(isRed ? 0 : Math.toRadians(-180))
            .addPath(new BezierCurve(
                mirror(new Pose(38.089, 31.500), isRed),
                mirror(new Pose(29.000, 38.500), isRed),
                mirror(new Pose(27.000, 38.500), isRed),
                mirror(new Pose(21, 38.500), isRed)
            ))
            .setConstantHeadingInterpolation(isRed ? 0 : Math.toRadians(-180))
            .build();
        final Path goBackToShoot = new Path(new BezierLine(
            mirror(new Pose(11.5, 38.500), isRed), 
            shooting
        ));
        final PathChain grabArtifactsAgain = follower
            .pathBuilder()
            .addPath(new BezierLine(
                shooting,
                mirror(new Pose(26.578, 10.000), isRed)
            ))
            .addPath(new BezierLine(
                mirror(new Pose(30.50, 10.000), isRed),
                mirror(new Pose(11.50, 10.000), isRed)
            ))
            .build();
        
        final Path goBackToShootAgain = new Path(new BezierLine(
            mirror(new Pose(11.5, 10), isRed), 
            shooting
        ));

        // turnSoAsToIntake.setLinearHeadingInterpolation(shooting.getHeading(), isRed ? 0 : Math.PI);
        final double grabHeading = isRed ? 0 : -Math.PI;
        grabArtifacts.getPath(0).setLinearHeadingInterpolation(shooting.getHeading(), grabHeading);
        grabArtifacts.getPath(1).setConstantHeadingInterpolation(grabHeading);
        grabArtifacts.getPath(2).setConstantHeadingInterpolation(grabHeading);
        grabArtifactsAgain.getPath(0).setLinearHeadingInterpolation(shooting.getHeading(), grabHeading);
        grabArtifactsAgain.getPath(1).setConstantHeadingInterpolation(grabHeading);
        goBackToShoot.setConstantHeadingInterpolation(shooting.getHeading());
        goBackToShootAgain.setConstantHeadingInterpolation(shooting.getHeading());

        result.put("grabArtifactsAndShoot", follower
            .pathBuilder()
            .addPath(grabArtifacts.getPath(0))
            .addPath(grabArtifacts.getPath(1))
            .addPath(grabArtifacts.getPath(2))
            .addPath(goBackToShoot)
            .build()
        );

        result.put("grabArtifactsAndShootAgain", follower
            .pathBuilder()
            .addPath(grabArtifactsAgain.getPath(0))
            .addPath(grabArtifactsAgain.getPath(1))
            .addPath(goBackToShootAgain)
            // .setReversed()
            .build()
        );

        result.put("park", follower
            .pathBuilder()
            .addPath(new BezierLine(
                shooting, 
                mirror(new Pose(48, 36, shooting.getHeading()), isRed)
            ))
            .setConstantHeadingInterpolation(shooting.getHeading())
            .build()
        );

        return result;
    }

    @Override
    public void runOpMode() {

        telemetry.setMsTransmissionInterval(30);

        // Creating subsystems
        // final Subsystem[] subsystems = createSubsystems(hardwareMap);
        final Robot robot = new Robot(hardwareMap, Set.of(
            Robot.Device.SHOOTER,
            Robot.Device.INTAKE,
            Robot.Device.LEFT_BLOCKER,
            Robot.Device.RIGHT_BLOCKER,
            Robot.Device.RAMP_PIVOT
        ));
        shooter      = robot.getShooter();
        intake       = robot.getIntake();
        leftBlocker  = robot.getLeftBlocker();
        rightBlocker = robot.getRightBlocker();
        final LinearHingePivot rampPivot = robot.getRampPivot();
        shooter.setTelemetry(telemetry);
        
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
        final AimbotManager aimbot = new AimbotManager(shooter, rampPivot, OpModeData.selection);

        if(OpModeData.selection == null) {
            try {
                aimbot.init(R.raw.arcs, this::filterArc, telemetry);
            } catch(IOException exc) {
                throw new RuntimeException(exc);
            }
        }

        // Init loop
        while(opModeInInit()) {
            if(aimbot.isInitialized()) {
                OpModeData.selection = aimbot.getSelection();
                OpModeData.isRed = isRed;
                OpModeData.inCompetitonMode = inCompetitonMode;

                telemetry.addData("Status", "Initialized");
                telemetry.addLine();
                telemetry.addData("Total arcs", OpModeData.selection.size());
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
        }
        
        waitForStart();
        rampPivot.runToAngle(SHOT_ANGLE);
        follower.setStartingPose(mirror(START_POS.pedroPose(), isRed));

        // Get the motif 
        final boolean cameraExists = obeliskViewerCam != null && motifGetter != null;
        Motif motif = null;

        // Moving to the shooting position
        if(paths.get("goFromCameraToShooting") == null) {
            throw new RuntimeException("Cannot find path: goFromCameraToShooting");
        }

        shooter.charge();
        CommandScheduler.getInstance().run();
        follower.followPath(paths.get("goFromCameraToShooting"), true);
        while(follower.isBusy() && opModeIsActive()) {

            follower.update();
            OpModeData.startPosition =  follower.getPose();

            // Snapping a photo of the motif if we are facing it
            // The camera sees 60 degrees, but we subtract a bit to fully see the motif
            final double F_O_V = Math.toRadians(40); 
            final Pose currentPose = follower.getPose();
            final double targetAngle = Math.atan2(
                mirror(OBELISK.pedroPose(), isRed).getY() - currentPose.getY(), 
                mirror(OBELISK.pedroPose(), isRed).getX() - currentPose.getX()
            );

            if(cameraExists && motif == null && Util.near(currentPose.getHeading(), targetAngle, 0.5 * F_O_V)) { 
                motifGetter.setGlobalRobotYaw(currentPose.getHeading());
                motif = motifGetter.getMotif();
                motifGetter.disable(); // Save bandwidth and performance by not accessing the camera
            }
        }
        

        // If the motif coul dnt be found, use a defa`ult
        // if(motif == null && allPurple) {
        //     motif = Motif.ALL_PURPLE;
        /* }  else */ if(motif == null) {
            motif = Motif.FIRST_GREEN;
        }

        emptyClip(motif);

        // Moving to grab artifacts
        // This goes back to shooting afterwards
        follower.followPath(paths.get("grabArtifactsAndShoot"), true);

        boolean hasReloaded = false;
        // follower.setMaxPower(0.5);
        while(follower.isBusy() && opModeIsActive()) {
            if(follower.getChainIndex() == 1 || follower.getChainIndex() == 2) {
                follower.setMaxPower(0.33);
                intake.intakeGamePieces();
                leftBlocker.close();
                rightBlocker.close();
            } else {
                follower.setMaxPower(1.0);
                intake.holdGamePieces();
                leftBlocker.open();
                rightBlocker.open();
            }

            telemetry.addData("Position", follower.getPose());
            telemetry.addData("Shooting", mirror(SHOOTING_POS.pedroPose(), isRed));
            telemetry.update();

            follower.update();
            OpModeData.startPosition = follower.getPose();
            CommandScheduler.getInstance().run();
        }
        // follower.setMaxPower(1.0);

        // Shooting once again
        emptyClip(motif);
        
        // Moving to grab artifacts
        // This goes back to shooting afterwards
        follower.followPath(paths.get("grabArtifactsAndShootAgain"), false);
        
        // hasReloaded = false;
        while(follower.isBusy() && opModeIsActive()) {
            if(follower.getChainIndex() == 1) {
                follower.setMaxPower(0.5);
                intake.intakeGamePieces();
            } else {
                follower.setMaxPower(1.0);
                intake.holdGamePieces();
            }
            
            follower.update();
            OpModeData.startPosition = follower.getPose();
            CommandScheduler.getInstance().run();
        }

        // Shooting once again
        emptyClip(motif);

        // Getting leave points
        intake.holdGamePieces();
        shooter.uncharge();
        follower.followPath(paths.get("park"), false);

        while(follower.isBusy() && opModeIsActive()) {
            follower.update();
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

    private void emptyClip(Motif unused) {
        runUntilCompleted(WrapConcurrentCommand.wrapUntilNotState(
            shooter,
            () -> shooter.charge(SHOT_SPEED, true),
            FlywheelTubeShooter.Status.CHARGING
        ));

        // Shooting depth 1
        leftBlocker.open();
        rightBlocker.open();
        runUntilCompleted(shooter.fireCommand());

        // Reloading and going
        runUntilCompleted(new WrapConcurrentCommand<ShooterSubsystem.Status>(
            shooter,
            () -> shooter.charge(SHOT_SPEED, true),
            FlywheelTubeShooter.Status.CHARGED
        ));
        
        // Shooting
        intake.intakeGamePieces();
        runUntilCompleted(shooter.fireCommand());

        // Ending
        leftBlocker.close();
        rightBlocker.close();
        intake.holdGamePieces();
        // shooter.charge();
        CommandScheduler.getInstance().run();
    }
    
    private void emptyForefrontClip(Motif unused) {
        runUntilCompleted(WrapConcurrentCommand.wrapUntilNotState(
            shooter,
            () -> shooter.charge(SHOT_SPEED, true),
            FlywheelTubeShooter.Status.CHARGING
        ));
        // final ElapsedTime timer = new ElapsedTime(); // FIXME: timeUtil

        // Shooting depth 1
        leftBlocker.open();
        rightBlocker.open();
        runUntilCompleted(WrapConcurrentCommand.wrapUntilNotState(
            shooter,
            () -> shooter.charge(SHOT_SPEED, true),
            FlywheelTubeShooter.Status.CHARGING
        ));

        // Ending
        intake.holdGamePieces();
        // shooter.charge();
        CommandScheduler.getInstance().run();
    }

    private void shootPattern(MotifGetter.Motif motif) {
        
        // Firing the artifacts we have, using the motif from the april tag
        int motifIndex = -1;
        boolean hasFiredPurple = false;


        shootingLoop:
        for(final ArtifactColor color : motif) {
            motifIndex++;
            runUntilCompleted(shooter.chargeCommand());

            // if(shooter.getStatus() != Status.EMPTY_CHARGED && shooter.getStatus() != Status.RELOADED_CHARGED) {
            //     CommandScheduler.getInstance().reset();
            //     requestOpModeStop();
            // }

            if(motifIndex == 0) {
                sleep(500); // AWait for correct power
            }


            // Firing the indicated color
            switch(color) {
                case GREEN: 
                    shooter.fireGreen();
                    break;
                case PURPLE:
                    hasFiredPurple = true;
                    shooter.firePurple();
                    break;
                default:
                    throw new RuntimeException("Encountered unfirable ArtifactColor: " + color.name());
            }

            // Waiting for the firing to end
            // The shooter is likely to charge after this, but we want to wait until after reloading
            // to do any extra charging (for saving time).
            runUntilCompleted(new WaitUntilCommand(() -> shooter.getStatus() != Status.FIRING));

            if(motifIndex == 2) {
                break shootingLoop;
            }

            // Getting ready for reloading by cycling the next artifact into position
            // and taking note of what colors are already reloaded.
            ArtifactColor rightColor = null;
            ArtifactColor leftColor = null;

            if(hasFiredPurple) {
                intake.intakeGamePieces();
            }

            for(
                int retries = 0; 
                hasFiredPurple && retries < 3 
                    && (rightColor = rightReload.getColor()) != PURPLE 
                    && (leftColor = leftReload.getColor()) != PURPLE; 
                retries++
            ) {
                // Reload both sides if both are empty
                // We do this to ensure *something* is reloaded
                if(hasFiredPurple && leftColor == ArtifactColor.UNKNOWN && rightColor == ArtifactColor.UNKNOWN) {
                    shooter.reload();
                }

                // Reload the left if it is empty and the other is green
                if(hasFiredPurple && leftColor == ArtifactColor.UNKNOWN && rightColor == ArtifactColor.GREEN ) {
                    shooter.reloadLeft();
                }
                
                // Reload the right if it is empty and the other is green
                if(hasFiredPurple && leftColor == ArtifactColor.GREEN && rightColor == ArtifactColor.UNKNOWN) {
                    shooter.reloadRight();
                }

                // Wait for the shooter to finish reloading and become charged again
                // Reloading naturally will cause the shooter to charge again, so this 
                // covers in case enough shooter velocity was lost when shooting
                runUntilCompleted(new WaitUntilCommand(() -> shooter.getStatus() != Status.CHARGING
                        && shooter.getStatus() != Status.RELOADING));
            }

            // If the charging failed, just tell it that it is charged, and move on
            if(shooter.getStatus() == Status.UNCHARGING) {
                shooter.forceCharged();
            }
        } 
    }
    
    private final TimeInjectionUtil timeUtil = new TimeInjectionUtil(this);

    
    public static final double DIST_TOLERANCE = 0.5; // inches
    public static final double MIN_ANGLE = Robot.positionToRadians(0);
    public static final double MAX_ANGLE = Math.toRadians(62.5); // Any higher, and the shooting is inaccurate
    public static final double MAX_SPEED = Robot.ticksToInches(2400);

    private boolean filterArc(BallisticArc arc) {
        final double theta = Criterion.ANGLE.of(arc);
        
        // Filter based off angle
        if(MIN_ANGLE <= theta && theta <= MAX_ANGLE) {
            return true;
        }

        final double speed = Criterion.SPEED.of(arc);
        return speed <= MAX_SPEED;
    }

}
