package org.firstinspires.ftc.teamcode.auto;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierPoint;
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

    public static double CAMERA_YAW_OFFSET = 0; // In radians

    public static double SHOT_SPEED = 340; // Determined using the ballistic arc text user interface
    public static double SHOT_ANGLE = Math.toRadians(43); // Determined using the ballistic arc text user interface

    public static ConfigPose START_POS = new ConfigPose(
        56,
        10,

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
    private boolean inCompetitonMode = OpModeData.inCompetitonMode;
    
    private ArrayList<String> nullDeviceNames = new ArrayList<>();
    private ArrayList<Class<?>> nullDeviceTypes = new ArrayList<>();



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
        final double grabHeading = isRed ? 0 : -Math.PI;

        // Creating the paths
        result.put("goFromCameraToShooting", follower
            .pathBuilder()
            .addPath(
                new BezierLine(start, shooting)
            )
            .setLinearHeadingInterpolation(start.getHeading(), shooting.getHeading())
            .build()
        );

        final PathChain grabArtifactsAgain = follower
            .pathBuilder()
            .addPath(new BezierCurve(
                () -> follower.getPose(),
                mirror(new Pose(75.038, 34.500), isRed),
                mirror(new Pose(58.489, 34.500), isRed),
                mirror(new Pose(54.089, 34.500), isRed)
            ))
            .setLinearHeadingInterpolation(shooting.getHeading(), grabHeading)
            .addPath(new BezierLine(
                () -> follower.getPose(),
                mirror(new Pose(42.089, 34.500), isRed)
            ))
            .setConstantHeadingInterpolation(grabHeading)
            .addPath(new BezierCurve(
                () -> follower.getPose(),
                mirror(new Pose(33.000, 41.500), isRed),
                mirror(new Pose(31.000, 41.500), isRed),
                mirror(new Pose(24.000, 41.500), isRed)
            ))
            .setConstantHeadingInterpolation(grabHeading)
            .build();
        
        final Path goBackToShootAgain = new Path(new BezierLine(
            () -> follower.getPose(),
            shooting
        ));

        grabArtifactsAgain.getPath(0).setLinearHeadingInterpolation(shooting.getHeading(), grabHeading);
        grabArtifactsAgain.getPath(1).setConstantHeadingInterpolation(grabHeading);
        grabArtifactsAgain.getPath(2).setConstantHeadingInterpolation(grabHeading);
        goBackToShootAgain.setConstantHeadingInterpolation(shooting.getHeading());

        result.put("grabLoadingZoneArtifacts", follower
            .pathBuilder()
            .addPath(new BezierLine(
                () -> follower.getPose(),
                mirror(new Pose(11, 7.75, grabHeading), isRed)
            ))
            // .setConstantHeadingInterpolation(grabHeading)
            .setLinearHeadingInterpolation(shooting.getHeading(), grabHeading)
            .build()
        );

        result.put("backToShooting", follower
            .pathBuilder()
            .addPath(new BezierLine(
                () -> follower.getPose(),
                shooting
            ))
            .setLinearHeadingInterpolation(grabHeading, shooting.getHeading())
            // .setConstantHeadingInterpolation(shooting.getHeading())

            .build()
        );

        result.put("grabArtifactsAndShootAgain", follower
            .pathBuilder()
            .addPath(grabArtifactsAgain.getPath(0))
            .addPath(grabArtifactsAgain.getPath(1))
            .addPath(grabArtifactsAgain.getPath(2))
            .addPath(goBackToShootAgain)
            .build()
        );

        result.put("park", follower
            .pathBuilder()
            .addPath(new BezierLine(
                () -> follower.getPose(),
                mirror(new Pose(11, 9, grabHeading), isRed)
            ))
            // .setConstantHeadingInterpolation(grabHeading)
            .setLinearHeadingInterpolation(shooting.getHeading(), grabHeading)
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
        CommandScheduler.getInstance().registerSubsystem(shooter, intake, leftBlocker, rightBlocker, rampPivot);
        
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
        boolean grabThirdSpike = true;
        while(opModeInInit()) {
            if(aimbot.isInitialized()) {
                OpModeData.selection = aimbot.getSelection();
                OpModeData.isRed = isRed;
                OpModeData.inCompetitonMode = inCompetitonMode;
                shooter.setTelemetry(inCompetitonMode ? null : telemetry);

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
                telemetry.addLine();
                telemetry.addData("Toggle grabThirdSpike", "X");
                telemetry.addData("grabThirdSpike", grabThirdSpike);
                telemetry.update();

                if(gamepad1.aWasPressed()) {
                    isRed = !isRed;
                    paths = createPaths(follower, isRed);
                }
                
                if(gamepad1.yWasPressed()) {
                    inCompetitonMode = !inCompetitonMode;
                }
                
                if(gamepad1.xWasPressed()) {
                    grabThirdSpike = !grabThirdSpike;
                }
            }    
        }
        
        waitForStart();
        rampPivot.runToAngle(SHOT_ANGLE);
        follower.setPose(mirror(START_POS.pedroPose(), isRed));

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
        
        final Pose shooting = mirror(SHOOTING_POS.pedroPose(), isRed);
        while(opModeIsActive() && !(
            follower.atPose(shooting, 0.5, 0.5) 
            && Util.anglesNear(follower.getPose().getHeading(), shooting.getHeading(), Math.toRadians(0.85))
        )) {
            follower.holdPoint(new BezierPoint(shooting), shooting.getHeading());
            follower.update();
            OpModeData.startPosition = follower.getPose();
            CommandScheduler.getInstance().run();
        }

        // If the motif coul dnt be found, use a defa`ult
        // if(motif == null && allPurple) {
        //     motif = Motif.ALL_PURPLE;
        /* }  else */ if(motif == null) {
            motif = Motif.FIRST_GREEN;
        }

        emptyClip(motif);

        // Grabbing the artifacts from the oponent's loading zone
        intake.intakeGamePieces();
        follower.followPath(paths.get("grabLoadingZoneArtifacts"), false);

        follower.setMaxPower(0.8);
        while(follower.isBusy() && opModeIsActive()) {
            follower.update();
            OpModeData.startPosition = follower.getPose();
            CommandScheduler.getInstance().run();
        }
        follower.setMaxPower(1.0);

        // Going again an shooting
        follower.followPath(paths.get("backToShooting"), false);
        intake.holdGamePieces();
        
        // hasReloaded = false;
        while(follower.isBusy() && opModeIsActive()) {
            follower.update();
            OpModeData.startPosition = follower.getPose();
            CommandScheduler.getInstance().run();
        }

        while(opModeIsActive()  && !(
            follower.atPose(shooting, 0.5, 0.5) 
            && Util.anglesNear(follower.getPose().getHeading(), shooting.getHeading(), Math.toRadians(0.85))
        )) {
            follower.holdPoint(new BezierPoint(shooting), shooting.getHeading());
            follower.update();
            OpModeData.startPosition = follower.getPose();
            CommandScheduler.getInstance().run();
        }

        emptyClip(motif);

        // Grabbing the third line of artifacts
        // This goes back to shooting afterwards
        if(grabThirdSpike) {
            intake.intakeGamePieces();
            follower.followPath(paths.get("grabArtifactsAndShootAgain"), false);

            leftBlocker.close();
            rightBlocker.close();
            while(follower.isBusy() && opModeIsActive()) {
                if(follower.getChainIndex() == 1 || follower.getChainIndex() == 2) {
                    follower.setMaxPower(0.4);
                    intake.intakeGamePieces();
                    shooter.reload();
                } else {
                    follower.setMaxPower(1.0);
                    intake.holdGamePieces();
                }

                follower.update();
                OpModeData.startPosition = follower.getPose();
                CommandScheduler.getInstance().run();
            }
            follower.setMaxPower(1.0);
            
            while(opModeIsActive()  && !(
                follower.atPose(shooting, 0.5, 0.5) 
                && Util.anglesNear(follower.getPose().getHeading(), shooting.getHeading(), Math.toRadians(0.85))
            )) {
                follower.holdPoint(new BezierPoint(shooting), shooting.getHeading());
                follower.update();
                OpModeData.startPosition = follower.getPose();
                CommandScheduler.getInstance().run();
            }

            // Shooting once again
            emptyClip(motif);
        }
        
        // Going and parking
        intake.intakeGamePieces();
        follower.followPath(paths.get("park"), false);

        while(follower.isBusy() && opModeIsActive()) {
            follower.update();
            OpModeData.startPosition = follower.getPose();
            CommandScheduler.getInstance().run();
        }

        // END
        shooter.uncharge();
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

        leftBlocker.open();
        rightBlocker.open();
        CommandScheduler.getInstance().run();
        // sleep(500);

        // Reloading and going
        // runUntilCompleted(new WrapConcurrentCommand<ShooterSubsystem.Status>(
        //     shooter,
        //     () -> shooter.charge(SHOT_SPEED, true),
        //     FlywheelTubeShooter.Status.CHARGED
        // ));
        
        // Shooting
        intake.intakeGamePieces();
        shooter.multiFire();
        CommandScheduler.getInstance().run();
        sleep(2500);
        
        // Ending
        shooter.charge(SHOT_SPEED, false);
        leftBlocker.close();
        rightBlocker.close();
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
        if(!(MIN_ANGLE <= theta && theta <= MAX_ANGLE)) {
            return false;
        }

        final double speed = Criterion.SPEED.of(arc);
        return speed <= MAX_SPEED;
    }

}
