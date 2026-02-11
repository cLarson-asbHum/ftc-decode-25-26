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

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.TimeUnit;

import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.hardware.ArtifactColorRangeSensor;
import org.firstinspires.ftc.teamcode.hardware.MotifLimelight;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.subsystem.BasicMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.subsystem.BlockerSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystem.CarwashIntake;
import org.firstinspires.ftc.teamcode.hardware.subsystem.FlywheelTubeShooter;
import org.firstinspires.ftc.teamcode.hardware.subsystem.LinearHingePivot;
import org.firstinspires.ftc.teamcode.hardware.subsystem.ShooterSubsystem.Status;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.teleop.ClearCommandScheduler;
import org.firstinspires.ftc.teamcode.temp.TimeInjectionUtil;
import org.firstinspires.ftc.teamcode.util.ArtifactColor;
import org.firstinspires.ftc.teamcode.util.ConfigPose;
import org.firstinspires.ftc.teamcode.util.KeyPoses;
import org.firstinspires.ftc.teamcode.util.MotifGetter;
import org.firstinspires.ftc.teamcode.util.MotifGetter.Motif;
import org.firstinspires.ftc.teamcode.util.OpModeData;
// import org.firstinspires.ftc.teamcode.util.RrCoordinates;
import org.firstinspires.ftc.teamcode.util.Util;
import org.firstinspires.ftc.teamcode.util.WrapConcurrentCommand;
import org.firstinspires.ftc.vision.VisionPortal;

import static org.firstinspires.ftc.teamcode.util.ArtifactColor.PURPLE;


@Configurable
@Autonomous(name = "ColorBlind Auto2: Blue Rippley", group = "A - Main")
public class RippleyColorBlind extends LinearOpMode {
    private ElapsedTime timer = new ElapsedTime();
    private DcMotor backRight = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft = null;
    private DcMotor frontLeft = null;
    private IMU imu = null;
    private double startDeer = 0;

    public static int GAIN = 50;
    public static int EXPOSURE_MS = 1;

    // TODO: find the robot width and length
    public static double ROBOT_LENGTH = 17; // Inches parallel to the robot's forward-facing axis
    public static double ROBOT_WIDTH = 17; // Inches perpendicular to the robot's forward-facing axis 
    public static double ROBOT_RADIUS = 7;

    public static double CAMERA_YAW_OFFSET = 0; // In radians

    public static ConfigPose START_POS = new ConfigPose(
        // In Inches. Resting flat against the blue goal
        20,

        // In Inches. Is along the top-most grid edge
        122,

        // In Radians. Along the blue goal, facing the upper wall
        // Determined emperically
        Math.toRadians(52)
    );

    public static ConfigPose SHOOTING_POS = new ConfigPose(KeyPoses.Blue.SHOOTING);

    public static ConfigPose OBELISK = new ConfigPose(
        72,
        144, 
        -Math.PI / 2
    );

    public static final double SECOND_SHOT_SPEED = 220;
    public static final double THIRD_SHOT_SPEED = Robot.ticksToInches(1600);
    
    private ArtifactColorRangeSensor rightReload = null;
    private ArtifactColorRangeSensor leftReload = null;

    private FlywheelTubeShooter shooter = null;
    private CarwashIntake intake = null;
    private BasicMecanumDrive drivetrain = null;
    private BlockerSubsystem leftBlocker = null;
    private BlockerSubsystem rightBlocker = null;
    private CRServo duckSpinner = null;

    private boolean isRed = false;
    
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
            return new Pose(72 - (pose.getX() - 72), pose.getY(), Math.PI - pose.getHeading());
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
        result.put("goFromCameraToShooting", follower // #region
            .pathBuilder()
            .addPath(
                new BezierLine(start, shooting)
            )
            .setLinearHeadingInterpolation(start.getHeading(), shooting.getHeading())
            .build()
        ); //#endregion

        final Pose firstGrabEndPose = mirror(new Pose(24, 89.500), isRed);
        final PathChain grabArtifacts = follower //#region
            .pathBuilder()
            .addPath(new BezierCurve(
                () -> follower.getPose(),
                mirror(new Pose(75.038, 82.500), isRed),
                mirror(new Pose(58.489, 82.500), isRed),
                mirror(new Pose(54.089, 82.500), isRed)
            ))
            .setLinearHeadingInterpolation(shooting.getHeading(), grabHeading)
            .addPath(new BezierLine(
                () -> follower.getPose(),
                mirror(new Pose(42.089, 82.500), isRed)
            ))
            .setConstantHeadingInterpolation(grabHeading)
            .addPath(new BezierCurve(
                () -> follower.getPose(),
                mirror(new Pose(33.000, 89.500), isRed),
                mirror(new Pose(31.000, 89.500), isRed),
                firstGrabEndPose
            ))
            .setConstantHeadingInterpolation(grabHeading)
            .build(); //#endregion
            
        final Path openGateAndGoToShooting = new Path(new BezierCurve( //#region
            // Swooping Bezier Form
            () -> follower.getPose(),
            mirror(new Pose( 5.641, 52.733), isRed),
            mirror(new Pose(46.082, 72.141), isRed),
            shooting
        )); // #endregion

        final Path openGate = new Path(new BezierCurve( //#region
            // Little Tap form
            () -> follower.getPose(),
            mirror(new Pose(21.315, 4 + 74.207), isRed),
            mirror(new Pose(16.133, 4 + 73.035), isRed)
        )); //#endregion

        // final Pose secondGrabStart = mirror(new Pose(43.839, 61.500), isRed);
        final Pose secondGrabStart = mirror(new Pose(50.839, 64.500), isRed);
        final Pose secondShooting = minTravelDist( // #region
            new BezierLine(
                mirror(new Pose(    -ROBOT_RADIUS * Math.sqrt(0.5), 144 - ROBOT_RADIUS * Math.sqrt(0.5)), isRed), 
                mirror(new Pose(62 - ROBOT_RADIUS * Math.sqrt(0.5),  82 - ROBOT_RADIUS * Math.sqrt(0.5)), isRed)
            ),
            firstGrabEndPose, 
            secondGrabStart
        ); //#endregion

        final Path goBackToShoot = new Path(new BezierLine( //#region
            () -> follower.getPose(),
            secondShooting
        )); //#endregion
        final Path goBackToShootAfterGate = new Path(new BezierLine( //#region
            () -> follower.getPose(),
            secondShooting
        )); //#endregion
        final PathChain grabArtifactsAgain =  follower //#region
            .pathBuilder()
            .addPath(new BezierCurve(
                () -> follower.getPose(),
                secondGrabStart,
                mirror(new Pose(48.611, 61.500), isRed),
                // mirror(new Pose(48.611, 67.675), isRed),
                secondGrabStart
            ))
            .setLinearHeadingInterpolation(shooting.getHeading(), grabHeading)
            .addPath(new BezierLine(
                () -> follower.getPose(),
                mirror(new Pose(40.000, 61.500), isRed)
            ))
            .setConstantHeadingInterpolation(grabHeading)
            .addPath(new BezierCurve(
                () -> follower.getPose(),
                mirror(new Pose(33.000, 56.500), isRed),
                mirror(new Pose(31.000, 56.500), isRed),
                mirror(new Pose(14,     56.500), isRed)
            ))
            .setConstantHeadingInterpolation(grabHeading)
            .build(); //#endregion

        final Pose parkPose = mirror(new Pose(48, 60, shooting.getHeading()), isRed);
        final Pose avoidGatePose = mirror(new Pose(26, 59.500), isRed);
        final Pose thirdShooting = minTravelDist( //#region
            new BezierLine(
                mirror(new Pose(    -ROBOT_RADIUS * Math.sqrt(0.5), 144 - ROBOT_RADIUS * Math.sqrt(0.5)), isRed), 
                mirror(new Pose(62 - ROBOT_RADIUS * Math.sqrt(0.5),  82 - ROBOT_RADIUS * Math.sqrt(0.5)), isRed)
            ),
            avoidGatePose, 
            parkPose
        ); //#endregion
        
        final PathChain goBackToShootAgain = follower.pathBuilder() //#region
            .addPath(new Path(new BezierLine(
                () -> follower.getPose(),
                avoidGatePose
            )))
            .addPath(new Path(new BezierLine(
                () -> follower.getPose(),
                thirdShooting
            )))
            .build(); //#endregion

        grabArtifacts.getPath(0).setLinearHeadingInterpolation(shooting.getHeading(), grabHeading);
        grabArtifacts.getPath(1).setConstantHeadingInterpolation(grabHeading);
        grabArtifacts.getPath(2).setConstantHeadingInterpolation(grabHeading);
        grabArtifactsAgain.getPath(0).setLinearHeadingInterpolation(shooting.getHeading(), grabHeading);
        grabArtifactsAgain.getPath(1).setConstantHeadingInterpolation(grabHeading);
        grabArtifactsAgain.getPath(2).setConstantHeadingInterpolation(grabHeading);
        openGateAndGoToShooting.setLinearHeadingInterpolation(grabHeading, shooting.getHeading());
        openGate.setConstantHeadingInterpolation(Math.toRadians(90));
        goBackToShoot.setLinearHeadingInterpolation(grabHeading, shooting.getHeading());
        goBackToShootAfterGate.setLinearHeadingInterpolation(Math.toRadians(90), shooting.getHeading());
        goBackToShootAgain.getPath(0).setConstantHeadingInterpolation(grabHeading);
        goBackToShootAgain.getPath(1).setLinearHeadingInterpolation(grabHeading, shooting.getHeading());

        result.put("grabArtifactsAndShoot", follower //#region
            .pathBuilder()
            .addPath(grabArtifacts.getPath(0))
            .addPath(grabArtifacts.getPath(1))
            .addPath(grabArtifacts.getPath(2))
            .addPath(goBackToShoot)
            .build()
        ); //#endregion
        
        result.put("grabArtifactsOpenGateAndShoot", follower //#region
            .pathBuilder()
            .addPath(grabArtifacts.getPath(0))
            .addPath(grabArtifacts.getPath(1))
            .addPath(grabArtifacts.getPath(2))
            // .addPath(openGateAndGoToShooting)
            .addPath(openGate)
            .addPath(goBackToShootAfterGate)
            .build()
        ); //#endregion

        result.put("grabArtifactsAndShootAgain", follower //#region
            .pathBuilder()
            .addPath(grabArtifactsAgain.getPath(0))
            .addPath(grabArtifactsAgain.getPath(1))
            .addPath(grabArtifactsAgain.getPath(2))
            .addPath(goBackToShootAgain.getPath(0))
            .addPath(goBackToShootAgain.getPath(1))
            .build()
        ); //#endregion

        result.put("park", follower //#region
            .pathBuilder()
            .addPath(new BezierLine(
                () -> follower.getPose(), 
                parkPose
            ))
            .setConstantHeadingInterpolation(shooting.getHeading())
            .build()
        ); //#endregion

        return result;
    }

    @Override
    public void runOpMode() {
        telemetry.setMsTransmissionInterval(30);

        // Creating subsystems
        final Robot robot = new Robot(hardwareMap, java.util.Set.of(
            Robot.Device.SHOOTER,
            Robot.Device.INTAKE,
            Robot.Device.LEFT_BLOCKER,
            Robot.Device.RIGHT_BLOCKER,
            Robot.Device.RAMP_PIVOT,
            Robot.Device.LEFT_RELOAD,
            Robot.Device.RIGHT_RELOAD,
            Robot.Device.MOTIF_LIMELIGHT,
            Robot.Device.ULTIMATE_POINT_EARNER
        )); 
        shooter      = robot.getShooter();
        intake       = robot.getIntake();
        leftBlocker  = robot.getLeftBlocker();
        rightBlocker = robot.getRightBlocker();
        leftReload   = robot.getLeftReload();
        rightReload  = robot.getRightReload();
        duckSpinner  = robot.getDuckSpinner(); // Trust me, this serves a purpose... it's an indicator for the motif
        final LinearHingePivot rampPivot = robot.getRampPivot();
        final MotifLimelight motifGetter = robot.getMotifLimelight();
        // CommandScheduler.getInstance().registerSubsystem(robot.getAllSubsystems());
        CommandScheduler.getInstance().registerSubsystem(shooter,intake,leftBlocker,rightBlocker,rampPivot);
        shooter.setTelemetry(telemetry);

        // Bulk caching
        final List<LynxModule> modules = hardwareMap.getAll(LynxModule.class);

        for(final LynxModule module : modules) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // Creating paths
        final Follower follower = Constants.createFollower(hardwareMap);
        Map<String, PathChain> paths = createPaths(follower, isRed);
        OpModeData.follower = follower;
        
        // Init loop
        boolean inCompetitonMode = OpModeData.inCompetitonMode;
        boolean openGate = true;
        while(opModeInInit()) {
            OpModeData.isRed = isRed;
            OpModeData.inCompetitonMode = inCompetitonMode;
            shooter.setTelemetry(inCompetitonMode ? null : telemetry);

            telemetry.addData("Status", "Initialized");
            telemetry.addLine();
            telemetry.addData("Left artifact",  nullSafeColor(leftReload));
            telemetry.addData("Right artifact", nullSafeColor(rightReload));
            telemetry.addLine();
            telemetry.addLine(Util.header("Settings"));
            telemetry.addLine();
            telemetry.addData("Toggle isRed", "A");
            telemetry.addData("isRed", isRed);
            telemetry.addLine();
            telemetry.addData("Toggle competiton mode", "Y");
            telemetry.addData("Competiton mode", OpModeData.inCompetitonMode);
            telemetry.addLine();
            telemetry.addData("Toggle openGate", "X");
            telemetry.addData("openGate", openGate);
            telemetry.update();

            if(gamepad1.aWasPressed()) {
                isRed = !isRed;
                paths = createPaths(follower, isRed);
            }      

            if(gamepad1.yWasPressed()) {
                inCompetitonMode = !inCompetitonMode;
            }
            
            if(gamepad1.xWasPressed()) {
                openGate = !openGate;
            }
        }
        
        waitForStart();
        leftBlocker.close();
        rightBlocker.close();
        rampPivot.runToAngle(Math.toRadians(61.6));
        follower.setPose(mirror(START_POS.pedroPose(), isRed));

        // Get the motif 
        final boolean cameraExists = motifGetter != null;
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
            OpModeData.startPosition = follower.getPose();

            if(cameraExists && motif == null) {
                motif = captureMotif(motifGetter, follower);
            }
        }
        
        if(!openGate) {
            shootPattern(motif);
        } else {
            // We know that this clip will be emptied, so the pattern doesn't matter
            emptyClip();
        }
        shooter.charge(SECOND_SHOT_SPEED, false);

        // Moving to grab artifacts
        // This goes back to shooting afterwards
        intake.intakeGamePieces();
        if(openGate) {
            follower.followPath(paths.get("grabArtifactsOpenGateAndShoot"), false);
        } else {
            follower.followPath(paths.get("grabArtifactsAndShoot"), false);
        }

        boolean hasReloaded = false;
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

            if(cameraExists && motif == null) {
                motif = captureMotif(motifGetter, follower);
            }
        }
        follower.setMaxPower(1.0);

        // Shooting once again
        shootPattern(SECOND_SHOT_SPEED, motif);
        shooter.charge(THIRD_SHOT_SPEED, false);

        // Moving to grab artifacts
        // This goes back to shooting afterwards
        intake.intakeGamePieces();
        follower.followPath(paths.get("grabArtifactsAndShootAgain"), false);

        // hasReloaded = false;
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
            if(cameraExists && motif == null) {
                motif = captureMotif(motifGetter, follower);
            }
        }
        follower.setMaxPower(1.0);

        // Shooting once again
        // rampPivot.runToAngle(Math.toRadians(56));
        shootPattern(THIRD_SHOT_SPEED, motif);

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

    private void emptyClip(double inchesPerSec) {
        // runUntilCompleted(shooter.chargeCommand());
        runUntilCompleted(WrapConcurrentCommand.wrapUntilNotState(
            shooter,
            () -> shooter.charge(inchesPerSec, true),
            FlywheelTubeShooter.Status.CHARGING
        ));
        leftBlocker.open();
        rightBlocker.open();
        CommandScheduler.getInstance().run();
        sleep(500);
        final ElapsedTime timer = new ElapsedTime(); // FIXME: timeUtil

        // Shooting depth 1
        // runUntilCompleted(shooter.chargeCommand());
        runUntilCompleted(WrapConcurrentCommand.wrapUntilNotState(
            shooter,
            () -> shooter.charge(inchesPerSec, true),
            FlywheelTubeShooter.Status.CHARGING
        ));
        // runUntilCompleted(shooter.fireCommand());

        // Reloading and going
        // runUntilCompleted(shooter.chargeCommand());
        
        // Shooting
        intake.intakeGamePieces();
        runUntilCompleted(shooter.fireCommand());
        runUntilCompleted(shooter.fireCommand());

        // Ending
        intake.holdGamePieces();
        leftBlocker.close();
        rightBlocker.close();
        shooter.charge();
        CommandScheduler.getInstance().run();
    }
    
    private void emptyClip() {
        runUntilCompleted(shooter.chargeCommand());
        leftBlocker.open();
        rightBlocker.open();
        CommandScheduler.getInstance().run();
        sleep(500);
        final ElapsedTime timer = new ElapsedTime(); // FIXME: timeUtil

        // Shooting depth 1
        runUntilCompleted(shooter.chargeCommand());
        // runUntilCompleted(shooter.fireCommand());

        // Reloading and going
        // runUntilCompleted(shooter.chargeCommand());
        
        // Shooting
        intake.intakeGamePieces();
        runUntilCompleted(shooter.fireCommand());
        runUntilCompleted(shooter.fireCommand());

        // Ending
        intake.holdGamePieces();
        leftBlocker.close();
        rightBlocker.close();
        shooter.charge();
        CommandScheduler.getInstance().run();
    }
    
    private void shootPattern(double inchesPerSec, MotifGetter.Motif motif) {
        if(motif == null) {
            emptyClip(inchesPerSec);
            return;
        }

        // Firing the artifacts we have, using the motif from the april tag
        int motifIndex = -1;
        boolean hasFiredPurple = false;
        
        shootingLoop:
        for(final ArtifactColor color : motif) {
            motifIndex++;

            // Reloading any empty side
            // Skip if this is index 0.
            if(motifIndex != 0) {
                intake.intakeGamePieces();
                shooter.reloadEmpty();
                // closeBlockers(shooter.getReloadingState());
                runUntilCompleted(new WaitUntilCommand(() -> shooter.getStatus() != Status.RELOADING));
            }
            
            // Sending the commands to fire the correct color
            intake.holdGamePieces();
            final boolean hadCorrectColor = fireColor(color);

            // If the color could not be loaded, give up trying to fire the pattern
            // Rather than trying to reload, we just assume that it is reloaded
            if(!hadCorrectColor && motifIndex == 0) {
                // Because we still have all three artifacts, give up and shoot everything
                emptyClip(inchesPerSec);
                return;
            } else if(!hadCorrectColor && motifIndex > 0) {
                shooter.fire();
            }

            // Letting firing finish
            openBlockers(shooter.getFiringState());
            CommandScheduler.getInstance().run();
            sleep(800);

            if(motifIndex == 2 || !hadCorrectColor) {
                return;
            }

            // Recharging as necessary
            if(shooter.getStatus() == Status.UNCHARGING) {
                shooter.charge(inchesPerSec, true);
            }
        } 
    }

    private void shootPattern(MotifGetter.Motif motif) {
        if(motif == null) {
            emptyClip();
            return;
        }

        // Firing the artifacts we have, using the motif from the april tag
        int motifIndex = -1;
        boolean hasFiredPurple = false;
        
        shootingLoop:
        for(final ArtifactColor color : motif) {
            motifIndex++;

            // Reloading any empty side
            // Skip if this is index 0.
            if(motifIndex != 0) {
                intake.intakeGamePieces();
                shooter.reloadEmpty();
                // closeBlockers(shooter.getReloadingState());
                runUntilCompleted(new WaitUntilCommand(() -> shooter.getStatus() != Status.RELOADING));
            }
            
            // Sending the commands to fire the correct color
            intake.holdGamePieces();
            final boolean hadCorrectColor = fireColor(color);

            // If the color could not be loaded, give up trying to fire the pattern
            // Rather than trying to reload, we just assume that it is reloaded
            if(!hadCorrectColor && motifIndex == 0) {
                // Because we still have all three artifacts, give up and shoot everything
                emptyClip();
                return;
            } else if(!hadCorrectColor && motifIndex > 0) {
                shooter.fire();
            }

            // Letting firing finish
            openBlockers(shooter.getFiringState());
            CommandScheduler.getInstance().run();
            sleep(800);

            if(motifIndex == 2 || !hadCorrectColor) {
                return;
            }

            // Recharging as necessary
            if(shooter.getStatus() == Status.UNCHARGING) {
                shooter.charge();
            }
        } 
    }

    private boolean fireColor(ArtifactColor color) {
        switch(color) {
            case GREEN: 
                return shooter.fireGreen();
            case PURPLE:
                return shooter.firePurple();
            default:
                throw new RuntimeException("Encountered unfirable ArtifactColor: " + color.name());
        }
    }

    private boolean openBlockers(FlywheelTubeShooter.FiringState firingState) {
        switch(firingState) {
            case FIRING_BOTH:
                leftBlocker.open();
                rightBlocker.open();
                return true;

            case FIRING_LEFT:
                leftBlocker.open();
                rightBlocker.close();
                return true;

            case FIRING_RIGHT:
                leftBlocker.close();
                rightBlocker.open();
                return true;

            default:
                return false;
        }
    }

    private boolean closeBlockers(FlywheelTubeShooter.ReloadingState reloadingState) {
        switch(reloadingState) {
            case RELOADING_BOTH:
                leftBlocker.close();
                rightBlocker.close();
                return true;

            case RELOADING_LEFT:
                leftBlocker.close();
                rightBlocker.open();
                return true;

            case RELOADING_RIGHT:
                leftBlocker.open();
                rightBlocker.close();
                return true;

            default:
                return false;
        }
    }

    /**
     * Calculates the point on the given segment that minimizes the total distance
     * between itself and the two points. This can be thought of the point on the
     * line that will have the shortest travel time, starting from the one point,
     * then going to the line, then to the other point.
     * 
     * The returned point is guarnateed to be on the segment.
     * 
     * @param segment The set of points the return value must be located on.
     * @param p One point to travel to. Heading is ignored.
     * @param q Another point to travel to. Heading is ignored
     * @return The point which minimizes the travel distance between the points.
     * The heading of this is undefined and should be ignored.
     */
    public static Pose minTravelDist(BezierLine segment, Pose p, Pose q) {
        final Pose start = segment.getPose(0);
        final Pose end = segment.getPose(1);

        // Transforming the points assuming that the start (point A) is 0
        // final Pose vecA = new Pose(0, 0);
        final Pose vecB = end.minus(start);
        final Pose vecP =   p.minus(start);
        final Pose vecQ =   q.minus(start);

        // Getting some essential coefficients from the translated points
        final double a = vecB.getX() * vecB.getX() + vecB.getY() * vecB.getY(); // b * b
        final double b = vecP.getX() * vecB.getX() + vecP.getY() * vecB.getY(); // p * b
        final double c = vecP.getX() * vecP.getX() + vecP.getY() * vecP.getY(); // p * p
        final double d = vecQ.getX() * vecB.getX() + vecQ.getY() * vecB.getY(); // q * b
        final double f = vecQ.getX() * vecQ.getX() + vecQ.getY() * vecQ.getY(); // q * q

        // Getting the two possible solutions
        // This uses the quadratic formula
        final double quad = a*a*f - a*a*c + a*b*b - a*d*d;
        final double line = -2 * (a*b*f - a*c*d + b*b*d - b*d*d);
        final double cons = b*b*f - c*d*d;

        if(quad == 0) {
            // TODO: The solution technicaly exists in this case, but the math above craps out
            throw new RuntimeException("The segment was parallel with the line containing p and q");
        }

        final double t1 = (-line + Math.sqrt(line * line - 4 * quad * cons)) / (2 * quad);
        final double t2 = (-line - Math.sqrt(line * line - 4 * quad * cons)) / (2 * quad);

        // Returning the solution that minimizes the travel distance
        final Pose s1 = segment.getPose(Util.clamp(0, t1, 1));
        final Pose s2 = segment.getPose(Util.clamp(0, t2, 1));

        if(s1.distanceFrom(p) + s1.distanceFrom(q) <= s2.distanceFrom(p) + s2.distanceFrom(q)) {
            return s1;
        } else {
            return s2;
        }
    }
    
    private ArtifactColor nullSafeColor(ArtifactColorRangeSensor nullableSensor) {
        if(nullableSensor == null) {
            return null;
        }

        return nullableSensor.getColor();
    }
    
    
    private final TimeInjectionUtil timeUtil = new TimeInjectionUtil(this);

    private Motif captureMotif(MotifLimelight motifGetter, Follower follower) {
        // Snapping a photo of the motif if we are facing it
        // The camera sees 60 degrees, but we subtract a bit to fully see the motif
        final double F_O_V = Math.toRadians(40); 
        final Pose currentPose = follower.getPose();
        final double targetAngle = Math.atan2(
            OBELISK.pedroPose().getY() - currentPose.getY(), 
            OBELISK.pedroPose().getX() - currentPose.getX()
        );

        if(Util.near(currentPose.getHeading(), targetAngle, 0.5 * F_O_V)) { 
            motifGetter.setGlobalRobotYaw(currentPose.getHeading());
            final Motif result = motifGetter.getMotif();
            duckSpinner.setPower(1.0);
            motifGetter.disable(); // Save bandwidth and performance by not accessing the camera
            return result;
        }

        return null;
    }
}
