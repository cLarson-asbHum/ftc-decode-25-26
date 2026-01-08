package org.firstinspires.ftc.teamcode.auto;

import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

import java.util.ArrayList;
import java.util.Map;
import java.util.HashMap;
import java.util.concurrent.TimeUnit;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystem.BasicMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.BlockerSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.CarwashIntake;
import org.firstinspires.ftc.teamcode.subsystem.FlywheelTubeShooter;
import org.firstinspires.ftc.teamcode.subsystem.ShooterSubsystem.Status;
import org.firstinspires.ftc.teamcode.teleop.ClearCommandScheduler;
import org.firstinspires.ftc.teamcode.temp.TimeInjectionUtil;
import org.firstinspires.ftc.teamcode.util.ArtifactColor;
import org.firstinspires.ftc.teamcode.util.ArtifactColorRangeSensor;
import org.firstinspires.ftc.teamcode.util.ConfigPose;
import org.firstinspires.ftc.teamcode.util.KeyPoses;
import org.firstinspires.ftc.teamcode.util.MotifGetter;
import org.firstinspires.ftc.teamcode.util.MotifGetter.Motif;
import org.firstinspires.ftc.teamcode.util.MotifWebcam;
import org.firstinspires.ftc.teamcode.util.RrCoordinates;
import org.firstinspires.ftc.teamcode.util.Util;
import org.firstinspires.ftc.teamcode.util.WrapConcurrentCommand;

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

    public static double CAMERA_YAW_OFFSET = 0; // In radians

    public static ConfigPose START_POS = new ConfigPose(
        // In Inches. Resting flat against the blue goal
        24,

        // In Inches. Is along the top-most grid edge
        120,

        // In Radians. Along the blue goal, facing the upper wall
        // Determined emperically
        Math.toRadians(45)
    );

    public static ConfigPose SHOOTING_POS = new ConfigPose(KeyPoses.Blue.SHOOTING);

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

    private Subsystem[] createSubsystems(HardwareMap hardwareMap) {
        // Find and create all of the hardware. This uses the hardware map. 
        // When using unit tests, the `hardwareMap` field can be set for dependency injection.
        final DcMotorEx frontLeftMotor  = (DcMotorEx) findHardware(DcMotor.class, "frontLeft"); // Null if not found
        final DcMotorEx backLeftMotor   = (DcMotorEx) findHardware(DcMotor.class, "backLeft"); // Null if not found
        final DcMotorEx frontRightMotor = (DcMotorEx) findHardware(DcMotor.class, "frontRight"); // Null if not found
        final DcMotorEx backRightMotor  = (DcMotorEx) findHardware(DcMotor.class, "backRight"); // Null if not found

        final DcMotorEx rightShooterMotor = (DcMotorEx) findHardware(DcMotor.class, "rightShooter");
        final DcMotorEx leftShooterMotor = (DcMotorEx) findHardware(DcMotor.class, "leftShooter");
        final CRServo rightFeederServo = findHardware(CRServo.class, "rightFeeder");
        final CRServo leftFeederServo = findHardware(CRServo.class, "leftFeeder");
        final ServoImplEx leftBlockerServo = (ServoImplEx) findHardware(Servo.class, "leftBlocker");
        final ServoImplEx rightBlockerServo = (ServoImplEx) findHardware(Servo.class, "rightBlocker");
        final DcMotorEx intakeMotor = (DcMotorEx) findHardware(DcMotor.class, "intake");

        final ColorRangeSensor rightReloadSensor = findHardware(ColorRangeSensor.class, "rightReload");
        final ColorRangeSensor leftReloadSensor = findHardware(ColorRangeSensor.class, "leftReload");
        // Checking that ALL hardware has been found (aka the nullHardware list is empty)
        // If any are not found, an error is thrown stating which.
        throwAFitIfAnyHardwareIsNotFound();

        // Setting all necessary hardware properties
        frontLeftMotor.setDirection(DcMotorEx.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorEx.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorEx.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorEx.Direction.FORWARD);

        rightShooterMotor.setDirection(DcMotor.Direction.REVERSE);
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);
        rightFeederServo.setDirection(DcMotor.Direction.REVERSE);
        leftFeederServo.setDirection(DcMotor.Direction.FORWARD);

        // Creating subsystems. 
        // Subsystems represent groups of hardware that achieve ONE function.
        // Subsystems can lead into each other, but they should be able to operate independently 
        // (even if nothing is achieved, per se).
        rightReload = new ArtifactColorRangeSensor(
            rightReloadSensor,
            new ArtifactColorRangeSensor.AlternateColorSensorConst().asColorSensorConst(), // Use alternate tuning because wierd
            new double[] { 0.400, 0.24, 0.16, 0.12, 0.08  }
        );
        leftReload = new ArtifactColorRangeSensor(
            leftReloadSensor,
            new ArtifactColorRangeSensor.ColorSensorConst(), // USe the default tuning
            new double[] { 0.400, 0.24, 0.16, 0.12, 0.08  }
        );

        final FlywheelTubeShooter rightShooter = new FlywheelTubeShooter.Builder(rightShooterMotor, leftShooterMotor) 
            .setLeftFeeder(leftFeederServo) 
            .setRightFeeder(rightFeederServo)
            .setRightReloadClassifier(rightReload)
            .setLeftReloadClassifier( leftReload)
            .build();
        final CarwashIntake intake = new CarwashIntake(intakeMotor);
        leftBlocker = new BlockerSubsystem(
            leftBlockerServo, 
            BlockerSubsystem.PositionPresets.LEFT
        );
        rightBlocker = new BlockerSubsystem(
            rightBlockerServo, 
            BlockerSubsystem.PositionPresets.RIGHT
        );

        // This means that no command will use the same subsystem at the same time.
        CommandScheduler.getInstance().registerSubsystem(rightShooter, intake, leftBlocker, rightBlocker);

        // Return a list of every subsystem that we have created
        return new Subsystem[] { rightShooter, intake };
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

        // Creating the paths
        result.put("goFromCameraToShooting", follower
            .pathBuilder()
            .addPath(
                // new BezierLine(new Pose(22.017, 119.603), new Pose(40.463, 102.942))
                new BezierLine(start, shooting)
            )
            .setLinearHeadingInterpolation(start.getHeading(), shooting.getHeading())
            .build()
        );

        final PathChain grabArtifacts = follower
            .pathBuilder()
            .addPath(new BezierCurve(
                shooting,
                mirror(new Pose(71.038, 79.500), isRed),
                mirror(new Pose(54.489, 79.500), isRed),
                mirror(new Pose(50.089, 79.500), isRed)
            ))
            .setLinearHeadingInterpolation(shooting.getHeading(), isRed ? 0 : Math.toRadians(-180))
            .addPath(new BezierLine(
                mirror(new Pose(50.089, 79.5), isRed),
                mirror(new Pose(38.089, 79.500), isRed)
            ))
            .setConstantHeadingInterpolation(isRed ? 0 : Math.toRadians(-180))
            .addPath(new BezierCurve(
                mirror(new Pose(38.089, 79.500), isRed),
                mirror(new Pose(29.000, 89.500), isRed),
                mirror(new Pose(27.000, 89.500), isRed),
                mirror(new Pose(21,     89.500), isRed)
            ))
            .setConstantHeadingInterpolation(isRed ? 0 : Math.toRadians(-180))
            .build();
        final Path goBackToShoot = new Path(new BezierLine(
            mirror(new Pose(21,   89.500), isRed), 
            shooting
        ));
        final PathChain grabArtifactsAgain =  follower
            .pathBuilder()
            .addPath(new BezierCurve(
                shooting,
                mirror(new Pose(71.038, 55.500), isRed),
                mirror(new Pose(54.489, 55.500), isRed),
                mirror(new Pose(50.089, 55.500), isRed)
            ))
            .setLinearHeadingInterpolation(shooting.getHeading(), isRed ? 0 : Math.toRadians(-180))
            .addPath(new BezierLine(
                mirror(new Pose(50.089, 55.5), isRed),
                mirror(new Pose(38.089, 55.500), isRed)
            ))
            .setConstantHeadingInterpolation(isRed ? 0 : Math.toRadians(-180))
            .addPath(new BezierCurve(
                mirror(new Pose(38.089, 79.500), isRed),
                mirror(new Pose(29.000, 66.500), isRed),
                mirror(new Pose(27.000, 66.500), isRed),
                mirror(new Pose(18,     66.500), isRed)
            ))
            .setConstantHeadingInterpolation(isRed ? 0 : Math.toRadians(-180))
            .build();
        final Path goBackToShootAgain = new Path(new BezierLine(
            mirror(new Pose(18,     66.500), isRed), 
            shooting
        ));
        final Path turnSoAsToIntake = new Path(new BezierLine(
            shooting,
            new Pose(shooting.getX(), shooting.getY(), isRed ? 0 : Math.PI)
        ));

        // turnSoAsToIntake.setLinearHeadingInterpolation(shooting.getHeading(), isRed ? 0 : Math.PI);
        final double grabHeading = isRed ? 0 : -Math.PI;
        grabArtifacts.getPath(0).setLinearHeadingInterpolation(shooting.getHeading(), grabHeading);
        grabArtifacts.getPath(1).setConstantHeadingInterpolation(grabHeading);
        grabArtifacts.getPath(2).setConstantHeadingInterpolation(grabHeading);
        grabArtifactsAgain.getPath(0).setLinearHeadingInterpolation(shooting.getHeading(), grabHeading);
        grabArtifactsAgain.getPath(1).setConstantHeadingInterpolation(grabHeading);
        grabArtifactsAgain.getPath(2).setConstantHeadingInterpolation(grabHeading);
        goBackToShoot.setConstantHeadingInterpolation(shooting.getHeading());
        goBackToShootAgain.setConstantHeadingInterpolation(shooting.getHeading());

        result.put("grabArtifactsAndShoot", follower
            .pathBuilder()
            // .addPath(turnSoAsToIntake) // FIXME: Tell pedropathing to do this correctly!
            .addPath(grabArtifacts.getPath(0))
            .addPath(grabArtifacts.getPath(1))
            .addPath(grabArtifacts.getPath(2))
            .addPath(goBackToShoot)
            .build()
        );

        result.put("grabArtifactsAndShootAgain", follower
            .pathBuilder()
            // .addPath(turnSoAsToIntake) // FIXME: Tell pedropathing to do this correctly!
            .addPath(grabArtifactsAgain.getPath(0))
            .addPath(grabArtifactsAgain.getPath(1))
            .addPath(grabArtifactsAgain.getPath(2))
            .addPath(goBackToShootAgain)
            .build()
        );

        result.put("park", follower
            .pathBuilder()
            .addPath(new BezierLine(
                shooting, 
                mirror(new Pose(48, 60, shooting.getHeading()), isRed)
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
        final Subsystem[] subsystems = createSubsystems(hardwareMap);
        shooter = (FlywheelTubeShooter) subsystems[0];
        intake = (CarwashIntake) subsystems[1];
        // drivetrain = (BasicMecanumDrive) subsystems[2];
        shooter.setTelemetry(telemetry);

        final Servo rampPivot = hardwareMap.get(Servo.class, "rampPivot");
        
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
        // follower.setStartingPose(mirror(START_POS.pedroPose(), isRed));

        // Init loop
        while(opModeInInit()) {
            telemetry.addData("Status", "Initialized");
            telemetry.addLine();
            telemetry.addLine(Util.header("Settings"));
            telemetry.addLine();
            telemetry.addData("Toggle isRed", "A");
            telemetry.addData("isRed", isRed);
            telemetry.update();

            if(gamepad1.aWasPressed()) {
                isRed = !isRed;
                paths = createPaths(follower, isRed);
            }
            
        }
        
        waitForStart();
        rampPivot.setPosition(0.58); // Determined emperically
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
            blackboard.put("startPosition", follower.getPose());

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
        intake.intakeGamePieces();
        follower.followPath(paths.get("grabArtifactsAndShoot"), false);

        boolean hasReloaded = false;
        // follower.setMaxPowerScaling(0.5); // Slowing down
        while(follower.isBusy() && opModeIsActive()) {
            if(follower.getChainIndex() == 1 || follower.getChainIndex() == 2) {
                follower.setMaxPower(0.33);
                intake.intakeGamePieces();
                leftBlocker.close();
                rightBlocker.close();
            } else {
                follower.setMaxPower(1.0);
                intake.holdGamePieces();
            }


            follower.update();
            blackboard.put("startPosition", follower.getPose());
            CommandScheduler.getInstance().run();
        }
        // follower.setMaxPowerScaling(1.0);

        // Shooting once again
        emptyClip(motif);

        // Moving to grab artifacts
        // This goes back to shooting afterwards
        intake.intakeGamePieces();
        follower.followPath(paths.get("grabArtifactsAndShootAgain"), false);

        // hasReloaded = false;
        // follower.setMaxPowerScaling(0.5); // Slowing down
        while(follower.isBusy() && opModeIsActive()) {
            if(follower.getChainIndex() == 1 || follower.getChainIndex() == 2) {
                follower.setMaxPower(0.33);
                intake.intakeGamePieces();
                leftBlocker.close();
                rightBlocker.close();
            } else {
                follower.setMaxPower(1.0);
                intake.holdGamePieces();
            }

            follower.update();
            blackboard.put("startPosition", follower.getPose());
            CommandScheduler.getInstance().run();
        }
        // follower.setMaxPowerScaling(1.0);

        // Shooting once again
        emptyClip(motif);

        // Getting leave points
        intake.holdGamePieces();
        shooter.uncharge();
        follower.followPath(paths.get("park"), false);

        while(follower.isBusy() && opModeIsActive()) {
            follower.update();
            blackboard.put("startPosition", follower.getPose());
        }


        // END
        blackboard.put("startPosition", follower.getPose());
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
        runUntilCompleted(shooter.chargeCommand());
        leftBlocker.open();
        rightBlocker.open();
        final ElapsedTime timer = new ElapsedTime(); // FIXME: timeUtil

        // Shooting depth 1
        runUntilCompleted(shooter.fireCommand());

        // Reloading and going
        runUntilCompleted(shooter.chargeCommand());
        
        // Shooting
        intake.intakeGamePieces();
        runUntilCompleted(shooter.fireCommand());

        // Ending
        intake.holdGamePieces();
        leftBlocker.close();
        rightBlocker.close();
        shooter.charge();
        CommandScheduler.getInstance().run();
    }
    
    private void emptyForefrontClip(Motif unused) {
        runUntilCompleted(shooter.chargeCommand());
        final ElapsedTime timer = new ElapsedTime(); // FIXME: timeUtil

        // Shooting depth 1
        runUntilCompleted(shooter.fireCommand());

        // Ending
        intake.holdGamePieces();
        shooter.charge();
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
    
    /**
     * Calculates the x so that the point (x, y) is along the edge of the goal.
     * This is used to find the x-coordinate a robot resting against the goal.
     * 
     * @param y The y coordinate that is along the edge of the goal.
     * @return The x coordinate so that (x, y) is along the edge of the goal.
     */
    private static double goalEdgeXFromY(double y) {
        final double RAMP_WIDTH = 6.75; // Inches
        final double GOAL_LENGTH_Y = 21.75; // Inches along the field y axis, up to the archway
        final double GOAL_LENGTH_X = 22.75; // Inches along the field x axis
        return Util.lerp(
            RAMP_WIDTH, 
            Util.invLerp(72 - GOAL_LENGTH_Y, y, 72), 
            RAMP_WIDTH + GOAL_LENGTH_X
        );
    }

    private final TimeInjectionUtil timeUtil = new TimeInjectionUtil(this);
}
