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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

import static org.firstinspires.ftc.teamcode.util.ArtifactColor.PURPLE;

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
import org.firstinspires.ftc.teamcode.subsystem.BasicMecanumDrive ;
import org.firstinspires.ftc.teamcode.subsystem.CarwashIntake;
import org.firstinspires.ftc.teamcode.subsystem.FlywheelTubeShooter;
import org.firstinspires.ftc.teamcode.subsystem.ShooterSubsystem.Status;
import org.firstinspires.ftc.teamcode.temp.TimeInjectionUtil;
import org.firstinspires.ftc.teamcode.util.ArtifactColor;
import org.firstinspires.ftc.teamcode.util.ArtifactColorRangeSensor;
import org.firstinspires.ftc.teamcode.util.ConfigPose;
import org.firstinspires.ftc.teamcode.util.MotifGetter;
import org.firstinspires.ftc.teamcode.util.MotifGetter.Motif;
import org.firstinspires.ftc.teamcode.util.MotifWebcam;
import org.firstinspires.ftc.teamcode.util.RrCoordinates;
import org.firstinspires.ftc.teamcode.util.Util;
import org.firstinspires.ftc.teamcode.util.WrapConcurrentCommand;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@Disabled
@Configurable
@Autonomous(name = "Auto2: Blue Rippley", group = "A - Main")
public class Rippley extends LinearOpMode {
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

    public static ConfigPose SHOOTING_POS = new ConfigPose(
        52,
        102, // A little higher than the jigsaw so we are facing the goal
        Math.toRadians(315)
    );

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
        final CRServo rightFeederServo = findHardware(CRServo.class, "rightFeeder");
        final CRServo leftFeederServo = findHardware(CRServo.class, "leftFeeder");
        final DcMotorEx intakeMotor = (DcMotorEx) findHardware(DcMotor.class, "intake");

        final ColorRangeSensor rightReloadSensor = findHardware(ColorRangeSensor.class, "rightReload");
        final ColorRangeSensor leftReloadSensor = findHardware(ColorRangeSensor.class, "leftReload");
        
        // rightRed = hardwareMap.tryGet(SwitchableLight.class, "rightRed");     // Intentionaly not caring if we don't find this
        // rightGreen = hardwareMap.tryGet(SwitchableLight.class, "rightGreen"); // Intentionaly not caring if we don't find this
        
        // leftRed = hardwareMap.tryGet(SwitchableLight.class, "leftRed");     // Intentionaly not caring if we don't find this
        // leftGreen = hardwareMap.tryGet(SwitchableLight.class, "leftGreen"); // Intentionaly not caring if we don't find this

        // Checking that ALL hardware has been found (aka the nullHardware list is empty)
        // If any are not found, an error is thrown stating which.
        throwAFitIfAnyHardwareIsNotFound();

        // Setting all necessary hardware properties
        frontLeftMotor.setDirection(DcMotorEx.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorEx.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorEx.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorEx.Direction.FORWARD);

        rightShooterMotor.setDirection(DcMotor.Direction.REVERSE);
        // rightShooterMotor.setVelocityPIDFCoefficients(
        //     -rightShooterMotor.getPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER).p, 
        //     -rightShooterMotor.getPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER).i, 
        //     -rightShooterMotor.getPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER).d, 
        //     -rightShooterMotor.getPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER).f 
        // );
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

        final FlywheelTubeShooter rightShooter = new FlywheelTubeShooter.Builder(rightShooterMotor) 
            .setLeftFeeder(leftFeederServo) 
            .setRightFeeder(rightFeederServo)
            .setRightReloadClassifier(rightReload)
            .setLeftReloadClassifier( leftReload)
            .build();
        final CarwashIntake intake = new CarwashIntake(intakeMotor);
        // final BasicMecanumDrive drivetrain = new BasicMecanumDrive(
        //     frontLeftMotor, 
        //     backLeftMotor,
        //     frontRightMotor,
        //     backRightMotor
        // );

        // This means that no command will use the same subsystem at the same time.
        CommandScheduler.getInstance().registerSubsystem(rightShooter, intake);

        // Return a list of every subsystem that we have created
        return new Subsystem[] { rightShooter, intake, drivetrain };
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

        final Path grabArtifacts = new Path(new BezierCurve(
            mirror(new Pose(40.463, 102.942 - 7), isRed),
            mirror(new Pose(89.851, 68.430 - 7), isRed),
            mirror(new Pose(12.496, 100.562 - 7), isRed),
            mirror(new Pose(25.289, 83.306 - 7), isRed),
            mirror(new Pose(23.207, 71.405 - 7), isRed),
            mirror(new Pose(25.207, 73.488 - 7), isRed) // End point
        ));
        final Path goBackToShoot = new Path(new BezierLine(
            mirror(new Pose(25.207, 73.488), isRed), 
            shooting
        ));
        final Path turnSoAsToIntake = new Path(new BezierLine(
            shooting,
            new Pose(shooting.getX(), shooting.getY(), isRed ? 0 : Math.PI)
        ));

        // turnSoAsToIntake.setLinearHeadingInterpolation(shooting.getHeading(), isRed ? 0 : Math.PI);
        grabArtifacts.setConstantHeadingInterpolation(/* start.getHeading(),  */isRed ? 0 : Math.PI);
        goBackToShoot.setLinearHeadingInterpolation(isRed ? 0 : Math.PI, shooting.getHeading());
        result.put("grabArtifactsAndShoot", follower
            .pathBuilder()
            // .addPath(turnSoAsToIntake) // FIXME: Tell pedropathing to do this correctly!
            .addPath(grabArtifacts)
            .addPath(goBackToShoot)
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
        drivetrain = (BasicMecanumDrive) subsystems[2];
        shooter.setTelemetry(telemetry);

        final Servo rampPivot = hardwareMap.get(Servo.class, "rampPivot");
        
        // Creating the webcam
        final WebcamName obeliskViewerCam = hardwareMap.get(WebcamName.class, "obeliskViewer");
        final MotifWebcam motifGetter = new MotifWebcam(obeliskViewerCam, CAMERA_YAW_OFFSET);

        setManualExposure(motifGetter, GAIN, EXPOSURE_MS);

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
            follower.update();
            blackboard.put("startPosition", follower.getPose());

            // Slowing down and reloading when in the correct part
            if(follower.atPose(new Pose(), 10, 5) && !hasReloaded) {
                shooter.reload();
                hasReloaded = true;
            }

            // Speeding Back up when out of range
            if(!follower.atPose(new Pose(), 10, 7) && hasReloaded) {
                follower.setMaxPower(1.0); // Slowing down
            }
        }
        // follower.setMaxPowerScaling(1.0);


        // Going back to shooting
        // follower.followPath(paths.get("goAndShoot"), false);

        // while(follower.isBusy() && opModeIsActive()) {
        //     follower.update();
        //     blackboard.put("startPosition", follower.getPose());
        // 
        // }

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
        CommandScheduler.getInstance().reset();

        // MoveForward(21);
        // Turn(90);
        // MoveForward(45);
        // Turn(90);
        // MoveForward(30);
        // Turn(-100);

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

    private void emptyClip(MotifGetter.Motif motif) {
        
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
