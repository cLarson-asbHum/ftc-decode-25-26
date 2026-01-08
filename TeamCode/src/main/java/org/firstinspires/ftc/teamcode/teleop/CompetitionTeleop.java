package org.firstinspires.ftc.teamcode.teleop;

// import com.qualcomm.robotcore.hardware
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.Subsystem;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.hardware.PwmControl.PwmRange;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.lynx.LynxModule;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.HashMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystem.*;
import org.firstinspires.ftc.teamcode.util.ArtifactColor;
import org.firstinspires.ftc.teamcode.util.ArtifactColorRangeSensor;
import org.firstinspires.ftc.teamcode.util.DcMotorGroup;
import org.firstinspires.ftc.teamcode.util.KeyPoses;
import org.firstinspires.ftc.teamcode.util.LinearInterpolator;
import org.firstinspires.ftc.teamcode.util.Util;

@Configurable
@TeleOp(group="A - Main")
public class CompetitionTeleop extends OpMode {
    public static double TRIGGER_PRESSED = 0.1;

    private ElapsedTime timer = new ElapsedTime();
    private double lastTime = 0;
    
    private ArrayList<String> nullDeviceNames = new ArrayList<>();
    private ArrayList<Class<?>> nullDeviceTypes = new ArrayList<>();

    private ArtifactColorRangeSensor rightReload = null;
    private ArtifactColorRangeSensor leftReload = null;

    private FlywheelTubeShooter shooter = null;
    private CarwashIntake intake = null;
    private BasicMecanumDrive drivetrain = null;
    private LinearHingePivot rampPivot = null;
    private BlockerSubsystem leftBlocker = null;
    private BlockerSubsystem rightBlocker = null;
    private Follower follower = null;


    private SwitchableLight rightRed;
    private SwitchableLight rightGreen;
    private SwitchableLight leftRed;
    private SwitchableLight leftGreen;

    private boolean autoReloadEnabled = false;

    private boolean wasPressingRightTrigger = false;
    private boolean wasPressingLeftTrigger = false;
    private boolean wasPressingLeftBumper = false;
    private boolean wasTogglingAutoReload = false;
    private boolean wasPressingIsRed = false;
    private boolean wasPressingX = false;
    private boolean wasPressingA = false;
    private boolean wasPressingPark = false;

    private Pose startPosition = null;
    private boolean isRed = false;

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

    private Map<Double, Double> getRampPivotTuning() {
        return new HashMap<>() {{
            final double[] dists = new double[] {
                0.00, 0.03, 0.06, 0.08,   0.11, 0.13, 0.16, 0.18, 
                0.20, 0.24, 0.26, 0.29,   0.31, 0.36, 0.40, 0.42, 
                0.46, 0.49, 0.52, 0.54,   0.56, 0.58, 0.60, 0.62, 
                0.65, 0.70, 0.72, 0.75,   0.77, 0.80, 0.83, 0.86,
                0.88, 0.90, 0.94, 0.98,   1.00
            };
            
            double angle = 37.0;
            for(final double dist : dists) {
                put(dist, Math.toRadians(angle));
                angle++;
            }
            
            // The following values are just in case we get out of bound values
            put(-0.1, Math.toRadians(37.001)); // Minimum possible angle
            put(1.1, Math.toRadians(73.001));  // Maximum possible angle
        }};
    }

    public double ticksToInches(double ticks) {
        // Determined with some samples and applying a regression using Desmos
        // Because this is experimental, the units will not work out
        final double K = 607.98623;
        final double B = -7.66965e16;
        final double H = -3495.02401;
        final double A = -15.37211;
        return K + B * Math.pow(Math.log(ticks - H), A);
    }

    public double inchesToTicks(double inches) {
        // Determined with some samples and applying a regression using Desmos
        // Because this is experimental, the units will not work out
        final double K = 607.98623;
        final double B = -7.66965e16;
        final double H = -3495.02401;
        final double A = -15.37211;
        return H + Math.exp(Math.pow((inches - K) / B, 1 / A));
    }

    private Subsystem[] createSubsystems(HardwareMap hardwareMap) {
        // Find and create all of the hardware. This uses the hardware map. 
        // When using unit tests, the `hardwareMap` field can be set for dependency injection.
        final DcMotorEx frontLeftMotor  = (DcMotorEx) findHardware(DcMotor.class, "frontLeft"); // Null if not found
        final DcMotorEx backLeftMotor   = (DcMotorEx) findHardware(DcMotor.class, "backLeft"); // Null if not found
        final DcMotorEx frontRightMotor = (DcMotorEx) findHardware(DcMotor.class, "frontRight"); // Null if not found
        final DcMotorEx backRightMotor  = (DcMotorEx) findHardware(DcMotor.class, "backRight"); // Null if not found

        // FIXME: left shootere disabled because of hardware fault
        final DcMotorEx rightShooterMotor = (DcMotorEx) findHardware(DcMotor.class, "rightShooter");
        final DcMotorEx leftShooterMotor = (DcMotorEx) findHardware(DcMotor.class, "leftShooter");
        final CRServo rightFeederServo = findHardware(CRServo.class, "rightFeeder");
        final CRServo leftFeederServo = findHardware(CRServo.class, "leftFeeder");
        final ServoImplEx leftBlockerServo = (ServoImplEx) findHardware(Servo.class, "leftBlocker");
        final ServoImplEx rightBlockerServo = (ServoImplEx) findHardware(Servo.class, "rightBlocker");
        final DcMotorEx intakeMotor = (DcMotorEx) findHardware(DcMotor.class, "intake");

        final ServoImplEx rampPivotServo = (ServoImplEx) findHardware(Servo.class, "rampPivot");

        final ColorRangeSensor rightReloadSensor = findHardware(ColorRangeSensor.class, "rightReload");
        final ColorRangeSensor leftReloadSensor = findHardware(ColorRangeSensor.class, "leftReload");
        final DistanceSensor rightDistanceSensor = findHardware(DistanceSensor.class, "rightDistance");
        final DistanceSensor leftDistanceSensor = findHardware(DistanceSensor.class, "leftDistance");
        
        rightRed = hardwareMap.tryGet(SwitchableLight.class, "rightRed");     // Intentionaly not caring if we don't find this
        rightGreen = hardwareMap.tryGet(SwitchableLight.class, "rightGreen"); // Intentionaly not caring if we don't find this
        
        leftRed = hardwareMap.tryGet(SwitchableLight.class, "leftRed");     // Intentionaly not caring if we don't find this
        leftGreen = hardwareMap.tryGet(SwitchableLight.class, "leftGreen"); // Intentionaly not caring if we don't find this

        // Checking that ALL hardware has been found (aka the nullHardware list is empty)
        // If any are not found, an error is thrown stating which.
        throwAFitIfAnyHardwareIsNotFound();

        // Setting all necessary hardware properties
        frontLeftMotor.setDirection(DcMotorEx.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorEx.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorEx.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorEx.Direction.FORWARD);

        rightShooterMotor.setDirection(DcMotor.Direction.REVERSE);
        leftShooterMotor.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);
        rightFeederServo.setDirection(DcMotor.Direction.REVERSE);
        leftFeederServo.setDirection(DcMotor.Direction.FORWARD);

        rightBlockerServo.setPwmRange(new PwmRange(500, 2500));
        leftBlockerServo.setPwmRange(new PwmRange(500, 2500));

        // Creating subsystems. 
        // Subsystems represent groups of hardware that achieve ONE function.
        // Subsystems can lead into each other, but they should be able to operate independently 
        // (even if nothing is achieved, per se).
        rightReload = new ArtifactColorRangeSensor(
            rightReloadSensor,
            rightDistanceSensor,
            new ArtifactColorRangeSensor.AlternateColorSensorConst().asColorSensorConst(), // Use alternate tuning because wierd
            new double[] { 0.400, 0.24, 0.16, 0.12, 0.08  }
        );
        leftReload = new ArtifactColorRangeSensor(
            leftReloadSensor,
            leftDistanceSensor,
            new ArtifactColorRangeSensor.ColorSensorConst(), // USe the default tuning
            new double[] { 0.400, 0.24, 0.16, 0.12, 0.08  }
        );

        final LinearInterpolator positionToRadians = new LinearInterpolator(getRampPivotTuning());

        // Creating subsystems. 
        // Subsystems represent groups of hardware that achieve ONE function.
        // Subsystems can lead into each other, but they should be able to operate independently 
        // (even if nothing is achieved, per se).
        final DcMotorGroup flywheels = new DcMotorGroup(leftShooterMotor, rightShooterMotor);
        final FlywheelTubeShooter rightShooter = new FlywheelTubeShooter.Builder(flywheels) 
            .setLeftFeeder(leftFeederServo) 
            .setRightFeeder(rightFeederServo)
            .setRightReloadClassifier(rightReload)
            .setLeftReloadClassifier(leftReload)
            .setTicksToInches(this::ticksToInches)
            .setInchesToTicks(this::inchesToTicks)
            .build();
        final CarwashIntake intake = new CarwashIntake(intakeMotor);
        final BasicMecanumDrive drivetrain = new BasicMecanumDrive(
            frontLeftMotor, 
            backLeftMotor,
            frontRightMotor,
            backRightMotor
        );
        final LinearHingePivot rampPivot = new LinearHingePivot.Builder(rampPivotServo)
            .setPositionToRadians(positionToRadians)
            .setRadiansToPosition(positionToRadians.inverse())
            .build();
        final BlockerSubsystem leftBlocker = new BlockerSubsystem(
            leftBlockerServo, 
            BlockerSubsystem.PositionPresets.LEFT
        );
        final BlockerSubsystem rightBlocker = new BlockerSubsystem(
            rightBlockerServo, 
            BlockerSubsystem.PositionPresets.RIGHT
        );

        // This means that no command will use the same subsystem at the same time.
        // Return a list of every subsystem that we have created
        final Subsystem[] subsystems = new Subsystem[] { 
            rightShooter, 
            intake, 
            drivetrain, 
            leftBlocker, 
            rightBlocker,
            rampPivot
        };
        CommandScheduler.getInstance().registerSubsystem(subsystems);
        return subsystems;
    }

    /**
     * Initializing the opmode. This is not expected to be HardwareFaker 
     * compatible.
     */
    @Override
    public void init() {
        final Subsystem[] subsystems = createSubsystems(hardwareMap);
        shooter = (FlywheelTubeShooter) subsystems[0];
        intake = (CarwashIntake) subsystems[1];
        drivetrain = (BasicMecanumDrive) subsystems[2];
        leftBlocker = (BlockerSubsystem) subsystems[3];
        rightBlocker = (BlockerSubsystem) subsystems[4];
        rampPivot = (LinearHingePivot) subsystems[5];

        shooter.setTelemetry(telemetry);

        // Creating the PerdoPathing path follower
        follower = Constants.createFollower(hardwareMap);

        // Bulk caching
        final List<LynxModule> modules = hardwareMap.getAll(LynxModule.class);

        for(final LynxModule module : modules) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // shooter.uncharge();
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    public void adjustSettings(Gamepad gamepad) {
        // Adjusting whether we are blue or red
        final Boolean blackboardIsRed = (Boolean) blackboard.get("isRed");
        isRed = blackboardIsRed == null ? false : blackboardIsRed.booleanValue();

        if(gamepad.aWasPressed()) {
            isRed = !isRed;
        }

        blackboard.put("isRed", isRed);
        
        // Adjusting where we start the opmode
        startPosition = (Pose) blackboard.get("startPosition");
        if(startPosition == null) {
            startPosition = new Pose(72, 72, 0);
        }
        
        // X
        final double X_STEP = 6;
        if(gamepad.dpadUpWasPressed()) {
            startPosition = startPosition.withX(Util.clamp(0, startPosition.getX() + X_STEP, 144));
        }

        if(gamepad.dpadDownWasPressed()) {
            startPosition = startPosition.withX(Util.clamp(0, startPosition.getX() - X_STEP, 144));
        }
        
        
        // Y
        final double Y_STEP = 6;
        if(gamepad.dpadRightWasPressed()) {
            startPosition = startPosition.withY(Util.clamp(0, startPosition.getY() + Y_STEP, 144));
        }
        
        
        if(gamepad.dpadLeftWasPressed()) {
            startPosition = startPosition.withY(Util.clamp(0, startPosition.getY() - Y_STEP, 144));
        }

        // θ
        final double YAW_STEP = Math.toRadians(15);
        if(gamepad.leftBumperWasPressed()) {
            final double newYaw = AngleUnit.normalizeRadians(startPosition.getHeading() + YAW_STEP);
            startPosition = startPosition.withHeading(newYaw);
        }
        
        
        if(gamepad.rightBumperWasPressed()) {
            final double newYaw = AngleUnit.normalizeRadians(startPosition.getHeading() - YAW_STEP);
            startPosition = startPosition.withHeading(newYaw);
        }

        blackboard.put("startPosition", startPosition);
    }

    public void displaySettings() {
        final String format = Util.lines(
            "",
            "========== Settings ==========",
            "",
            "isRed: %b",
            "",
            "startPos", 
            "    x: %.2f;",
            "    y: %.2f;",
            "    θ: %.2f;°",
            "",
            "========== Controls ==========",
            "",
            "Toggle isRed: A",
            "",
            "Add   X: dpad_up",
            "minus X: dpad_down",
            "",
            "Add   Y: dpad_up",
            "Minus Y: dpad_left",
            "",
            "Add   θ: left_bumper",
            "Minus θ: right_bumper",
            ""
        );

        telemetry.addLine(String.format(
            format, 
            isRed,
            startPosition.getX(),
            startPosition.getY(),
            Math.toDegrees(startPosition.getHeading())
        ));
        telemetry.update();
    }

    @Override
    public void init_loop() {
        adjustSettings(gamepad1);
        displaySettings();
    }

    @Override
    public void start() {
        rampPivot.runToAngle(rampPivot.convertFromPosition(0.66));

        // Calling this method multiple times on the same reference will
        // cause any calls after the first one to be ignored.
        follower.setStartingPose(startPosition);
        timer.reset();
    }

    @Override
    public void loop() {
        final double timestamp = timer.seconds();
        final double deltaTime = timestamp - lastTime;
        lastTime = timestamp;

        // Driving the drivetrain            
        drivetrain.mecanumDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        changeDrivetrainSpeed(
            gamepad1.left_trigger > TRIGGER_PRESSED || gamepad1.right_trigger > TRIGGER_PRESSED, // Fast
            gamepad1.left_bumper || gamepad1.right_bumper, // Slow
            gamepad1.left_bumper && gamepad1.right_bumper // Slowest, has precedent over the above
        );      
        
        reverseDrivingDirection(gamepad1.bWasPressed());

        // Team change
        toggleIsRed(gamepad1.back && gamepad1.a && !wasPressingIsRed);   

        // // Automatic driving
        // goToShootingZone(gamepad1.x && !wasPressingX, gamepad1.x, !gamepad1.x && wasPressingX);
        // goToLoadingZone(gamepad1.a && !wasPressingA, gamepad1.a, !gamepad1.a && wasPressingA);
        // goParkForthwith(
        //     gamepad1.dpad_up && gamepad1.y && !wasPressingPark,
        //     gamepad1.dpad_up && gamepad1.y, 
        //     !(gamepad1.dpad_up && gamepad1.y) && wasPressingPark
        // ); // TODO: Have this only work if it is endgame?

        // Shooting the given color of artifact
        fireBasedOffColor(gamepad2.aWasPressed() /* Green */, gamepad2.xWasPressed() /* Purple */);

        // Firing the given side if anything goes wrong
        fireBasedOffSide(
            gamepad2.rightStickButtonWasPressed(), // Right
            gamepad2.leftStickButtonWasPressed() // Left
        );

        // MULTIFIRE (hold right trigger)
        fireIndiscriminantly(
            gamepad2.right_trigger > TRIGGER_PRESSED && !wasPressingRightTrigger, // Begin
            !(gamepad2.right_trigger > TRIGGER_PRESSED) && wasPressingRightTrigger // Cancel
        );

        // RELOAD
        if(autoReloadEnabled) {
            // Because we assume that if we're reloading manually, something's wrong
            reloadBothSides(gamepad2.yWasPressed());
        } else {
            manualReloadEmptySides(gamepad2.yWasPressed());
        }

        // AUTO RELOAD
        // Performing the auto reload if allowed to AND we are fully charged
        // We check that we are charged so that we don't automaticcally exit, say, 
        // `UNCHARGING` and cause the shooter to instantly speed up again.
        autoReloadEmptySides(autoReloadEnabled && shooter.getStatus() == ShooterSubsystem.Status.CHARGED);

        // Toggling auto reload upon p
        toggleAutoReload(gamepad2.back && gamepad2.y && !wasTogglingAutoReload);

        // CHARGE
        chargeShooter(gamepad2.dpadUpWasPressed());
        
        // UNCHARGE
        unchargeShooter(gamepad2.dpadDownWasPressed());

        // ABORT
        bringArtifactsOutOfShooter(gamepad2.bWasPressed());

        // INATKE (hold)
        intakeFromFloor(
            gamepad2.left_trigger > TRIGGER_PRESSED && !wasPressingLeftTrigger, // Begin
            !(gamepad2.left_trigger > TRIGGER_PRESSED) && wasPressingLeftTrigger // End
        );
        
        ejectToFloor(
            gamepad2.left_bumper && !wasPressingLeftBumper, // Begin
            !gamepad2.left_bumper && wasPressingLeftBumper // End
        );

        // Blocking if we aren't firing
        if(shooter.getFiringState() == FlywheelTubeShooter.FiringState.UNKNOWN) {
            closeBlockers();
        }

        // LEDs
        // final ArtifactColor artifactcolorL = leftReload.getColor();
        // final ArtifactColor artifactcolorR = rightReload.getColor();
        final ArtifactColor artifactcolorR = ArtifactColor.GREEN;
        final ArtifactColor artifactcolorL = ArtifactColor.GREEN;

        // colorLed(rightRed, rightGreen, artifactcolorR);
        // colorLed(leftRed, leftGreen, artifactcolorL);

        // BUTTON PRESSES
        wasPressingRightTrigger = gamepad2.right_trigger > TRIGGER_PRESSED;
        wasPressingLeftTrigger = gamepad2.left_trigger > TRIGGER_PRESSED;
        wasPressingLeftBumper = gamepad2.left_bumper;
        wasTogglingAutoReload = gamepad2.back && gamepad2.y;
        wasPressingIsRed = gamepad1.back && gamepad1.a;
        wasPressingX = gamepad1.x;
        wasPressingA = gamepad1.a;
        wasPressingPark = gamepad1.dpad_up && gamepad1.y;

        // TELELEMETRY
        telemetry.clearAll();
        telemetry.addData("deltaTime (seconds)", deltaTime);
        telemetry.addLine();
        telemetry.addLine(Util.header("Settings"));
        telemetry.addLine();
        telemetry.addData("autoReloadEnabled (Back2 + Y)", autoReloadEnabled);
        telemetry.addLine();
        telemetry.addData("isRed (Back1 + A)", isRed);
        telemetry.addData("startPosition", startPosition);
        
        telemetry.addLine();
        telemetry.addLine(Util.header("Sensors"));
        telemetry.addLine();
        telemetry.addData("leftReload Artifact", artifactcolorL);
        telemetry.addData("rightReload Artifact", artifactcolorR);
        telemetry.addData("currentPose", () -> {
            follower.updatePose();
            return follower.getPose();
        });
        telemetry.addLine();

        // if(telemetry.update()) {}

        // COMMAND SCHEDULER
        CommandScheduler.getInstance().run();

    }

    public boolean toggleIsRed(boolean doToggle) {
        if(doToggle) {
            isRed = !isRed;
            return true;
        }

        return false;
    }

    /**
     * Sets the drivetrain speed to be abnormal when eithe of the 
     * parameters are true. If both `goFast` and `goSlow` are true, this 
     * goes fast. This goes at normal speed if ano only if both `goFast` and 
     * `goSlow` are false.
     * 
     * @param goFast The drivetrain goes fast only when this is true
     * @param goSlow The drivetrain goes fast only when this is false
     * @return Whether the drivetrain speed was not at middle (normal) speed
     */
    public boolean changeDrivetrainSpeed(boolean goFast, boolean goSlow, boolean goSnail) {
        if(goSnail) {
            drivetrain.engageSlowestMode();
            return false;
        }

        if(goFast) {
            drivetrain.engageFastMode();
            return false;
        }
        
        if(goSlow) {
            drivetrain.engageSlowMode();
            return false;
        }
        
        // Normal Speed
        drivetrain.engageMiddleMode();
        return true;
    }

    public boolean goParkForthwith(boolean doStart, boolean doFollow, boolean cancel) {
        if(cancel && follower != null) {
            follower.breakFollowing();
            follower.update();
            return false;            
        }

        if(doStart && follower != null) {
            final Path path = new Path(new BezierLine(follower.getPose(), KeyPoses.base(isRed)));
            path.setConstantHeadingInterpolation(KeyPoses.base(isRed).getHeading());
            follower.followPath(path);
        }
        
        if((doStart || doFollow) && follower != null) {
            follower.update();
            return true;
        }

        return false;
    }

    public boolean goToShootingZone(boolean doStart, boolean doFollow, boolean cancel) {
        if(cancel && follower != null) {
            follower.breakFollowing();
            follower.update();
            return false;            
        }

        if(doStart && follower != null) {
            final Path path = new Path(new BezierLine(follower.getPose(), KeyPoses.base(isRed)));
            path.setConstantHeadingInterpolation(KeyPoses.shooting(isRed).getHeading());
            follower.followPath(path);
        }
        
        if((doStart || doFollow) && follower != null) {
            follower.update();
            return true;
        }

        return false;
    }

    public boolean goToLoadingZone(boolean doStart, boolean doFollow, boolean cancel) {
        if(cancel && follower != null) {
            follower.breakFollowing();
            follower.update();
            return false;            
        }

        if(doStart && follower != null) {
            final Path path = new Path(new BezierLine(follower.getPose(), KeyPoses.base(isRed)));
            path.setConstantHeadingInterpolation(KeyPoses.loading(isRed).getHeading());
            follower.followPath(path);
        }
        
        if((doStart || doFollow) && follower != null) {
            follower.update();
            return true;
        }

        return false;
    }

    public boolean openBlockers(FlywheelTubeShooter.FiringState firingState) {
        switch(firingState) {
            case FIRING_BOTH:
                leftBlocker.open();
                rightBlocker.close();
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

    public boolean closeBlockers() {
        final boolean closeLeft = leftBlocker.close();
        final boolean closeRight = rightBlocker.close();

        return closeLeft || closeRight;
    }

    public boolean fireBasedOffColor(boolean fireGreen, boolean firePurple) {
        if(fireGreen) {
            shooter.fireGreen();
            openBlockers(shooter.getFiringState());
            return true;
        }
        
        if(firePurple) {
            shooter.firePurple();
            openBlockers(shooter.getFiringState());
            return true;
        }

        // Nothing was fired
        closeBlockers();
        return false;
    }

    public boolean fireBasedOffSide(boolean fireRight, boolean fireLeft) {
        if(fireRight) {
            shooter.fireRight();
            openBlockers(shooter.getFiringState());
            return true;
        }
        if(fireLeft) {
            shooter.fireLeft();
            openBlockers(shooter.getFiringState());
            return true;
        }

        // Nothing was fired
        closeBlockers();
        return false;
    }

    public boolean fireIndiscriminantly(boolean startFiring, boolean cancelFiring) {
        if(startFiring) {
            shooter.multiFire();
            openBlockers(shooter.getFiringState());
            return true;
        }

        if(cancelFiring) {
            shooter.charge();
        } 

        return false;
    }

    public boolean reloadBothSides(boolean doReload) {
        if(doReload) {
            shooter.reload();
            return true;
        }

        return false;
    }

    public boolean manualReloadEmptySides(boolean doReload) {
        if(doReload) {
            shooter.reloadEmpty();
            return true;
        }

        return false;
    }

    public boolean toggleAutoReload(boolean doToggle) {
        if(doToggle) {
            autoReloadEnabled = !autoReloadEnabled;
            return true;
        }

        return false;
    }

    public boolean autoReloadEmptySides(boolean doReload) {
        if(doReload) {
            shooter.autoReload();
            return true;
        }

        return false;
    }

    public boolean chargeShooter(boolean doCharge) {
        if(doCharge) {
            shooter.charge();
            return true;
        }

        return false;
    }

    public boolean unchargeShooter(boolean doUncharge) {
        if(doUncharge) {
            shooter.uncharge();
            return true;
        } 

        return false;
    }

    public boolean bringArtifactsOutOfShooter(boolean doBringBack) {
        if(doBringBack) {
            shooter.abort();
            return true;
        }

        return false;
    }

    public boolean intakeFromFloor(boolean startIntake, boolean cancelIntake) {
        if(startIntake) {
            intake.intakeGamePieces();
            return true;
        }
        
        if(cancelIntake) {
            intake.holdGamePieces();
        } 

        return false;
    }

    public boolean ejectToFloor(boolean startEject, boolean cancelEject) {
        if(startEject) {
            intake.ejectGamePieces();
            return true;
        } 
        
        if(cancelEject) {
            intake.holdGamePieces();
        }

        return false;
    }

    public boolean reverseDrivingDirection(boolean doReverse) {
        if(doReverse) {
            drivetrain.reverse();
            return true;
        }

        return false;
    }

    public boolean colorLed(SwitchableLight redLed, SwitchableLight greenLed, ArtifactColor color) {
        if(greenLed == null && redLed == null) {
            return false;
        }

        switch (color) {
            case PURPLE:
                nullSafeLedEnable(greenLed, false);
                nullSafeLedEnable(redLed, true);
                return true;

            case GREEN:
                nullSafeLedEnable(greenLed, true);
                nullSafeLedEnable(redLed, false);
                return true;

            case UNKNOWN:
            default:
                nullSafeLedEnable(greenLed, false);
                nullSafeLedEnable(redLed, false);
                return false;
        }
    }

    public boolean nullSafeLedEnable(SwitchableLight led, boolean enable) {
        if(led == null) {
            return false;
        }

        led.enableLight(enable);
        return true;
    }

    @Override
    public void stop() {
        CommandScheduler.getInstance().reset();
    }
}