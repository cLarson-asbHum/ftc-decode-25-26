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
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.hardware.lynx.LynxModule;

import java.util.ArrayList;
import java.util.List;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystem.*;
import org.firstinspires.ftc.teamcode.util.ArtifactColor;
import org.firstinspires.ftc.teamcode.util.ArtifactColorRangeSensor;
import org.firstinspires.ftc.teamcode.util.DcMotorGroup;
import org.firstinspires.ftc.teamcode.util.KeyPoses;
import org.firstinspires.ftc.teamcode.util.Util;

@Configurable
@TeleOp(group="A - Main")
public class CompetitionTeleop extends OpMode {
    public static double TRIGGER_PRESSED = 0.1;
    
    private ArrayList<String> nullDeviceNames = new ArrayList<>();
    private ArrayList<Class<?>> nullDeviceTypes = new ArrayList<>();

    private ArtifactColorRangeSensor rightReload = null;
    private ArtifactColorRangeSensor leftReload = null;

    private FlywheelTubeShooter shooter = null;
    private CarwashIntake intake = null;
    private BasicMecanumDrive drivetrain = null;
    private Follower follower = null;

    private Servo rampPivot = null;

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
        final DcMotorEx intakeMotor = (DcMotorEx) findHardware(DcMotor.class, "intake");

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
            rightDistanceSensor,
            new ArtifactColorRangeSensor.AlternateColorSensorConst().asColorSensorConst(), // Use alternate tuning because wierd
            new double[] { 0.400, 0.24, 0.16, 0.12, 0.08  }
            // new double[] { 0.60, 0.16, 0.11, 0.08, 0.05  }
            // new double[] {1.00}
        );
        leftReload = new ArtifactColorRangeSensor(
            leftReloadSensor,
            leftDistanceSensor,
            new ArtifactColorRangeSensor.ColorSensorConst(), // USe the default tuning
            new double[] { 0.400, 0.24, 0.16, 0.12, 0.08  }
            // new double[] { 0.40, 0.16, 0.11, 0.08, 0.05  }
            // new double[] {1.00}

        );

        final DcMotorGroup flywheels = new DcMotorGroup(leftShooterMotor, rightShooterMotor);
        final FlywheelTubeShooter rightShooter = new FlywheelTubeShooter.Builder(flywheels) 
            .setLeftFeeder(leftFeederServo) 
            .setRightFeeder(rightFeederServo)
            .setRightReloadClassifier(rightReload)
            .setLeftReloadClassifier(leftReload)
            .build();
        final CarwashIntake intake = new CarwashIntake(intakeMotor);
        final BasicMecanumDrive drivetrain = new BasicMecanumDrive(
            frontLeftMotor, 
            backLeftMotor,
            frontRightMotor,
            backRightMotor
        );

        // This means that no command will use the same subsystem at the same time.
        CommandScheduler.getInstance().registerSubsystem(rightShooter, intake, drivetrain);

        // Return a list of every subsystem that we have created
        return new Subsystem[] { rightShooter, intake, drivetrain };
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

        shooter.setTelemetry(telemetry);

        // Creating the PerdoPathing path follower
        follower = Constants.createFollower(hardwareMap);

        // Getting the rampPivot
        rampPivot = hardwareMap.get(Servo.class, "rampPivot");

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
        rampPivot.setPosition(0.58);

        // Calling this method multiple times on the same reference will
        // cause any calls after the first one to be ignored.
        follower.setStartingPose(startPosition);

        // DEV START: Trying to replicate when artifacts bounce off the intake
        // isRed = true;
        // follower.setStartingPose(KeyPoses.Red.BASE.plus(new Pose(48, 0, 0)));
        // DEV END
    }

    @Override
    public void loop() {

        // Driving the drivetrain            
        drivetrain.mecanumDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        changeDrivetrainSpeed(
            gamepad1.left_trigger > TRIGGER_PRESSED || gamepad1.right_trigger > TRIGGER_PRESSED, // Fast
            gamepad1.left_bumper || gamepad1.right_bumper, // Slow
            gamepad1.left_bumper && gamepad1.right_bumper // Slowest, has precedent over the above
        );      
        
        reverseDrivingDirection(gamepad1.bWasPressed());

        // Parking if the park combination is held
        toggleIsRed(gamepad1.back && gamepad1.a && !wasPressingIsRed);   
        goParkForthwith(
            gamepad1.dpad_up && gamepad1.y && !wasPressingPark,
            gamepad1.dpad_up && gamepad1.y, 
            !(gamepad1.dpad_up && gamepad1.y) && wasPressingPark
        ); // TODO: Have this only work if it is endgame?

        // Going to nearest shooting point if X is held
        goToShootingZone(gamepad1.x, !gamepad1.x && wasPressingX);

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

        // TODO: Fire pattern

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

        // LEDs
        final ArtifactColor artifactcolorL = leftReload.getColor();
        final ArtifactColor artifactcolorR = rightReload.getColor();

        colorLed(rightRed, rightGreen, artifactcolorR);
        colorLed(leftRed, leftGreen, artifactcolorL);

        // BUTTON PRESSES
        wasPressingRightTrigger = gamepad2.right_trigger > TRIGGER_PRESSED;
        wasPressingLeftTrigger = gamepad2.left_trigger > TRIGGER_PRESSED;
        wasPressingLeftBumper = gamepad2.left_bumper;
        wasTogglingAutoReload = gamepad2.back && gamepad2.y;
        wasPressingIsRed = gamepad1.back && gamepad1.a;
        wasPressingX = gamepad1.x;
        wasPressingPark = gamepad1.dpad_up && gamepad1.y;

        // TELELEMETRY
        telemetry.clearAll();
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

    public boolean goToShootingZone(boolean doGoTo, boolean cancel) {
        if(cancel) {
            follower.breakFollowing();
            follower.update();
            return false;
        }

        if(doGoTo && follower != null) {
            // TODO: Calculate shooting position
            // follower.holdPosition(KeyPoses.base(isRed));
            // return true;
        }

        return false;
    }

    public boolean fireBasedOffColor(boolean fireGreen, boolean firePurple) {
        if(fireGreen) {
            shooter.fireGreen();
            return true;
        }
        
        if(firePurple) {
            shooter.firePurple();
            return true;
        }

        // Nothing was fired
        return false;
    }

    public boolean fireBasedOffSide(boolean fireRight, boolean fireLeft) {
        if(fireRight) {
            shooter.fireRight();
            return true;
        }
        if(fireLeft) {
            shooter.fireLeft();
            return true;
        }

        // Nothing was fired
        return false;
    }

    public boolean fireIndiscriminantly(boolean startFiring, boolean cancelFiring) {
        if(startFiring) {
            shooter.multiFire();
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