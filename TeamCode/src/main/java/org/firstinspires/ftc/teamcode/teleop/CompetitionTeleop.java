package org.firstinspires.ftc.teamcode.teleop;

// import com.qualcomm.robotcore.hardware
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.Subsystem;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.hardware.lynx.LynxModule;

import java.util.ArrayList;
import java.util.List;

import org.firstinspires.ftc.teamcode.subsystem.*;
import org.firstinspires.ftc.teamcode.util.ArtifactColor;
import org.firstinspires.ftc.teamcode.util.ArtifactColorRangeSensor;
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

    private SwitchableLight rightRed;
    private SwitchableLight rightGreen;
    private SwitchableLight leftRed;
    private SwitchableLight leftGreen;

    private boolean autoReloadEnabled = false;

    private boolean wasPressingRightTrigger = false;
    private boolean wasPressingLeftTrigger = false;
    private boolean wasPressingLeftBumper = false;
    private boolean wasTogglingAutoReload = false;


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

        // Bulk caching
        final List<LynxModule> modules = hardwareMap.getAll(LynxModule.class);

        for(final LynxModule module : modules) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // shooter.uncharge();
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {

        // Driving the drivetrain            
        drivetrain.mecanumDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        changeDrivetrainSpeed(
            gamepad1.left_trigger > TRIGGER_PRESSED || gamepad1.right_trigger > TRIGGER_PRESSED, // Fast
            gamepad1.left_bumper || gamepad1.right_bumper // Slow
        );        

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
            reloadEmptySides(gamepad2.yWasPressed());
        }

        // AUTO RELOAD
        // Performing the auto reload if allowed to AND we are fully charged
        // We check that we are charged so that we don't automaticcally exit, say, 
        // `UNCHARGING` and cause the shooter to instantly speed up again.
        reloadEmptySides(autoReloadEnabled && shooter.getStatus() == ShooterSubsystem.Status.CHARGED);

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

        // TELELEMETRY
        telemetry.addLine();
        telemetry.addLine(Util.header("Settings"));
        telemetry.addLine();
        telemetry.addData("autoReloadEnabled (Back + Y)", autoReloadEnabled);
        telemetry.addLine();
        telemetry.addLine(Util.header("Colors"));
        telemetry.addLine();
        telemetry.addData("leftReload Artifact", artifactcolorL);
        telemetry.addData("rightReload Artifact", artifactcolorR);
        telemetry.addLine();
        // telemetry.update();

        // COMMAND SCHEDULER
        CommandScheduler.getInstance().run();

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
    public boolean changeDrivetrainSpeed(boolean goFast, boolean goSlow) {
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

    public boolean reloadEmptySides(boolean doReload) {
        if(doReload) {
            // Checking which sides are loaded or not
            final boolean leftReloaded = shooter.checkIsLeftReloaded();
            final boolean rightReloaded = shooter.checkIsRightReloaded();

            if(!leftReloaded && !rightReloaded) {
                shooter.reload();
            } 

            if(!leftReloaded && rightReloaded) {
                shooter.reloadLeft();
            }

            if(leftReloaded && !rightReloaded) {
                shooter.reloadRight();
            }
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