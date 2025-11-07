package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
// import com.qualcomm.robotcore.hardware
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.HardwareDevice;

import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.Command;

import com.bylazar.configurables.annotations.Configurable;

import java.util.ArrayList;

import org.firstinspires.ftc.teamcode.subsystem.*;
import org.firstinspires.ftc.teamcode.util.Util;
import org.firstinspires.ftc.teamcode.util.ArtifactColor;
import org.firstinspires.ftc.teamcode.util.ArtifactColorRangeSensor;

@Configurable
@TeleOp(group="B - Testing")
public class LittleThingy extends OpMode {
    public static double TRIGGER_PRESSED = 0.1;

    public static double FULL_POWER = -1.0;
    public static double FULL_INTAKE = 1.0;

    public static double FEEDER_FULL = 1.0;
    public static double FEEDER_HOLD = 0.1;
    public static double FEEDER_NIL = 0.0;
    public static double FEEDER_BACK = -1.0;

    
    private boolean wasInjected = false;
    private ArrayList<String> nullDeviceNames = new ArrayList<>();
    private ArrayList<Class<?>> nullDeviceTypes = new ArrayList<>();

    private ArtifactColorRangeSensor rightReload = null;
    private ArtifactColorRangeSensor leftReload = null;

    private FlywheelTubeShooter shooter = null;
    private CarwashIntake intake = null;
    private BasicMecanumDrive drivetrain = null;

    private boolean wasPressingLeftBumper = false;
    private boolean wasPressingLeftTrigger = false;

    private LED rightRed;
    private LED rightGreen;
    private LED leftRed;
    private LED leftGreen;

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
                concat += "    ";

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

            throw new RuntimeException("Cannot find hardware: \n" + concat);
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
        
        rightRed = hardwareMap.tryGet(LED.class, "rightRed");     // Intentionaly not caring if we don't find this
        rightGreen = hardwareMap.tryGet(LED.class, "rightGreen"); // Intentionaly not caring if we don't find this
        
        leftRed = hardwareMap.tryGet(LED.class, "leftRed");     // Intentionaly not caring if we don't find this
        leftGreen = hardwareMap.tryGet(LED.class, "leftGreen"); // Intentionaly not caring if we don't find this

        // Checking that ALL hardware has been found (aka the nullHardware list is empty)
        // If any are not found, an error is thrown stating which.
        throwAFitIfAnyHardwareIsNotFound();

        // Setting all necessary hardware properties
        frontLeftMotor.setDirection(DcMotorEx.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorEx.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorEx.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorEx.Direction.FORWARD);

        // rightShooterMotor.setDirection(DcMotor.Direction.REVERSE);
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
            new ArtifactColorRangeSensor.AlternateColorSensorConst().asColorSensorConst() // Use alternate tuning because wierd
        );
        leftReload = new ArtifactColorRangeSensor(
            leftReloadSensor
            // USe the default tuning
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

    @Override
    public void init() {
        final Subsystem[] subsystems = createSubsystems(hardwareMap);
        shooter = (FlywheelTubeShooter) subsystems[0];
        intake = (CarwashIntake) subsystems[1];
        drivetrain = (BasicMecanumDrive) subsystems[2];

        shooter.setTelemetry(telemetry);

        // shooter.uncharge();
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {

        // Driving the drivetrain            
        drivetrain.mecanumDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

        // Speed change for the drivetrain
        if(gamepad1.left_trigger > 0.1 || gamepad1.right_trigger > 0.1) {
            drivetrain.engageFastMode();
        } else if(gamepad1.left_bumper || gamepad1.right_bumper) {
            drivetrain.engageSlowMode();
        } else {
            drivetrain.engageMiddleMode();
        }

        // Shooting the given color of artifact
        // PUPLE
        if(gamepad2.aWasPressed()) {
            shooter.fireGreen();
        }
        
        if(gamepad2.xWasPressed()) {
            shooter.firePurple();
        }

        // Firing the given side if anything goes wrong
        if(gamepad2.rightStickButtonWasPressed()) {
            shooter.fireRight();
        }
        if(gamepad2.leftStickButtonWasPressed()) {
            shooter.fireLeft();
        }

        // RELOAD
        if(gamepad2.yWasPressed()) {
            shooter.reload();
        }

        // CHARGE
        if(gamepad2.dpadUpWasPressed()) {
            shooter.charge();
        }


        // UNCHARGE
        if(gamepad2.dpadDownWasPressed()) {
            shooter.uncharge();
        } 

        // EJECT
        if(gamepad2.bWasPressed()) {
            shooter.abort();
        }

        // INATKE (hold)
        if(gamepad2.left_trigger > TRIGGER_PRESSED && !wasPressingLeftTrigger) {
            intake.intakeGamePieces();
        } else if(!(gamepad2.left_trigger > TRIGGER_PRESSED) && wasPressingLeftTrigger) {
            intake.holdGamePieces();
        }

        if(gamepad2.left_bumper && !wasPressingLeftBumper) {
            intake.ejectGamePieces();
        } else if(!gamepad2.left_bumper && wasPressingLeftBumper) {
            intake.holdGamePieces();
        }

        // LED
        final ArtifactColor artifactcolorL = leftReload.getColor();
        final ArtifactColor artifactcolorR = rightReload.getColor();
        if(rightRed != null && artifactcolorR == ArtifactColor.PURPLE) {
            rightRed.on();
        } else if(rightRed != null) {
            rightRed.off();
        }
        
        // Coloring the LEDs if at all possible
        if(rightGreen != null && artifactcolorR == ArtifactColor.GREEN) {
            rightGreen.on();
        } else if(rightGreen != null) {
            rightGreen.off();
        }

        if(leftRed != null && artifactcolorL == ArtifactColor.PURPLE) {
            leftRed.on();
        } else if(leftRed != null) {
            leftRed.off();
        }
        
        // Coloring the LEDs if at all possible
        if(leftGreen != null && artifactcolorL == ArtifactColor.GREEN) {
            leftGreen.on();
        } else if(leftGreen != null) {
            leftGreen.off();
        }
        

        // BUTTON PRESSES
        wasPressingLeftBumper = gamepad2.left_bumper;
        wasPressingLeftTrigger = gamepad2.left_trigger > TRIGGER_PRESSED;

        // TELELEMETRY
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

    @Override
    public void stop() {
        CommandScheduler.getInstance().reset();
    }
}