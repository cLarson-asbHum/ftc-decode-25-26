package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.bylazar.configurables.annotations.Configurable;

import com.arcrobotics.ftclib.command.CommandScheduler;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystem.CarwashIntake;
import org.firstinspires.ftc.teamcode.subsystem.FlywheelTubeShooter;
import org.firstinspires.ftc.teamcode.util.ArtifactColorRangeSensor;
import org.firstinspires.ftc.teamcode.util.Util;
import org.firstinspires.ftc.teamcode.subsystem.BasicMecanumDrive;

@TeleOp(group="B - Testing")
public class SubsystemIntegrationTesting extends LinearOpMode {
    @Override
    public void runOpMode() {
        final DcMotorEx frontLeft  = (DcMotorEx) hardwareMap.get(DcMotor.class, "frontLeft"); // Null if not found
        final DcMotorEx backLeft   = (DcMotorEx) hardwareMap.get(DcMotor.class, "backLeft"); // Null if not found
        final DcMotorEx frontRight = (DcMotorEx) hardwareMap.get(DcMotor.class, "frontRight"); // Null if not found
        final DcMotorEx backRight  = (DcMotorEx) hardwareMap.get(DcMotor.class, "backRight"); // Null if not found

        final DcMotorEx rightShooterMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "rightShooter");
        final CRServo leftFeeder = hardwareMap.get(CRServo.class, "leftFeeder");
        final CRServo rightFeeder = hardwareMap.get(CRServo.class, "rightFeeder");
        final DcMotorEx intakeMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "intake");

        final ColorRangeSensor rightReloadSensor = hardwareMap.get(ColorRangeSensor.class, "rightReload");
        final ColorRangeSensor leftReloadSensor = hardwareMap.get(ColorRangeSensor.class, "leftReload");
        
        frontLeft.setDirection(DcMotorEx.Direction.REVERSE);
        backLeft.setDirection(DcMotorEx.Direction.REVERSE);
        frontRight.setDirection(DcMotorEx.Direction.FORWARD);
        backRight.setDirection(DcMotorEx.Direction.FORWARD);
        
        // rightShooterMotor.setDirection(DcMotor.Direction.REVERSE);
        rightFeeder.setDirection(DcMotor.Direction.REVERSE);
        leftFeeder.setDirection(DcMotor.Direction.FORWARD);

        rightShooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        final ArtifactColorRangeSensor rightReload = new ArtifactColorRangeSensor(
            rightReloadSensor, 
            new ArtifactColorRangeSensor.AlternateColorSensorConst().asColorSensorConst()
        );
        final ArtifactColorRangeSensor leftReload = new ArtifactColorRangeSensor(leftReloadSensor);
        final FlywheelTubeShooter rightShooter = new FlywheelTubeShooter.Builder(rightShooterMotor) 
            .setLeftFeeder(leftFeeder) 
            .setRightFeeder(rightFeeder)
            .setRightReloadClassifier(rightReload)
            .setLeftReloadClassifier(leftReload)
            .build();
        final CarwashIntake intake = new CarwashIntake(intakeMotor);
        final BasicMecanumDrive drivetrain = new BasicMecanumDrive(frontLeft, backLeft, frontRight, backRight);

        CommandScheduler.getInstance().registerSubsystem(rightShooter, intake, drivetrain);

        rightShooter.setTelemetry(telemetry);
        // intake.setTelemetry(telemetry);
        // drivetrain.setTelemtry(telemetry);
        
        telemetry.setMsTransmissionInterval(40);

        waitForStart();

        boolean wasPressingRightTrigger = false;
        boolean wasPressingLeftTrigger = false;
        String persistent = "";

        
        while(opModeIsActive()) {
            final boolean gamepad2_dpadUpWasPressed = gamepad2.dpadUpWasPressed();
            final boolean gamepad2_backWasPressed = gamepad2.backWasPressed();
            final boolean gamepad2_dpadDownWasPressed = gamepad2.dpadDownWasPressed();
            final boolean gamepad2_yWasPressed = gamepad2.yWasPressed();
            final boolean gamepad2_leftBumperWasPressed = gamepad2.leftBumperWasPressed();
            final boolean gamepad2_rightBumperWasPressed = gamepad2.rightBumperWasPressed();
            final boolean gamepad2_xWasPressed = gamepad2.xWasPressed();
            final boolean gamepad2_leftStickWasPressed = gamepad2.leftStickButtonWasPressed();
            final boolean gamepad2_rightStickWasPressed = gamepad2.rightStickButtonWasPressed();
            
            drivetrain.mecanumDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

            if(gamepad2_dpadUpWasPressed /* && !gamepad2.start */) {
                if(gamepad2.dpad_left) {
                    rightShooter.multiFireLeft();
                } else if(gamepad2.dpad_right) {
                    rightShooter.multiFireRight();
                } else {
                    rightShooter.multiFire();
                }
            } else if(gamepad2_dpadUpWasPressed) {
                // CommandScheduler.getInstance().schedule(rightShooter.multiFireCommand());
            }
            
            if(gamepad2_backWasPressed && rightShooter.getStatus() != FlywheelTubeShooter.Status.CHARGING && !gamepad2.start) {
                rightShooter.charge();
                // rightShooterMotor.setPower(FlywheelTubeShooter.FLYWHEEL_CONST.chargedPower);
                persistent += "Charged with " + FlywheelTubeShooter.FLYWHEEL_CONST.chargedPower + "\n";
            } else if(gamepad2_backWasPressed && !gamepad2.start) {
                rightShooter.forceCharged();

                persistent += "Force Charged\n";
            } else if(gamepad2_backWasPressed) {
                CommandScheduler.getInstance().schedule(rightShooter.chargeCommand());
                persistent += "(Command) Charged with " + FlywheelTubeShooter.FLYWHEEL_CONST.chargedPower + "\n";

            }
            
            if(gamepad2_dpadDownWasPressed && !gamepad2.start) {
                rightShooter.uncharge();
                persistent += "UnCharged\n";

            } else if(gamepad2_dpadDownWasPressed) {
                CommandScheduler.getInstance().schedule(rightShooter.unchargeCommand());
                persistent += "(Command) UnCharged\n";
            }
            
            if(gamepad2_yWasPressed && !gamepad2.start) {
                if(gamepad2.dpad_left) {
                    rightShooter.abortLeft();
                    persistent += "Aborted (Left)\n";
                } else if(gamepad2.dpad_right) {
                    rightShooter.abortRight();
                    persistent += "Aborted (Right)\n";
                } else {
                    rightShooter.abort();
                    persistent += "Aborted\n";
                }

            } else if(gamepad2_yWasPressed) {
                CommandScheduler.getInstance().schedule(rightShooter.abortCommand());
                persistent += "(Command) Aborted\n";
            }

            if(gamepad2.right_trigger > 0.1 && !wasPressingRightTrigger && !gamepad2.start) {
                if(gamepad2.dpad_left) {
                    rightShooter.fireLeft();
                    persistent += "Fired (Left)\n";
                } else if(gamepad2.dpad_right) {
                    rightShooter.fireRight();
                    persistent += "Fired (Right)\n";
                } else {
                    rightShooter.fire();
                    persistent += "Fired\n";
                }

            } else if(gamepad2.right_trigger > 0.1 && !wasPressingRightTrigger) {
                CommandScheduler.getInstance().schedule(rightShooter.fireCommand());
                persistent += "(Command) Fired\n";
            }

            
            if(gamepad2_leftStickWasPressed) {
                rightShooter.firePurple();
                persistent += "Fired (Purple)\n";
            }

            if(gamepad2_rightStickWasPressed) {
                rightShooter.fireGreen();
                persistent += "Fired (Green)\n";
            }

            if(gamepad2.left_trigger > 0.1 && !wasPressingLeftTrigger && !gamepad2.start) {
                if(gamepad2.dpad_left) {
                    rightShooter.reloadLeft();
                    persistent += "Reloaded (Left)\n";
                } else if(gamepad2.dpad_right) {
                    rightShooter.reloadRight();
                    persistent += "Reloaded (Right)\n";
                } else {
                    rightShooter.reload();
                    persistent += "Reloaded\n";
                }
            } else if(gamepad2.left_trigger > 0.1 && !wasPressingLeftTrigger) {
                CommandScheduler.getInstance().schedule(rightShooter.reloadCommand());
                persistent += "(Command) Reloaded\n";
            }

            wasPressingLeftTrigger = gamepad2.left_trigger > 0.1;
            wasPressingRightTrigger = gamepad2.right_trigger > 0.1;
            
            if(gamepad2_leftBumperWasPressed/*  && !gamepad2.start */) {
                intake.holdGamePieces();
                persistent += "Held\n";

            } else if(gamepad2_leftBumperWasPressed) {
                // intake.holdGamePiecesCommand();
                persistent += "(Command) Held\n";
            }

            if(gamepad2_rightBumperWasPressed/*  && !gamepad2.start */) {
                intake.intakeGamePieces();
                persistent += "Intaked\n";

            } else if(gamepad2_rightBumperWasPressed) {
                // intake.intakeGamePiecesCommand();
                persistent += "(Command) Intaked\n";
            }

            if(gamepad2_xWasPressed/*  && !gamepad2.start */) {
                intake.ejectGamePieces();
                persistent += "Ejected\n";

            } else if(gamepad2_xWasPressed) {
                // intake.ejectGamePiecesCommand();
                persistent += "(Command) Ejected\n";
            }

            if(gamepad1.left_trigger > 0.1 || gamepad1.right_trigger > 0.1) {
                drivetrain.engageFastMode();
            } else if(gamepad1.left_bumper || gamepad1.right_bumper) {
                drivetrain.engageSlowMode();
            } else {
                drivetrain.engageMiddleMode();
            }

            // if(gamepad2_dpadDownWasPressed) {
            //     persistent += "adsklfh\n";
            // }

            telemetry.addLine();
            telemetry.addLine(Util.header("Gamepad"));
            telemetry.addLine();
            telemetry.addData("dpadDownWasPressed", gamepad2_dpadDownWasPressed);
            telemetry.addData("start", gamepad2.start);
            telemetry.addLine();

            telemetry.addLine();
            telemetry.addLine(Util.header("Colors"));
            telemetry.addLine();
            telemetry.addData("leftReload Artifact", leftReload.getColor());
            telemetry.addData("rightReload Artifact", rightReload.getColor());
            telemetry.addLine();

            telemetry.addLine();
            telemetry.addLine(Util.header("Persistent"));
            telemetry.addLine();
            telemetry.addLine(persistent);
            telemetry.update();

            CommandScheduler.getInstance().run();
        }

        CommandScheduler.getInstance().reset();
    }
}