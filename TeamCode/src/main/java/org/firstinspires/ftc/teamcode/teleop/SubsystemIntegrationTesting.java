package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.bylazar.configurables.annotations.Configurable;

import com.arcrobotics.ftclib.command.CommandScheduler;

import org.firstinspires.ftc.teamcode.subsystem.CarwashIntake;
import org.firstinspires.ftc.teamcode.subsystem.FlywheelTubeShooter;
import org.firstinspires.ftc.teamcode.subsystem.BasicMecanumDrive;

@Configurable
@TeleOp(group="B - Testing")
public class SubsystemIntegrationTesting extends LinearOpMode {
    @Override
    public void runOpMode() {
        final DcMotorEx frontLeft  = (DcMotorEx) hardwareMap.get(DcMotor.class, "frontLeft"); // Null if not found
        final DcMotorEx backLeft   = (DcMotorEx) hardwareMap.get(DcMotor.class, "backLeft"); // Null if not found
        final DcMotorEx frontRight = (DcMotorEx) hardwareMap.get(DcMotor.class, "frontRight"); // Null if not found
        final DcMotorEx backRight  = (DcMotorEx) hardwareMap.get(DcMotor.class, "backRight"); // Null if not found

        final DcMotorEx rightShooterMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "rightShooter");
        final DcMotorEx intakeMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "intake");

        final FlywheelTubeShooter rightShooter = new FlywheelTubeShooter(rightShooterMotor);
        final CarwashIntake intake = new CarwashIntake(intakeMotor);
        final BasicMecanumDrive drivetrain = new BasicMecanumDrive(frontLeft, backLeft, frontRight, backRight);

        CommandScheduler.getInstance().registerSubsystem(rightShooter, intake, drivetrain);


        waitForStart();

        boolean wasPressingRightTrigger = false;

        while(opModeIsActive()) {
            drivetrain.mecanumDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

            if(gamepad2.dpadUpWasPressed() /* && !gamepad2.start */) {
                rightShooter.multiFire();
            } else if(gamepad2.dpadUpWasPressed()) {
                // CommandScheduler.getInstance().schedule(rightShooter.multiFireCommand());
            }
            
            if(gamepad2.backWasPressed() && !gamepad2.start) {
                rightShooter.charge();
            } else if(gamepad2.backWasPressed()) {
                CommandScheduler.getInstance().schedule(rightShooter.chargeCommand());
            }
            
            if(gamepad2.dpadDownWasPressed() && !gamepad2.start) {
                rightShooter.uncharge();
            } else if(gamepad2.dpadDownWasPressed()) {
                CommandScheduler.getInstance().schedule(rightShooter.unchargeCommand());
            }
            
            if(gamepad2.aWasPressed() && !gamepad2.start) {
                rightShooter.abort();
            } else if(gamepad2.aWasPressed()) {
                CommandScheduler.getInstance().schedule(rightShooter.abortCommand());
            }

            if(gamepad2.right_trigger > 0.1 && !wasPressingRightTrigger && !gamepad2.start) {
                rightShooter.fire();
            } else if(gamepad2.right_trigger > 0.1 && !wasPressingRightTrigger) {
                CommandScheduler.getInstance().schedule(rightShooter.fireCommand());
            }

            wasPressingRightTrigger = gamepad2.right_trigger > 0.1;
            
            if(gamepad2.leftBumperWasPressed()/*  && !gamepad2.start */) {
                intake.holdGamePieces();
            } else if(gamepad2.leftBumperWasPressed()) {
                // intake.holdGamePiecesCommand();
            }

            if(gamepad2.rightBumperWasPressed()/*  && !gamepad2.start */) {
                intake.intakeGamePieces();
            } else if(gamepad2.rightBumperWasPressed()) {
                // intake.intakeGamePiecesCommand();
            }

            if(gamepad2.xWasPressed()/*  && !gamepad2.start */) {
                intake.ejectGamePieces();
            } else if(gamepad2.xWasPressed()) {
                // intake.ejectGamePiecesCommand();
            }

            CommandScheduler.getInstance().run();
        }

        CommandScheduler.getInstance().reset();
    }
}