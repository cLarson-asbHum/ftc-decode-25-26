package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.PwmControl.PwmRange;

// import clarson.ftc.faker.ServoImplExFake;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystem.BlockerSubsystem;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
@TeleOp(group="B - Testing")
public class PrototypingIntakeAndShooter extends LinearOpMode {
    public static double FULL_POWER = -1.0;
    public static double FULL_INTAKE = -1.0;
    public static double EJECT = 1.0;

    public static double FEEDER_FULL = -1.0;
    public static double FEEDER_HOLD = 0.1;
    public static double FEEDER_NIL = 0.0;

    @Override
    public void runOpMode() {
        final DcMotorEx rightShooter = (DcMotorEx) hardwareMap.get(DcMotor.class, "rightShooter");
        final DcMotorEx leftShooter = (DcMotorEx) hardwareMap.get(DcMotor.class, "leftShooter");
        final DcMotor intake = hardwareMap.get(DcMotor.class, "intake");
        final CRServo rightFeeder = hardwareMap.get(CRServo.class, "rightFeeder");
        final CRServo leftFeeder = hardwareMap.get(CRServo.class, "leftFeeder");
        final ServoImplEx leftBlockerServo = (ServoImplEx) hardwareMap.get(Servo.class, "leftBlocker"); 
        final ServoImplEx rightBlockerServo = (ServoImplEx) hardwareMap.get(Servo.class, "rightBlocker"); 

        boolean closed = false;
        boolean rclosed = false;

        leftBlockerServo.setPwmRange(new PwmRange(500, 2500));
        rightBlockerServo.setPwmRange(new PwmRange(500, 2500));
        leftShooter.setDirection(DcMotorSimple.Direction.REVERSE);

        rightFeeder.setDirection(DcMotorSimple.Direction.FORWARD);
        leftFeeder.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.setMsTransmissionInterval(33);

        // TODO: Tune these
        final BlockerSubsystem leftBlocker = new BlockerSubsystem(leftBlockerServo, 0.595, 0.575);
        final BlockerSubsystem rightBlocker = new BlockerSubsystem(rightBlockerServo, 0.550, 0.575);

        waitForStart();

        double intakePower = 0;
        while(opModeIsActive()) {
            final boolean x = gamepad2.xWasPressed();
            if(x && closed) {
                leftBlocker.open();
                closed = !closed;
            } else if(x) {
                leftBlocker.close();
                closed = !closed;
            }
            final boolean y = gamepad2.bWasPressed();
            if(y && rclosed) {
                rightBlocker.open();
                rclosed = !rclosed;
            } else if(y) {
                rightBlocker.close();
                rclosed = !rclosed;
            }

            if(gamepad2.right_trigger > 0.1f) {
                rightShooter.setPower(FULL_POWER);

            } else {
                rightShooter.setPower(0);
            }

            if(gamepad2.left_trigger > 0.1f) {
                leftShooter.setPower(FULL_POWER);

            } else {
                leftShooter.setPower(0);
            }

            if(gamepad2.dpad_left) {
                rightFeeder.setPower(FEEDER_FULL);
            } else {
                rightFeeder.setPower(0.0);
            }
            

            if(gamepad2.dpad_right) {
                leftFeeder.setPower(FEEDER_FULL);
            } else {
                leftFeeder.setPower(0.0);
            }

            if(gamepad2.left_bumper) {
                intakePower = 0;
            } 

            if(gamepad2.right_bumper) {
                intakePower = FULL_INTAKE;
            }

            if(gamepad2.back) {
                intakePower = EJECT;
            }

            intake.setPower(intakePower);

            telemetry.addData("lvel", leftShooter.getVelocity());
            telemetry.addData("rvel", rightShooter.getVelocity());
            telemetry.addData("lb state", leftBlocker.getState());
            telemetry.update();
            leftBlocker.periodic();
            rightBlocker.periodic();
        }
    }
}