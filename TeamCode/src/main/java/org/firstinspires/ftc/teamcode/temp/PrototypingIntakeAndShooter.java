package org.firstinspires.ftc.teamcode.temp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
@TeleOp(group="B - Testing")
public class PrototypingIntakeAndShooter extends LinearOpMode {
    public static double FULL_POWER = -1.0;
    public static double FULL_INTAKE = -1.0;

    public static double FEEDER_FULL = 1.0;
    public static double FEEDER_HOLD = 0.1;
    public static double FEEDER_NIL = 0.0;

    @Override
    public void runOpMode() {
        final DcMotor rightShooter = hardwareMap.get(DcMotor.class, "rightShooter");
        final DcMotor intake = hardwareMap.get(DcMotor.class, "intake");
        final DcMotor rightFeeder = hardwareMap.get(DcMotor.class, "rightFeeder");
        final DcMotor leftFeeder = hardwareMap.get(DcMotor.class, "leftFeeder");

        rightFeeder.setDirection(DcMotorSimple.Direction.FORWARD);
        leftFeeder.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        double intakePower = 0;
        while(opModeIsActive()) {
            if(gamepad2.left_trigger > 0.1f) {
                rightShooter.setPower(FULL_POWER);

            } else {
                rightShooter.setPower(0);
            }

            if(gamepad2.y) {
                leftFeeder.setPower(FEEDER_FULL);
                rightFeeder.setPower(FEEDER_FULL);
            } else {
                leftFeeder.setPower(FEEDER_NIL);
                rightFeeder.setPower(FEEDER_NIL);
            }

            if(gamepad2.left_bumper) {
                intakePower = 0;
            } 

            if(gamepad2.right_bumper) {
                intakePower = FULL_INTAKE;
            } else {
                leftFeeder.setPower(FEEDER_HOLD);
                rightFeeder.setPower(FEEDER_HOLD);
            }

            intake.setPower(intakePower);
        }
    }
}