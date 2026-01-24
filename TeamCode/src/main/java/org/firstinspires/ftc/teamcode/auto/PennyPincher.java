package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;

@Disabled
@Autonomous
public class PennyPincher extends LinearOpMode {
    @Override
    public void runOpMode() {
        
        final DcMotor leftFront  = hardwareMap.get(DcMotor.class, "frontLeft");
        final DcMotor leftBack  = hardwareMap.get(DcMotor.class, "backLeft");
        final DcMotor rightFront = hardwareMap.get(DcMotor.class, "frontRight");
        final DcMotor rightBack = hardwareMap.get(DcMotor.class, "backRight");
        final DcMotor shooter = hardwareMap.get(DcMotor.class, "rightShooter");
        final CRServo rightFeeder = hardwareMap.get(CRServo.class, "rightFeeder");
        final CRServo leftFeeder = hardwareMap.get(CRServo.class, "leftFeeder");

        
        // rightShooterMotor.setDirection(DcMotor.Direction.REVERSE);
        rightFeeder.setDirection(DcMotor.Direction.REVERSE);
        leftFeeder.setDirection(DcMotor.Direction.FORWARD);

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        leftFront.setPower(0.3);
        leftBack.setPower(0.3);
        rightFront.setPower(0.3);
        rightBack.setPower(0.3);

        sleep(2000);

        leftFront.setPower(-0.3);
        leftBack.setPower(-0.3);
        rightFront.setPower(-0.3);
        rightBack.setPower(-0.3);


        sleep(50);

        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        // rightBack.setPower(0);

        // // Multifiring
        // rightShooter.setPower(1.0);

        // sleep(5000);

        // rightFeeder.setPower(1.0);
        // leftFeeder.setPower(1.0);

        // sleep(1000);
    }
}