package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
@Autonomous(group="A - Main")
public class EmptyPreloadRed extends LinearOpMode {
    public static double FULL_LEFT_POWER = -0.5;
    public static double FULL_RIGHT_POWER = -0.8;
    public static double INTAKE_FULL = 1.0;

    public static double FEEDER_FULL = 1.0;
    public static double FEEDER_HOLD = 0.1;
    public static double FEEDER_NIL = 0.0;
    
    // public static double INTAKE_FULL = ;
    
    @Override
    public void runOpMode() {
        // final DcMotor rightShooter = hardwareMap.get(DcMotor.class, "rightShooter");
        // final DcMotor intake = hardwareMap.get(DcMotor.class, "intake");
        // final CRServo rightFeeder = hardwareMap.get(CRServo.class, "rightFeeder");
        // final CRServo leftFeeder = hardwareMap.get(CRServo.class, "leftFeeder");
        
        final DcMotor leftFront  = hardwareMap.get(DcMotor.class, "frontLeft");
        final DcMotor leftBack  = hardwareMap.get(DcMotor.class, "backLeft");
        final DcMotor rightFront = hardwareMap.get(DcMotor.class, "frontRight");
        final DcMotor rightBack = hardwareMap.get(DcMotor.class, "backRight");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        
        final DcMotor rightShooter = hardwareMap.get(DcMotor.class, "rightShooter");
        final CRServo leftFeeder = hardwareMap.get(CRServo.class, "leftFeeder");
        final CRServo rightFeeder = hardwareMap.get(CRServo.class, "rightFeeder");
        final DcMotor intake = hardwareMap.get(DcMotor.class, "intake");
        
        rightFeeder.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFeeder.setDirection(DcMotorSimple.Direction.FORWARD);
        
        waitForStart();
        
        // Charging the flywheels to fire left
        rightShooter.setPower(FULL_RIGHT_POWER);
        sleep(2000);
        
        // Moving the robot forwards
        leftFront.setPower(0.3);
        rightFront.setPower(0.3);
        leftBack.setPower(0.3);
        rightBack.setPower(0.3);
        sleep(1000);
        
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
        
        // Feeding the artifacts
        rightFeeder.setPower(FEEDER_FULL);
        sleep(1000);
        
        // Mving the next ones
        intake.setPower(INTAKE_FULL);
        sleep(2000);
        
        // Charging for the left artifacts
        rightShooter.setPower(FULL_LEFT_POWER);
        sleep(1000);
        leftFeeder.setPower(FEEDER_FULL);
        sleep(2000);
        
        // Shutting everything off
        rightShooter.setPower(0.5);
        leftFeeder.setPower(0);
        rightFeeder.setPower(0);
        intake.setPower(0);
        
        sleep(1000);
        rightShooter.setPower(0);
        
        // Moving the robot forwards
        leftFront.setPower(0.3);
        rightFront.setPower(0.3);
        leftBack.setPower(0.3);
        rightBack.setPower(0.3);
        sleep(1000);
        
        
        // Moving the robot left
        leftFront.setPower(-0.6);
        rightFront.setPower(0.6);
        leftBack.setPower(0.6);
        rightBack.setPower(-0.6);
        sleep(1000);
        
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
    }
}