package org.firstinspires.ftc.teamcode.temp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
// import com.qualcomm.robotcore.hardware
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
@TeleOp(group="B - Testing")
public class LittleThingy extends LinearOpMode {
    public static double FULL_POWER = -0.3;
    public static double FULL_INTAKE = 1.0;

    public static double FEEDER_FULL = 1.0;
    public static double FEEDER_HOLD = 0.1;
    public static double FEEDER_NIL = 0.0;
    public static double FEEDER_BACK = -1.0;

    @Override
    public void runOpMode() {
        final DcMotor rightShooter = hardwareMap.get(DcMotor.class, "rightShooter");
        final DcMotor intake = hardwareMap.get(DcMotor.class, "intake");
        final CRServo rightFeeder = hardwareMap.get(CRServo.class, "rightFeeder");
        final CRServo leftFeeder = hardwareMap.get(CRServo.class, "leftFeeder");
        
        final DcMotor leftFront  = hardwareMap.get(DcMotor.class, "frontLeft");
        final DcMotor leftBack  = hardwareMap.get(DcMotor.class, "backLeft");
        final DcMotor rightFront = hardwareMap.get(DcMotor.class, "frontRight");
        final DcMotor rightBack = hardwareMap.get(DcMotor.class, "backRight");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);

        rightFeeder.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFeeder.setDirection(DcMotorSimple.Direction.FORWARD);

        waitForStart();

        double intakePower = 0;
        double power = FEEDER_NIL;
        boolean leftTriggerWasPressed = false;
        while(opModeIsActive()) {
            if(gamepad2.left_trigger > 0.1f && !leftTriggerWasPressed) {
                rightShooter.setPower(FULL_POWER);

            } else if(gamepad2.b && !(gamepad2.left_trigger > 0.1)) {
                rightShooter.setPower(-FULL_POWER);
            } else if(!(gamepad2.left_trigger > 0.1)) {
                rightShooter.setPower(0);
            }

            leftTriggerWasPressed = gamepad2.left_trigger > 0.1f;

            if(gamepad2.a || gamepad2.x) {
                // leftFeeder.setPower(FEEDER_FULL);
                // rightFeeder.setPower(FEEDER_FULL);
                power = FEEDER_FULL;
            } else if(!gamepad2.right_bumper) {
                power = FEEDER_NIL;// leftFeeder.setPower(FEEDER_NIL);
                // power = // rightFeeder.setPower(FEEDER_NIL);
            }
            
            if(gamepad2.dpad_down) {
                power = FEEDER_BACK;
            }

            if(gamepad2.left_bumper) {
                intakePower = 0;
            } 

            if(gamepad2.right_bumper) {
                intakePower = FULL_INTAKE;
                power = FEEDER_HOLD;
                // leftFeeder.setPower(FEEDER_HOLD);
                // rightFeeder.setPower(FEEDER_HOLD);
            } else {
            }

            rightFeeder.setPower(power);
            leftFeeder.setPower(power);
            intake.setPower(intakePower);
            
            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower  = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower   = axial - lateral + yaw;
            double rightBackPower  = axial + lateral - yaw;

            

            // Send calculated power to wheels
            leftFront.setPower(leftFrontPower);
            rightFront.setPower(rightFrontPower);
            leftBack.setPower(leftBackPower);
            rightBack.setPower(rightBackPower);
        }
    }
}