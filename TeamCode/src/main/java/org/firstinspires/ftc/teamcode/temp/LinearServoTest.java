package org.firstinspires.ftc.teamcode.temp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@TeleOp(group="B - Testing")
public class LinearServoTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        final ServoImplEx rampPivot = (ServoImplEx) hardwareMap.get(Servo.class, "rampPivot");

        while(opModeIsActive() || opModeInInit()) {
            // Far side: 0.22
            // Close: 0.58
            if(gamepad1.a) { 
                rampPivot.setPosition(-gamepad1.left_stick_y);
            }
            if(gamepad2.a) { 
                rampPivot.setPosition(-gamepad2.left_stick_y);
            }

            if(gamepad1.b) { 
                rampPivot.setPosition(-0.333 * gamepad1.left_stick_y);
            }
            if(gamepad2.b) { 
                rampPivot.setPosition(-0.333 * gamepad2.left_stick_y);
            }
            telemetry.addData("Current Position", rampPivot.getPosition());
            telemetry.update();
        }

    }   
}