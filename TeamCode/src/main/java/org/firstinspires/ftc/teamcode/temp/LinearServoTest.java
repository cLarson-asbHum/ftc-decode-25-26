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
            if(gamepad1.a) { 
                rampPivot.setPosition(-gamepad1.left_stick_y);
            }
            if(gamepad2.a) { 
                rampPivot.setPosition(-gamepad2.left_stick_y);
            }
        }
    }   
}