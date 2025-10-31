package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.LED;

@Disabled
@TeleOp(name="Turn Off LEDs", group="B - Testing")
public class LedTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        final LED rightRed = hardwareMap.get(LED.class, "rightRed");
        final LED rightGreen = hardwareMap.get(LED.class, "rightGreen");

        final LED leftRed = hardwareMap.get(LED.class, "leftRed");
        final LED leftGreen = hardwareMap.get(LED.class, "leftGreen");

        rightRed.off();
        rightGreen.off();


        leftRed.off();
        leftGreen.off();

        waitForStart();
    }
}