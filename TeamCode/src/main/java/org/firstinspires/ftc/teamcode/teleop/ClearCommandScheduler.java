package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.arcrobotics.ftclib.command.CommandScheduler;

@TeleOp(group="C - Util")
public class ClearCommandScheduler extends LinearOpMode {
    @Override
    public void runOpMode() {
        telemetry.addLine("Hit start to clear CommandScheduler");
        telemetry.update();

        waitForStart();

        CommandScheduler.getInstance().reset();
        telemetry.addLine("Finished!");
        telemetry.update();
    }
}