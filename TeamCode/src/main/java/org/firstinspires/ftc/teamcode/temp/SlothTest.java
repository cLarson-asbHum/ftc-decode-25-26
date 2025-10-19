package org.firstinspires.ftc.teamcode.temp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.bylazar.configurables.annotations.Configurable;

@Disabled
@Configurable
@TeleOp(group="B - Testing")
public final class SlothTest extends LinearOpMode {
    public static long SLEEP_DUR = 1500; // Milliseconds

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.addLine("Sleeping for " + SLEEP_DUR + " milliseconds");
        telemetry.update();

        // try {
            sleep(SLEEP_DUR);
        // } catch (InterruptedException err) {
        //     Just interuppted; do nothing.
        // }

        while(opModeInInit()) {
            telemetry.addData("Status", "Waiting for start");
            // telemetry.addLine("Did Something *cool* 🥳🥳🥳!");
            telemetry.addLine("Did Something *cool🦾🦾🦾🦾!");
            telemetry.addData("Seconds Since Init", getRuntime());
            telemetry.update();
        }

        // waitForStart();

        resetRuntime();
        while(opModeIsActive()) {
            telemetry.addData("Status", "Running");
            telemetry.addData("Seconds Since Init", getRuntime());
            telemetry.update();
        }
    }
}