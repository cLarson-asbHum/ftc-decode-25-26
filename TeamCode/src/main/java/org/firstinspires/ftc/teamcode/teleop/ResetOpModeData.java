package org.firstinspires.ftc.teamcode.teleop;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ballistics.BallisticArcSelection;
import org.firstinspires.ftc.teamcode.util.OpModeData;
import org.firstinspires.ftc.teamcode.util.Util;

@TeleOp(group="C - Util")
public final class ResetOpModeData extends OpMode {
    public static final class DefaultOpModeData {
        public static boolean inCompetitonMode = false;
        public static boolean isRed = false;
        public static BallisticArcSelection selection = null;
        public static Pose startPosition = new Pose(72, 72, 0);
        public static Follower follower = null;
    }

    private static final String format = Util.lines(
        "The following options have the given values. Press their shown button to reset them to default",
        "",
        Util.header("Values", 30),
        "",
        "%s\t |  inCompetitonMode: %b",
        "%s\t |  isRed: %b",
        "%s\t |  selection: %s",
        "%s\t |  startPosition: %s",
        "%s\t |  follower: %s",
        ""
    );

    @Override
    public void init() {
        telemetry.setMsTransmissionInterval(100); // 10 Hz
        // telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE); // 10 Hz
    }
    
    @Override
    public void init_loop() {
        if(gamepad1.a || gamepad2.a) OpModeData.inCompetitonMode = DefaultOpModeData.inCompetitonMode;

        if(gamepad1.b || gamepad2.b) OpModeData.isRed = DefaultOpModeData.isRed;

        if(gamepad1.dpad_up || gamepad2.dpad_up) OpModeData.selection = DefaultOpModeData.selection;

        if(gamepad1.dpad_left || gamepad2.dpad_left) OpModeData.startPosition = DefaultOpModeData.startPosition;

        if(gamepad1.dpad_down || gamepad2.dpad_down) OpModeData.follower = DefaultOpModeData.follower;


        // TELEMETRY
        telemetry.addLine(String.format(
            format, 
            "A", OpModeData.inCompetitonMode,
            "B", OpModeData.isRed,
            "↑", nullishToString(OpModeData.selection),
            "←", nullishToString(OpModeData.startPosition),
            "↓", nullishToString(OpModeData.follower)
        ));

        // ↑←→↓
    }

    private String nullishToString(Object nullable) {
        if(nullable == null) {
            return "null";
        }

        return nullable.toString();
    }
    
    @Override
    public void loop() {
        // According to Wikipedia:
        // 
        // > A **loop* is a quasigroup with an identity element; that is an
        // > element, *e*, such that 
        // >
        // > x * e = x and e * x = x for all x in Q
    }
}