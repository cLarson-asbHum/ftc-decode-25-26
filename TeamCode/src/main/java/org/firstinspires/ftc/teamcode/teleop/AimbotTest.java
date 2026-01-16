package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.io.BufferedInputStream;
import java.io.ByteArrayInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.Set;
import java.util.function.DoubleUnaryOperator;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.ballistics.BallisticArc;
import org.firstinspires.ftc.teamcode.ballistics.BallisticArcSelection;
import org.firstinspires.ftc.teamcode.ballistics.BallisticArcSelection.Criterion;
import org.firstinspires.ftc.teamcode.ballistics.BallisticFileIo;
import org.firstinspires.ftc.teamcode.hardware.ArtifactColorRangeSensor;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.Robot.Device;
import org.firstinspires.ftc.teamcode.hardware.subsystem.FlywheelTubeShooter;
import org.firstinspires.ftc.teamcode.hardware.subsystem.LinearHingePivot;
import org.firstinspires.ftc.teamcode.hardware.subsystem.PivotSubsystem;
import org.firstinspires.ftc.teamcode.res.R;
import org.firstinspires.ftc.teamcode.util.AimbotManager;
import org.firstinspires.ftc.teamcode.util.LinearInterpolator;
import org.firstinspires.ftc.teamcode.util.Util;

@TeleOp(group="B - Testing")
public class AimbotTest extends OpMode {
    public static final double DIST_TOLERANCE = 0.5; // inches
    public static final double MIN_ANGLE = Robot.positionToRadians(0);
    public static final double MAX_ANGLE = Math.toRadians(62.5); // Any higher, and the shooting is inaccurate
    public static final double MAX_SPEED = Robot.ticksToInches(2400);

    private FlywheelTubeShooter shooter = null;
    private PivotSubsystem pivot = null;

    private boolean areControlsHidden = true;

    private double targetDist = 0;
    private double newTargetDist = 0;

    private AimbotManager aimbot = null;
    private BallisticArc arc = null;

    @Override
    public void init() {
        final Robot robot = new Robot(hardwareMap, Set.of(Device.LEFT_SHOOTER, Device.RAMP_PIVOT));
        shooter = robot.getShooter();
        pivot = robot.getRampPivot();

        // This means that no command will use the same subsystem at the same time.
        CommandScheduler.getInstance().reset();
        CommandScheduler.getInstance().registerSubsystem(shooter, pivot);
        aimbot = new AimbotManager(shooter, pivot);
        
        // Getting the source selection
        telemetry.setMsTransmissionInterval(33);
        telemetry.addData("Status", "Reading ballistic arcs...");
        telemetry.update();

        try {
            aimbot.init(R.raw.arcs, this::filterArc, telemetry);
        } catch (IOException exc) {
            throw new RuntimeException(exc);
        }
    }
    
    @Override
    public void init_loop() {
        if(aimbot.isInitialized()) {
            telemetry.addData("Status", "Intialized");
            telemetry.addLine();
            telemetry.addData("Selection size", aimbot.getSelection().size());
            telemetry.addLine();
            telemetry.addLine("NOTE: The ramp pivot will move upon starting this opmode.");
            telemetry.addLine("          In other words: Watch your hands!");
            telemetry.update();
        }
    }

    @Override
    public void start() {
        // Nothing to do as of yet.
    }

    @Override
    public void loop() {
        // Handling button presses
        if(gamepad1.backWasPressed()) {
            areControlsHidden = !areControlsHidden;
        }

        double increment = 5;
        if(gamepad1.left_trigger > 0.1) {
            increment *= 0.2;
        }

        if(gamepad1.right_trigger > 0.1) {
            increment *= 5;
        }

        if(gamepad1.rightBumperWasPressed()) {
            newTargetDist += increment;
        }
        
        if(gamepad1.leftBumperWasPressed()) {
            newTargetDist -= increment;
        }

        if(gamepad1.yWasPressed()) {
            targetDist = newTargetDist;
            aimbot.followArc(arc = aimbot.selectArc(targetDist, DIST_TOLERANCE));
        }

        if(gamepad1.xWasPressed()) {
            shooter.fire();
        }
        
        if(gamepad1.bWasPressed()) {
            shooter.uncharge();
        }
        
        // Controls
        if(!areControlsHidden) {
            telemetry.addLine(Util.header("Controls (Back to hide)"));
            telemetry.addData("Increment distance", "RB");
            telemetry.addData("Decrement distance", "LB");
            telemetry.addLine();
            telemetry.addData("Large increment", "RT");
            telemetry.addData("Small increment", "LT");
            telemetry.addLine();
            telemetry.addData("Apply distance", "Y");
            telemetry.addData("Fire", "X");
            telemetry.addData("Stop", "B");
        } else {
            telemetry.addLine(Util.header("Controls (Back to show)"));
        }

        telemetry.addLine();
        telemetry.addLine(Util.header("Aimbot"));
        telemetry.addLine();

        if(newTargetDist != targetDist) {
            telemetry.addData(
                "New target", "%.1f in", 
                newTargetDist
            );
            telemetry.addLine();
        }

        
        if(arc != null) {
            telemetry.addLine("Target      ");
            telemetry.addData("  |    dist ",  "%.1f in", Criterion.DISTANCE.of(arc));
            telemetry.addData("  |    angle",  "%.1f°", Math.toDegrees(Criterion.ANGLE.of(arc)));
            telemetry.addData("  |    speed",  "%.1f in s⁻¹", Criterion.SPEED.of(arc));
            telemetry.addData("  \\    time ", "%.3f s", arc.getElapsedTime());
            telemetry.addLine();
        }
        telemetry.addData("Current angle", "%.1f°", Math.toDegrees(pivot.getCurrentAngle()));
        telemetry.addData("Current speed", "%.1f in s⁻¹", shooter.getSpeed());

        CommandScheduler.getInstance().run();
    }

    @Override
    public void stop() {
        try {
            aimbot.close();
        } catch(IOException exc) {
            // If we get an IOException... that sucks!
            // We would rather just clean up what we can.
        }

        CommandScheduler.getInstance().reset();
    }

    private boolean filterArc(BallisticArc arc) {
        final double theta = Criterion.ANGLE.of(arc);
        
        // Filter based off angle
        if(MIN_ANGLE <= theta && theta <= MAX_ANGLE) {
            return true;
        }

        final double speed = Criterion.SPEED.of(arc);
        return speed <= MAX_SPEED;
    }

}
