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
import org.firstinspires.ftc.teamcode.util.LinearInterpolator;
import org.firstinspires.ftc.teamcode.util.Util;

@TeleOp(group="B - Testing")
public class AimbotTest extends OpMode {
    public static final double DIST_TOLERANCE = 0.5; // inches
    public static final double ANGLE_TOLERANCE = Math.toRadians(0.25);
    public static final double SPEED_TOLERANCE = 2.5;
    public static final double MIN_ANGLE = Robot.positionToRadians(0);
    public static final double MAX_ANGLE = Math.toRadians(62.5); // Any higher, and the shooting is inaccurate
    public static final double MAX_SPEED = Robot.ticksToInches(2400);

    public static final int BUFFER_SIZE = Float.BYTES * 4 * 1000; // A size 1000 arc

    private FlywheelTubeShooter shooter = null;
    private PivotSubsystem pivot = null;

    private boolean areControlsHidden = true;

    private double targetDist = 0;
    private double newTargetDist = 0;

    public static final int MAJOR_ARCS_INCREMENT = 250;
    public static final int MINOR_ARCS_INCREMENT = 50;
    public static final String MINOR_INDICATOR = ".";

    private int parsedArcs = 0;
    private String readStatus = "";

    private BallisticArcSelection selection = null;
    private BallisticArc arc = null;
    private InputStream stream = null;
    private Thread readFiles = null;

    @Override
    public void init() {
        final Robot robot = new Robot(hardwareMap, Set.of(Device.LEFT_SHOOTER, Device.RAMP_PIVOT));
        shooter = robot.getShooter();
        pivot = robot.getRampPivot();

        // This means that no command will use the same subsystem at the same time.
        CommandScheduler.getInstance().reset();
        CommandScheduler.getInstance().registerSubsystem(shooter, pivot);
        
        // Getting the source selection
        telemetry.setMsTransmissionInterval(33);
        telemetry.addData("Status", "Reading ballistic arcs...");
        telemetry.update();
        
        try {
            createSourceArcs();
        } catch(IOException exception) {
            throw new RuntimeException(exception);
        }
    }

    public double ticksToInches(double ticks) {
        // Determined with some samples and applying a regression using Desmos
        // Because this is experimental, the units will not work out
        final double K = 607.98623;
        final double B = -7.66965e16;
        final double H = -3495.02401;
        final double A = -15.37211;
        return K + B * Math.pow(Math.log(ticks - H), A);
    }

    public double inchesToTicks(double inches) {
        // Determined with some samples and applying a regression using Desmos
        // Because this is experimental, the units will not work out
        final double K = 607.98623;
        final double B = -7.66965e16;
        final double H = -3495.02401;
        final double A = -15.37211;
        return H + Math.exp(Math.pow((inches - K) / B, 1 / A));
    }

    private byte[] readAll(InputStream stream) throws IOException {
        // Reading the stream in increments of PART_SIZE
        final int PART_SIZE = 10; // 1 KiB
        final LinkedList<byte[]> parts = new LinkedList<>();
        int lastLength = -1;

        readerLoop: 
        do {
            final byte[] part = new byte[PART_SIZE];
            int length = stream.read(part);

            if(length == -1) {
                break readerLoop;
            }

            lastLength = length;
            parts.add(part);
        } while(true); 

        // Putting all the parts together
        final byte[] result = new byte[lastLength + (parts.size() - 1) * PART_SIZE];

        int majorIndex = 0;
        for(final byte[] part : parts) {
            for(int minor = 0; minor < part.length && majorIndex + minor < result.length; minor++) {
                result[majorIndex + minor] = part[minor];
            }

            majorIndex += PART_SIZE;
        }

        return result;
    }

    private void createSourceArcs() throws IOException {
        // FIXME: AppUtil prevents unit testing because of RobotCore lib dependence.
        // Reading out the entire stream
        stream = AppUtil
            .getDefContext()
            .getResources()
            .openRawResource(R.raw.arcs);

        // Converting it into something useful
        stream = new BufferedInputStream( stream, BUFFER_SIZE );

        if(!stream.markSupported()) {
            stream.close();
            throw new IOException("Mark is not supported by the arcs.bin input sream.");
        }

        // TODO: Make this select only those within a range of realistic angles and speeds.
        // FIXME: This creates an infinite loop that crashes when the opmode is stopped
        readFiles = new Thread(this::initSelection);
        readFiles.run();
    }

    private void initSelection() {
        try {
            // Getting the arcs
            final Collection<BallisticArc> arcs = BallisticFileIo.readArcs(stream, this::filterArc);
            
            // Ending if we have been interrupted
            if(Thread.interrupted()) {
                stream.close();
                return;
            }
            
            // Creating the selection
            telemetry.clear();
            telemetry.addData("Status", "Sorting the arcs...");
            telemetry.update();
            selection = new BallisticArcSelection(arcs);
            stream.close();
        } catch(IOException exc) {
            throw new RuntimeException(exc);
        }
    }

    @Override
    public void init_loop() {
        if(!readFiles.isAlive()) {
            telemetry.addData("Status", "Intialized");
            telemetry.addLine();
            telemetry.addData("Selection size", selection.size());
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

    private void selectArcs() {
        final BallisticArc longest = selection.minDistance().first();
        final BallisticArc shortest = selection.maxDistance().first();
        final BallisticArcSelection subsel = selection
            .withinDistance(targetDist, DIST_TOLERANCE)
            .minSpeed(SPEED_TOLERANCE)  // First Tiebreaker,  so as to make charging easiest
            .minAngle(ANGLE_TOLERANCE)  // Second Tiebreaker, so as to avoid losing speed at high angles
            .maxDistance();             // Third Tiebreaker,  to get as far as possible

        BallisticArc arc = null;

        // Clamping our results if it is not in the data set
        if(subsel.size() == 0 && targetDist > Criterion.DISTANCE.of(longest)) {
            arc = longest;
        } else if(subsel.size() == 0 && targetDist < Criterion.DISTANCE.of(shortest)) {
            arc = shortest;
        } else {
            arc = subsel.first();
        }

        // Sending this data to the shooter and pivot
        this.arc = arc;
        pivot.runToAngle(Criterion.ANGLE.of(arc), ANGLE_TOLERANCE);
        shooter.charge(Criterion.SPEED.of(arc), true); // NOTE: The second arg should be false for CompetitionTeleop
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
            selectArcs();
        }

        if(gamepad1.xWasPressed()) {
            shooter.fire();
        }
        
        if(gamepad1.bWasPressed()) {
            // shooter.uncharge();
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
            stream.close();
        } catch(IOException exc) {
            // If we get an IOException... that sucks!
            // We would rather just clean up what we can.
        }

        readFiles.interrupt();
        CommandScheduler.getInstance().reset();
    }

    private boolean filterArc(BallisticArc arc) {
        countArc(arc);
        final double theta = Criterion.ANGLE.of(arc);
        
        // Filter based off angle
        if(MIN_ANGLE <= theta && theta <= MAX_ANGLE) {
            return true;
        }

        final double speed = Criterion.SPEED.of(arc);
        return speed <= MAX_SPEED;
    }

    private boolean countArc(BallisticArc arc) {
        boolean update = false;
        parsedArcs++;

        if(parsedArcs % MINOR_ARCS_INCREMENT == 0) {
            readStatus += MINOR_INDICATOR;
            update = true;
        }

        if(parsedArcs % MAJOR_ARCS_INCREMENT == 0) {
            readStatus += parsedArcs;
            update = true;
        }

        if(update) {
            telemetry.addData("Status", "Reading ballistic arcs...");
            telemetry.addLine(readStatus);
            telemetry.update();
        }

        return true;
    }
}
