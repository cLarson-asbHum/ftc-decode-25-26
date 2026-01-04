package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.PwmControl.PwmRange;

import java.io.BufferedInputStream;
import java.io.ByteArrayInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.function.DoubleUnaryOperator;

import org.firstinspires.ftc.teamcode.res.R;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.ballistics.BallisticArc;
import org.firstinspires.ftc.teamcode.ballistics.BallisticArcSelection;
import org.firstinspires.ftc.teamcode.ballistics.BallisticArcSelection.Criterion;
import org.firstinspires.ftc.teamcode.ballistics.BallisticFileIo;
import org.firstinspires.ftc.teamcode.util.ArtifactColorRangeSensor;
import org.firstinspires.ftc.teamcode.util.DcMotorGroup;
import org.firstinspires.ftc.teamcode.util.LinearInterpolator;
import org.firstinspires.ftc.teamcode.util.Util;
import org.firstinspires.ftc.teamcode.subsystem.FlywheelTubeShooter;
import org.firstinspires.ftc.teamcode.subsystem.LinearHingePivot;
import org.firstinspires.ftc.teamcode.subsystem.PivotSubsystem;

@TeleOp(group="B - Testing")
public class AimbotTest extends OpMode {
    
    // public static final double INCHES_TO_TICKS = 1300.0 / 250.0;
    // TODO: Tune this constants!!!
    public static final double INCHES_TO_TICKS = 1; // DEV NOTE: So that we can input ticks, not in/s

    public static final double DIST_TOLERANCE = 0.5; // inches
    public static final double ANGLE_TOLERANCE = Math.toRadians(0.25);
    public static final double SPEED_TOLERANCE = 2.5;

    public static final int BUFFER_SIZE = Float.BYTES * 4 * 1000; // A size 1000 arc

    private ArrayList<String> nullDeviceNames = new ArrayList<>();
    private ArrayList<Class<?>> nullDeviceTypes = new ArrayList<>();    

    private FlywheelTubeShooter shooter = null;
    private DoubleUnaryOperator inchesToTicks = null;
    private PivotSubsystem pivot = null;

    private boolean areControlsHidden = true;

    private double targetDist = 0;
    private double newTargetDist = 0;
    private DcMotorEx shootingMotor = null;

    public static final int MAJOR_ARCS_INCREMENT = 250;
    public static final int MINOR_ARCS_INCREMENT = 50;
    public static final String MINOR_INDICATOR = ".";

    private int parsedArcs = 0;
    private String readStatus = "";

    private BallisticArcSelection selection = null;
    private BallisticArc arc = null;
    private InputStream stream = null;
    private Thread readFiles = null;

    /**
     * Attempts to get the given hardware from the hardwareMap. If it cannot be 
     * found, then it returns null without finding an error.
     * 
     * This method should be used instead of hardwareMap.get() because it allows
     * us to see **all** the hardware that we cannot find.
     * 
     * @return The hardware with that name, or null if it cannot be found.
     */
    private <T extends HardwareDevice> T findHardware(Class<T> hardwareType, String name) {
        final T result = hardwareMap.tryGet(hardwareType, name);

        // Adding it to the list if null
        if(result == null) {
            nullDeviceNames.add(name);
            nullDeviceTypes.add(hardwareType);
        }

        return result;
    }

    /**
     * Throws an exception if any devices are in the nullDeviceNames or 
     * nullDeviceTypes lists. The thrown exception contains the names and types 
     * of all null hardware devices. 
     */
    private void throwAFitIfAnyHardwareIsNotFound() {
        if(nullDeviceNames.size() != 0 || nullDeviceTypes.size() != 0) {
            String concat = "";

            for(int i = 0; i < nullDeviceNames.size() || i < nullDeviceNames.size(); i++) {
                final String name = nullDeviceNames.get(i);
                final Class type = nullDeviceTypes.get(i); 
                concat += "\n    ";

                if(name != null) {
                    concat += '"' + name + '"';
                } else {
                    concat += "[null]";
                }

                concat += " with type ";
                
                if(type != null) {
                    concat += type.getName() + ".class";
                } else {
                    concat += "[null]";
                }
            }

            throw new RuntimeException("Cannot find hardware:" + concat);
        }
    }

    @Override
    public void init() {
        CommandScheduler.getInstance().reset();

        // Find and create all of the hardware. This uses the hardware map. 
        // When using unit tests, the `hardwareMap` field can be set for dependency injection.
        final DcMotorEx rightShooterMotor = (DcMotorEx) findHardware(DcMotor.class, "rightShooter");
        final DcMotorEx leftShooterMotor = (DcMotorEx) findHardware(DcMotor.class, "leftShooter");
        final CRServo rightFeederServo = findHardware(CRServo.class, "rightFeeder");
        final CRServo leftFeederServo = findHardware(CRServo.class, "leftFeeder");

        final ColorRangeSensor rightReloadSensor = findHardware(ColorRangeSensor.class, "rightReload");
        final ColorRangeSensor leftReloadSensor = findHardware(ColorRangeSensor.class, "leftReload");
        final DistanceSensor rightDistanceSensor = findHardware(DistanceSensor.class, "rightDistance");
        final DistanceSensor leftDistanceSensor = findHardware(DistanceSensor.class, "leftDistance");
        
        final ServoImplEx rampPivotServo = (ServoImplEx) findHardware(Servo.class, "rampPivot");

        // Checking that ALL hardware has been found (aka the nullHardware list is empty)
        // If any are not found, an error is thrown stating which.
        throwAFitIfAnyHardwareIsNotFound();

        rightShooterMotor.setDirection(DcMotor.Direction.REVERSE);
        leftShooterMotor.setDirection(DcMotor.Direction.FORWARD);
        rightFeederServo.setDirection(DcMotor.Direction.REVERSE);
        leftFeederServo.setDirection(DcMotor.Direction.FORWARD);

        rampPivotServo.setPwmRange(new PwmRange(1050, 1950));

        // Creating subsystems. 
        // Subsystems represent groups of hardware that achieve ONE function.
        // Subsystems can lead into each other, but they should be able to operate independently 
        // (even if nothing is achieved, per se).
        final ArtifactColorRangeSensor rightReload = new ArtifactColorRangeSensor(
            rightReloadSensor,
            rightDistanceSensor,
            new ArtifactColorRangeSensor.AlternateColorSensorConst().asColorSensorConst(), // Use alternate tuning because wierd
            new double[] { 0.400, 0.24, 0.16, 0.12, 0.08  }
            // new double[] { 0.60, 0.16, 0.11, 0.08, 0.05  }
            // new double[] {1.00}
        );
        final ArtifactColorRangeSensor leftReload = new ArtifactColorRangeSensor(
            leftReloadSensor,
            leftDistanceSensor,
            new ArtifactColorRangeSensor.ColorSensorConst(), // USe the default tuning
            new double[] { 0.400, 0.24, 0.16, 0.12, 0.08  }
            // new double[] { 0.40, 0.16, 0.11, 0.08, 0.05  }
            // new double[] {1.00}

        );

        final DcMotorGroup flywheels = new DcMotorGroup(leftShooterMotor/* , rightShooterMotor */);
        shootingMotor = flywheels;
        inchesToTicks = (inches) -> inches * INCHES_TO_TICKS; // TODO: Tune this
        final FlywheelTubeShooter rightShooter = new FlywheelTubeShooter.Builder(flywheels) 
            .setLeftFeeder(leftFeederServo) 
            .setRightFeeder(rightFeederServo)
            .setRightReloadClassifier(rightReload)
            .setLeftReloadClassifier(leftReload)
            .setTicksToInches((ticks) -> ticks / INCHES_TO_TICKS)
            .setInchesToTicks(inchesToTicks)
            .build();

        this.shooter = rightShooter;

        
        // NOTE: This is a default tuning, from 27 Dec 2025 at 1:28 PM
        final HashMap<Double, Double> pretunedData = new HashMap<>() {{
            final double[] dists = new double[] {
                0.00, 0.03, 0.06, 0.08,   0.11, 0.13, 0.16, 0.18, 
                0.20, 0.24, 0.26, 0.29,   0.31, 0.36, 0.40, 0.42, 
                0.46, 0.49, 0.52, 0.54,   0.56, 0.58, 0.60, 0.62, 
                0.65, 0.70, 0.72, 0.75,   0.77, 0.80, 0.83, 0.86,
                0.88, 0.90, 0.94, 0.98,   1.00
            };
            
            double angle = 37.0;
            for(final double dist : dists) {
                put(dist, Math.toRadians(angle));
                angle++;
            }
        }};
        final LinearInterpolator positionToRadians = new LinearInterpolator(pretunedData);
        final LinearInterpolator radiansToPosition = positionToRadians.inverse();
        pivot = new LinearHingePivot.Builder(rampPivotServo)
            .setPositionToRadians((pos) -> positionToRadians.clampedCalculate(pos))
            .setRadiansToPosition((ang) -> radiansToPosition.clampedCalculate(ang))
            .build();

        telemetry.setMsTransmissionInterval(33);

        // This means that no command will use the same subsystem at the same time.
        CommandScheduler.getInstance().registerSubsystem(rightShooter, pivot);

        // Getting the source selection
        telemetry.addData("Status", "Reading ballistic arcs...");
        telemetry.update();
        try {
            createSourceArcs();
        } catch(IOException exception) {
            throw new RuntimeException(exception);
        }
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
            final Collection<BallisticArc> arcs = BallisticFileIo.readArcs(stream, this::countArcs);
            
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
        arc = selection
            .withinDistance(targetDist, DIST_TOLERANCE)
            .minSpeed(SPEED_TOLERANCE)    // First Tiebreaker,  so as to make charging easiest
            .maxAngle(ANGLE_TOLERANCE)    // Second Tiebreaker, so as to increase shot success probability
            .maxDistance()                // Third Tiebreaker,  to get as far as possible
            .first();      // to get an arc rather than a selection

        // Sending this data to the shooter and pivot
        pivot.runToAngle(Criterion.ANGLE.of(arc), ANGLE_TOLERANCE);
        shooter.charge(Criterion.DISTANCE.of(arc), true); // NOTE: The second arg should be false for CompetitionTeleop
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
            // shooter.fire();
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

    private Boolean countArcs(BallisticArc arc) {
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
