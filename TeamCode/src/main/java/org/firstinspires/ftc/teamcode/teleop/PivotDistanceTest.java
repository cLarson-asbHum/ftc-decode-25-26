package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.PwmControl.PwmRange;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.function.DoubleSupplier;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.AngleGetter;
import org.firstinspires.ftc.teamcode.util.DistanceGetter;
import org.firstinspires.ftc.teamcode.util.LinearInterpolator;
import org.firstinspires.ftc.teamcode.util.RampAngleInterpolator;
import org.firstinspires.ftc.teamcode.util.RollingAverage;
import org.firstinspires.ftc.teamcode.util.Util;

@TeleOp(group="B - Testing")
public class PivotDistanceTest extends OpMode {
    public static double SERVO_STEP = 0.01;
    public static double ANGLE_START = Math.toRadians(37);
    public static double ANGLE_STEP = Math.toRadians(1);

    private final ArrayList<String> nullDeviceNames = new ArrayList<>();
    private final ArrayList<Class<?>> nullDeviceTypes = new ArrayList<>();

    private Servo rampPivot = null;
    private DistanceGetter rampDistance = null;
    private RampAngleInterpolator rampAngle = null;
    private LinearInterpolator inverseAngle = null;

    private DistanceUnit units = DistanceUnit.INCH;
    private ArrayList<Double> distances = new ArrayList<>();

    private static enum Mode {
        NO_DATA,
        NEW_DATA;
    }

    private Mode currentMode = Mode.NO_DATA;

    private boolean areControlsHidden = true;

    private double lastAngle = ANGLE_START;
    private boolean usingPosition = true;

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
        // Get all the necessary hardware
        final DistanceSensor rampDistanceSensor = findHardware(DistanceSensor.class, "rampDistance");
        rampPivot = findHardware(Servo.class, "rampPivot");
        ((ServoImplEx) rampPivot).setPwmRange(new PwmRange(1050, 1950));

        // Throwing if he hardware cannot be found
        throwAFitIfAnyHardwareIsNotFound();

        // Wrapping the hardware with utility classes
        final DoubleSupplier getDistance = new DoubleSupplier() {
            @Override
            public double getAsDouble() {
                // return rampDistanceSensor.getDistance(units);
                return rampPivot.getPosition(); // NOTE: Distance sensor was unreliable, so we'll use the servo
            }
        };
        final RollingAverage rollingAverage = new RollingAverage(
            getDistance, 
            // new double[] { 0.400, 0.24, 0.16, 0.12, 0.08  }
            // new double[]{ 0.4, 0.24, 0.15, 0.09, 0.05, 0.03, 0.02, 0.01, 0.01, 0.05, 0.05 }
            // new double[] {
            //     .4002, .2401, .1441, .0864, .0519, .0311, .0187, .0112, 
            //     .0067, .0040, .0024, .0015, .0009, .0005, .0003, .0002
            // }
            new double[]{
                .200, .162, .131, .106, .084, .069, .056, .045, 
                .036, .029, .024, .019, .016, .013, .010
            }
        );
        rampDistance = (unit) -> rollingAverage.getAsDouble();

        // NOTE: This is a default tuning, from 27 Dec 2025 at 1:28 AM
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
            
            // The following values are just in case we get out of bound values
            put(-0.1, Math.toRadians(37.0)); // Minimum possible angle
            put(1.1, Math.toRadians(73.0));  // Maximum possible angle
        }};
        rampAngle = new RampAngleInterpolator(DistanceUnit.INCH, pretunedData, rampDistance); 
        inverseAngle = rampAngle.inverse();

        telemetry.setMsTransmissionInterval(33);
        // telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() {
        telemetry.addData("Status", "Initialized");
        telemetry.addLine();
        telemetry.addData("Servo step", SERVO_STEP);
        telemetry.addData("Angle start", "%.1f°", Math.toDegrees(ANGLE_START));
        telemetry.addData("Angle step", "%.1f°", Math.toDegrees(ANGLE_STEP));
    }

    private void clearData() {
        currentMode = Mode.NO_DATA;
        this.distances.clear();
    }

    private void handleNoData() {
        clearData();
    }

    private void handleNewData() {
        
        if(gamepad1.xWasPressed()) {
            // Transitioning to no data
            clearData();
        }
        
        if(gamepad1.yWasPressed()) {
            // Tabulating the data
            final HashMap<Double, Double> pairs = new HashMap<>();

            for(int i = 0; i < distances.size(); i++) {
                pairs.put(distances.get(i), ANGLE_START + i * ANGLE_STEP);
            }

            // Updating the angle getter and going back to NO_DATA
            rampAngle = new RampAngleInterpolator(units, pairs, rampDistance);
            clearData();
        }
    
        telemetry.addLine();
        telemetry.addLine(Util.header("Data"));
        telemetry.addLine();

        // final String distanceSuffix = units == DistanceUnit.CM ? "cm" : "in";
        final String distanceSuffix = "pos"; // 
        for(int i = 0; i < distances.size(); i++) {
            telemetry.addData(
                String.format("%.2f %s", distances.get(i), distanceSuffix), 
                String.format("%.1f°", Math.toDegrees(ANGLE_START + i * ANGLE_STEP))
            );
        }
    }

    private void changeUnits() {
        final DistanceUnit oldUnits = units;
        // final DistanceUnit newUnits = units == DistanceUnit.CM ? DistanceUnit.INCH : DistanceUnit.CM;
        final DistanceUnit newUnits = DistanceUnit.INCH; // NOTE: Keep it the same because we're using servo pos, not distance
        units = newUnits;

        // Convering the recorded distances to the new units
        final ArrayList<Double> newDistances = new ArrayList<>();
        for(final double distance : distances) {
            newDistances.add(newUnits.fromUnit(oldUnits, distance));
        }

        distances = newDistances;
    }

    private void changePivotAngle() {
        double newPosition = 0.0;

        if(usingPosition) {
            newPosition = rampPivot.getPosition();
            if(gamepad1.dpadUpWasPressed()) {
                newPosition += SERVO_STEP;
            }

            if(gamepad1.dpadDownWasPressed()) {
                newPosition -= SERVO_STEP;
            }
        } else {
            newPosition = lastAngle;
            if(gamepad1.dpadUpWasPressed()) {
                newPosition += ANGLE_STEP;
            }

            if(gamepad1.dpadDownWasPressed()) {
                newPosition -= ANGLE_STEP;
            }
            newPosition = Util.clamp(Math.toRadians(37), newPosition, Math.toRadians(73));
            lastAngle = newPosition;
            newPosition = inverseAngle.applyAsDouble(newPosition);
        }

        // telemetry.addData("new Pos", newPosition);
        rampPivot.setPosition(Util.clamp(0, newPosition, 1.0));
    }

    private void handleUniversal() {
        // Handling button presses
        if(gamepad1.backWasPressed()) {
            areControlsHidden = !areControlsHidden;
        }

        if(gamepad1.aWasPressed()) {
            currentMode = Mode.NEW_DATA;
            distances.add(rampDistance.getDistance(units));
        }

        if(gamepad1.bWasPressed()) {
            // changeUnits();
            usingPosition = !usingPosition;
        }

        changePivotAngle();

        // Controls
        if(!areControlsHidden) {
            telemetry.addLine(Util.header("Controls (Back to hide)"));
            // telemetry.addData("Change units", "B");
            telemetry.addLine();
            telemetry.addData("Record distance", "A");
            telemetry.addData("Save data", "Y");
            telemetry.addData("Clear data", "X");
            telemetry.addLine();
            telemetry.addData("Change angle units", "B");
            telemetry.addData("Raise pivot", "↑");
            telemetry.addData("Lower pivot", "↓");
        } else {
            telemetry.addLine(Util.header("Controls (Back to show)"));
        }
        
        // Data
        // final String distanceSuffix = units == DistanceUnit.CM ? "cm" : "in";
        final String distanceSuffix = "pos";
        telemetry.addLine();
        telemetry.addLine(Util.header(currentMode.name()));
        telemetry.addLine();

        if(usingPosition) {
            telemetry.addLine("[POS]");
        } else {
            telemetry.addLine("[RAD]");
        }

        telemetry.addData("Servo pos", rampPivot.getPosition());
        telemetry.addData("Distance", "%.2f %s", rampDistance.getDistance(units), distanceSuffix);

        if(usingPosition) {
            telemetry.addData("Expected Angle", "%.1f°", rampAngle.getAngle(AngleUnit.DEGREES));
        } else {
            telemetry.addData("Expected Angle", "%.1f°", Math.toDegrees(lastAngle));
        }
    }

    @Override
    public void loop() {
        handleUniversal();

        switch(currentMode) {
            case NEW_DATA:
                handleNewData();
                break;

            default: 
            case NO_DATA:
                handleNoData();
                break;
        }
    }
}