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
import org.firstinspires.ftc.teamcode.util.RampAngleInterpolator;
import org.firstinspires.ftc.teamcode.util.RollingAverage;
import org.firstinspires.ftc.teamcode.util.Util;

@TeleOp(group="B - Testing")
public class PivotDistanceTest extends OpMode {
    public static double SERVO_STEP = 0.02;
    public static double ANGLE_START = Math.toRadians(38);
    public static double ANGLE_STEP = Math.toRadians(5);

    private final ArrayList<String> nullDeviceNames = new ArrayList<>();
    private final ArrayList<Class<?>> nullDeviceTypes = new ArrayList<>();

    private Servo rampPivot = null;
    private DistanceGetter rampDistance = null;
    private AngleGetter rampAngle = null;

    private DistanceUnit units = DistanceUnit.INCH;
    private ArrayList<Double> distances = new ArrayList<>();

    private static enum Mode {
        NO_DATA,
        NEW_DATA;
    }

    private Mode currentMode = Mode.NO_DATA;

    private boolean areControlsHidden = true;

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
                return rampDistanceSensor.getDistance(units);
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

        // NOTE: This is a default tuning, from 18 Dec 2025 at 10:32 AM
        final HashMap<Double, Double> pretunedData = new HashMap<>() {{
            put(3.35, Math.toRadians(38.0));
            put(3.61, Math.toRadians(40.0));
            put(3.66, Math.toRadians(42.0));
            put(3.70, Math.toRadians(44.0));

            put(3.75, Math.toRadians(46.0));
            put(3.78, Math.toRadians(48.0));
            put(3.95, Math.toRadians(50.0));
            put(3.99, Math.toRadians(52.0));

            put(4.10, Math.toRadians(54.0));
            put(4.22, Math.toRadians(56.0));
            put(4.37, Math.toRadians(58.0));
            put(4.49, Math.toRadians(60.0));

            put(4.64, Math.toRadians(62.0));
            put(4.67, Math.toRadians(64.0));
            put(4.69, Math.toRadians(66.0));
            put(4.75, Math.toRadians(68.0));

            put(4.79, Math.toRadians(70.0));
            put(4.82, Math.toRadians(72.0));

            // The following values are just in case we get out of bound values
            put(0.5, Math.toRadians(38.0)); // Minimum possible angle
            put(20.0, Math.toRadians(72.0));  // Maximum possible angle
        }};
        rampAngle = new RampAngleInterpolator(DistanceUnit.INCH, pretunedData, rampDistance); 

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

        final String distanceSuffix = units == DistanceUnit.CM ? "cm" : "in";
        for(int i = 0; i < distances.size(); i++) {
            telemetry.addData(
                String.format("%.2f %s", distances.get(i), distanceSuffix), 
                String.format("%.1f°", Math.toDegrees(ANGLE_START + i * ANGLE_STEP))
            );
        }
    }

    private void changeUnits() {
        final DistanceUnit oldUnits = units;
        final DistanceUnit newUnits = units == DistanceUnit.CM ? DistanceUnit.INCH : DistanceUnit.CM;
        units = newUnits;

        // Convering the recorded distances to the new units
        final ArrayList<Double> newDistances = new ArrayList<>();
        for(final double distance : distances) {
            newDistances.add(newUnits.fromUnit(oldUnits, distance));
        }

        distances = newDistances;
    }

    private void changePivotAngle() {
        double newPosition = rampPivot.getPosition();
        if(gamepad1.dpadUpWasPressed()) {
            newPosition += SERVO_STEP;
        }

        if(gamepad1.dpadDownWasPressed()) {
            newPosition -= SERVO_STEP;
        }

        telemetry.addData("new Pos", newPosition);
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
            changeUnits();
        }

        changePivotAngle();

        // Controls
        if(!areControlsHidden) {
            telemetry.addLine(Util.header("Controls (Back to hide)"));
            telemetry.addData("Change units", "B");
            telemetry.addLine();
            telemetry.addData("Record distance", "A");
            telemetry.addData("Save data", "Y");
            telemetry.addData("Clear data", "X");
            telemetry.addLine();
            telemetry.addData("Raise pivot", "↑");
            telemetry.addData("Lower pivot", "↓");
        } else {
            telemetry.addLine(Util.header("Controls (Back to show)"));
        }
        
        // Data
        final String distanceSuffix = units == DistanceUnit.CM ? "cm" : "in";
        telemetry.addLine();
        telemetry.addLine(Util.header(currentMode.name()));
        telemetry.addLine();
        telemetry.addData("Servo pos", rampPivot.getPosition());
        telemetry.addData("Distance", "%.2f %s", rampDistance.getDistance(units), distanceSuffix);
        telemetry.addData("Expected Angle", "%.1f°", rampAngle.getAngle(AngleUnit.DEGREES));
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